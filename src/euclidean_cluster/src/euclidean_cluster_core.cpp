#include "euclidean_cluster_core.h"

EuClusterCore::EuClusterCore(ros::NodeHandle &nh)
{
    seg_distance_ = {15, 30, 45, 60};              //划分区域
    cluster_distance_ = {0.2, 1.0, 1.5, 2.0, 2.5}; //不同距离区域使用不同的聚类半径阈值

    sub_point_cloud_ = nh.subscribe("/filtered_points_no_ground", 5, &EuClusterCore::point_cb, this);
    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);

    ros::spin();
}

EuClusterCore::~EuClusterCore() {}

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, jsk_recognition_msgs::BoundingBoxArray &obj_list)
{
    // 读取背景颜色参数
    float g_min_x,g_min_y,g_min_z;
    float g_max_x,g_max_y,g_max_z;
	ros::param::get("/min_x", g_min_x);
	ros::param::get("/min_y", g_min_y);
    ros::param::get("/min_z", g_min_z);
    ros::param::get("/max_x", g_max_x);
    ros::param::get("/max_y", g_max_y);
    ros::param::get("/max_z", g_max_z);
    // float temp1;
    // ros::param::get("/dynamic_reconfigure_node/min_x", temp1);
    // ROS_INFO("change? %f",temp1);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    // make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    //欧式距离分类
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    std::vector<pcl::PointIndices> local_indices;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance); //设置近邻搜索的搜索半径
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);          //设置最小聚类尺寸
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    int id = 0;
    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        jsk_recognition_msgs::BoundingBox obj_info;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

        // point_cloud_header_=
        obj_info.header = point_cloud_header_;

        // 提取点云
        pcl::copyPointCloud(*in_pc, local_indices[i], *temp); //注意cloud2D没有高度信息，无法建立bounding box
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*temp, minPt, maxPt);

        obj_info.pose.position.x = (maxPt.x + minPt.x) / 2;
        obj_info.pose.position.y = (maxPt.y + minPt.y) / 2;
        obj_info.pose.position.z = (maxPt.z + minPt.z) / 2;
        // TODO:orientation 四元素+点云配准

        // tf2::Quaternion quaternion_tf2;
        // quaternion_tf2.setRPY(0.7875, 0, 0);
        // geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);

        // float yaw = 0.52;
        // float pitch = 0.52;
        // float roll = 0.78;

        // double cy = cos(yaw * 0.5);
        // double sy = sin(yaw * 0.5);
        // double cp = cos(pitch * 0.5);
        // double sp = sin(pitch * 0.5);
        // double cr = cos(roll * 0.5);
        // double sr = sin(roll * 0.5);

        // obj_info.pose.orientation.w = cy * cp * cr + sy * sp * sr;
        // obj_info.pose.orientation.x = cy * cp * sr - sy * sp * cr;
        // obj_info.pose.orientation.y = sy * cp * sr + cy * sp * cr;
        // obj_info.pose.orientation.z = sy * cp * cr - cy * sp * sr;

        obj_info.dimensions.x = maxPt.x - minPt.x;
        obj_info.dimensions.y = maxPt.y - minPt.y;
        obj_info.dimensions.z = maxPt.z - minPt.z;

        // ROS_INFO("Get Param[%f, %f, %f]", g_min_x, g_min_y, g_min_z);

        if (obj_info.dimensions.z < 1.4 || obj_info.dimensions.z > 1.7)
            continue;

        id++;
        obj_info.label = id;
        obj_list.boxes.push_back(obj_info);
    }
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, jsk_recognition_msgs::BoundingBoxArray &obj_list)
{
    // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    // in this way, the points farther in the pc will also be clustered

    // 0 => 0-15m d=0.5
    // 1 => 15-30 d=1
    // 2 => 30-45 d=1.6
    // 3 => 45-60 d=2.1
    // 4 => >60   d=2.6
    //将点云根据距离划分为5组
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于120m, 忽略该点
        if (origin_distance >= 120)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point); // push_back: 向数据结构中添加元素
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    std::vector<pcl::PointIndices> final_indices;
    std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list);
    }
}

void EuClusterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header_ = in_cloud_ptr->header;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // down sampling the point cloud before cluster
    // voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);
    filtered_pc_ptr = current_pc_ptr;
    jsk_recognition_msgs::BoundingBoxArray BOXS;
    cluster_by_distance(filtered_pc_ptr, BOXS);

    BOXS.header = point_cloud_header_;
    pub_bounding_boxs_.publish(BOXS);
}