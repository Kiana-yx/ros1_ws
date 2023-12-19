#include "lidar_core.h"
#include "common.h"

LidarCore::LidarCore(ros::NodeHandle &nh)
{
    string intrinsic_path, extrinsic_path;
    cout << "Get the parameters from the launch file" << endl;
    if (!ros::param::get("intrinsic_path", intrinsic_path))
    {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path))
    {
        cout << "Can not get the value of extrinsic_path" << endl;
        exit(1);
    }

    getIntrinsic(intrinsic_path, intrinsic);
    getDistortion(intrinsic_path, distortion);
    getExtrinsic(extrinsic_path, extrinsic);
    sub_point_cloud_ = nh.subscribe("/livox/lidar", 10, &LidarCore::callback, this);
    pub_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
    pub_depthImage_ = nh.advertise<sensor_msgs::Image>("/depthImage", 10);

    // 程序到达ros::spin()之前按照一系列规则，设定一系列话题订阅者
    // ros::spin()使订阅者们可以开始接受话题，进入回调函数
    ros::spin();
}

LidarCore::~LidarCore() {}

void LidarCore::Spin()
{
}

void LidarCore::viewFalseColor(cv::Mat &depth_map)
{
    cv::Mat depth_norm;
    cv::normalize(depth_map, depth_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

    // 将归一化后的深度图转换为伪彩色图
    cv::Mat depth_color;
    cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

    // 显示深度图和伪彩色图
    cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Depth Image", depth_norm);
    cv::namedWindow("Depth Color Map", cv::WINDOW_AUTOSIZE);
    cv::imshow("Depth Color Map", depth_color);
    cv::waitKey(1);
}

// calculate theoretical U and V from x,y,z
void LidarCore::getTheoreticalUV(float *theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic, double x, double y,
                                 double z)
{
    // set the intrinsic and extrinsic matrix
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]},
                            {intrinsic[3], intrinsic[4], intrinsic[5]},
                            {intrinsic[6], intrinsic[7], intrinsic[8]}};
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]},
                            {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]},
                            {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};
    double matrix3[4][1] = {x, y, z, 1};

    // transform into the opencv matrix
    cv::Mat matrixIn(3, 3, CV_64F, matrix1);
    cv::Mat matrixOut(3, 4, CV_64F, matrix2);
    cv::Mat coordinate(4, 1, CV_64F, matrix3);

    // calculate the result of u and v
    cv::Mat result = matrixIn * matrixOut * coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    theoryUV[0] = u / depth;
    theoryUV[1] = v / depth;
}

void LidarCore::createDepth(int height, int width, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered)
{
    cv::Mat depth_map(height, width, CV_32F, cv::Scalar(0));
    float x, y, z;
    float depth_temp = 0;
    float theoryUV[2] = {0, 0};
    int myCount = 0;
    for (unsigned int i = 0; i < cloud_filtered->points.size(); i++)
    {
        x = cloud_filtered->points[i].x;
        y = cloud_filtered->points[i].y;
        z = cloud_filtered->points[i].z;

        depth_temp = sqrt(x * x + y * y + z * z);

        getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
        int u = floor(theoryUV[0] + 0.5); // 四舍五入像素值
        int v = floor(theoryUV[1] + 0.5);

        u > width ? u = width - 1 : u;
        u < 0 ? u = 1 : u;
        v > height ? v = height - 1 : v;
        v < 0 ? v = 1 : v;
        // DEBUG:输出计算结果
        // cout << u << " " << v << endl;
        depth_map.at<float>(v, u) = depth_temp;

        ++myCount;
    }
    // DEBUG:可选直接查看原始深度图
    // cv::imshow("window", depth_map);
    // cv::waitKey(1);

    // viewFalseColor(depth_map);//无用
    publish_image(pub_depthImage_, depth_map);
}

void LidarCore::publish_image(const ros::Publisher &in_publisher, const cv::Mat &image)
{
    sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, image).toImageMsg();
    imageMsg->header.stamp = ros::Time::now(); // 当前时间戳
    imageMsg->header.frame_id = "livox_frame"; // 帧ID
    in_publisher.publish(imageMsg);
}

void LidarCore::publish_cloud(const ros::Publisher &in_publisher,
                              const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                              const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

// yaw偏航，pitch俯仰
void LidarCore::changeFOV(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered,
                          const std::pair<float, float> &yaw,
                          const std::pair<float, float> &pitch)
{
    // 进行点云滤波处理
    for (size_t i = 0; i < in_cloud->size(); ++i)
    {
        // 计算俯仰角和航向角
        auto theta_yaw = static_cast<float>(atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI);
        auto theta_pitch = static_cast<float>(
            atan2(in_cloud->points[i].z, sqrt(pow(in_cloud->points[i].x, 2) + pow(in_cloud->points[i].y, 2))) *
            180 / M_PI);

        // 过滤点云数据，保留特定角度范围内的点
        if (theta_yaw > yaw.first && theta_yaw < yaw.second && theta_pitch < pitch.second &&
            theta_pitch > pitch.first && in_cloud->points[i].y < 15)
        {
            cloud_filtered->push_back(in_cloud->points[i]);
        }
    }
}

void LidarCore::callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // ROS_INFO("lidar get!");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>); // 存储二次初步滤波结果

    pcl::fromROSMsg(*in_cloud_ptr, *cloud);
    changeFOV(cloud, cloud_filtered, std::make_pair(60, 120), std::make_pair(-30, 45));
    createDepth(480, 640, cloud_filtered);

    publish_cloud(pub_filtered_, cloud_filtered, in_cloud_ptr->header);
}