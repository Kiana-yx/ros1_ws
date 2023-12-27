#include "pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh)
{
    sub_point_cloud_ = nh.subscribe("/livox/lidar", 10, &PclTestCore::point_cb, this);

    pub_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);

    // 程序到达ros::spin()之前按照一系列规则，设定一系列话题订阅者
    // ros::spin()使订阅者们可以开始接受话题，进入回调函数
    ros::spin();
}

PclTestCore::~PclTestCore() {}

void PclTestCore::Spin()
{
}

void PclTestCore::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out,
                             bool PassThrough)
{
    if (PassThrough == false)
    {
        pcl::ExtractIndices<pcl::PointXYZI> cliper;

        cliper.setInputCloud(in);
        pcl::PointIndices indices;
#pragma omp for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (in->points[i].z > clip_height)
            {
                indices.indices.push_back(i);
            }
        }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true); // ture to remove the indices
        cliper.filter(*out);
    }
    else
    {
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(in);
        pass.setFilterFieldName("z");          // 设置滤波的field
        pass.setFilterLimits(-2, clip_height); // 滤波范围
        pass.filter(*out);                     // 滤波结果存放

#ifdef IN_door
        pass.setInputCloud(out);      // 设置输入点云
        pass.setFilterFieldName("x"); // 设置滤波的field
        pass.setFilterLimits(0, 15);  // 滤波范围
        pass.filter(*out);            // 滤波结果存放

        pass.setInputCloud(out);       // 设置输入点云
        pass.setFilterFieldName("y");  // 设置滤波的field
        pass.setFilterLimits(-15, 15); // 滤波范围
        pass.filter(*out);             // 滤波结果存放
#endif
    }
}
void PclTestCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
}

void PclTestCore::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); // ture to remove the indices
    cliper.filter(*out);
}

// 离群点滤波 注意包含#include <pcl/filters/statistical_outlier_removal.h>
void PclTestCore::remove_outlier(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                 const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor; // 离群滤波器对象
    sor.setInputCloud(in);                              // 设置待滤波的点云
    sor.setMeanK(100);                                  // 设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(1.0);                        // 设置判断是否为离群点的阀值
    sor.filter(*out);
}

//
void PclTestCore::remove_ground_RANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_no_ground,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground, bool simplify)
{
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne; // 法线估计对象
    pcl::ExtractIndices<pcl::PointXYZI> extract;           // 点提取对象
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());   // 存储位于平面上点
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>); // 存储剔除平面后的点
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    if (simplify = false)
    {
        pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg; // 分割对象

        // Estimate point normals
        ne.setSearchMethod(tree); // 搜索方式
        ne.setInputCloud(in);
        ne.setKSearch(50); // 选择最近邻的50个点进行法线估计
        ne.compute(*cloud_normals);

        // Create the segmentation object for the planar model and set all the parameters
        // 可选设置,设置模型系数需要优化
        seg.setOptimizeCoefficients(true);
        // 必须设置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);      // 设置迭代的最大次数，默认是10000
        seg.setDistanceThreshold(0.05); // 设置内点到模型的距离允许最大值
        seg.setInputCloud(in);
        seg.setInputNormals(cloud_normals);               // 输入的法线
        seg.setEpsAngle(PI / 120);                        // 设置角度误差
        seg.segment(*inliers_plane, *coefficients_plane); // 得到平面内点和平面的4个系数
    }

    else
    {
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        // 必须设置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);      // 设置迭代的最大次数，默认是10000
        seg.setDistanceThreshold(0.05); // 设置内点到模型的距离允许最大值
        seg.setInputCloud(in);
        seg.segment(*inliers_plane, *coefficients_plane); // 得到平面内点和平面的4个系数
    }
    // 从点云中抽取分割的处在平面上的点集
    extract.setInputCloud(in);
    extract.setIndices(inliers_plane);
    extract.setNegative(true); // 存储平面外的点
    extract.filter(*out_no_ground);
    extract.setNegative(false); // 存储平面上的点
    extract.filter(*out_ground);
}

void PclTestCore::remove_ground_designated(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_no_ground,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in, *temp);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(temp);
    pass.setFilterFieldName("z");                                     // 设置滤波的field
    pass.setFilterLimits(-SENSOR_HEIGHT - 0.5, -SENSOR_HEIGHT + 0.5); // 滤波范围
    pass.filter(*temp);                                               // 滤波结果存放

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    // 必须设置，设置分割的模型类型、所用的随机参数估计方法、距离阈值、输入点云
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);      // 设置迭代的最大次数，默认是10000
    seg.setDistanceThreshold(0.05); // 设置内点到模型的距离允许最大值
    seg.setInputCloud(temp);
    seg.segment(*inliers_plane, *coefficients_plane); // 得到平面内点和平面的4个系数

    // 从点云中抽取分割的处在平面上的点集
    pcl::ExtractIndices<pcl::PointXYZI> extract; // 点提取对象
    extract.setInputCloud(temp);
    extract.setIndices(inliers_plane);
    extract.setNegative(false); // 存储平面上的点
    extract.filter(*out_ground);

    // TODO:怎样求两点集差？对点云索引理解有误
    //  pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    //  extract_ground.setInputCloud(in);
    //  extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(out_ground));
    //  extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    //  extract_ground.filter(*out_no_ground);
}

void PclTestCore::remove_ground_Ray(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_no_ground,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground)
{
    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

    XYZI_to_RTZColor(in, organized_points,
                     radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(in);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); // true removes the indices, false leaves only the indices
    extract_ground.filter(*out_ground);

    extract_ground.setNegative(true); // true removes the indices, false leaves only the indices
    extract_ground.filter(*out_no_ground);
}
/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void PclTestCore::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   PointCloudXYZIRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    // 对每个点计算水平距离和theta
    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
            in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
        {
            theta += 360;
        }
        // 角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE); // auto申明一块临时的变量内存
        // 半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i]; // 保留原始XYZI属性
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i; // 原始编号

        out_organized_points[i] = new_point;

        // radial divisions更加角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);

    } // end for

    // 将同一根射线上的点按照半径（距离）排序
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b)
                  { return a.radius < b.radius; });
    }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void PclTestCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) // sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) // loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

            // for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            // check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                // Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                // check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void PclTestCore::publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr base_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr, true);
    remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close_ptr);
    remove_outlier(remove_close_ptr, base_filtered_ptr); // NOTICE:速度太慢，暂时放弃

    // remove_ground_RANSAC(base_filtered_ptr, no_ground_cloud_ptr, ground_cloud_ptr, true); //是否采用简化的平面滤波方式，简化后大致频率6.92Hz--6.88Hz
    // remove_ground_designated(base_filtered_ptr, no_ground_cloud_ptr, ground_cloud_ptr);//failure
    remove_ground_Ray(base_filtered_ptr, no_ground_cloud_ptr, ground_cloud_ptr);

    publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_filtered_, remove_close_ptr, in_cloud_ptr->header);

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ROS_INFO("处理一次数据用时: %f 秒", time_used.count());
}