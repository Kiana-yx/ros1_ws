#pragma once //防止重复引用造成二义性，类似#ifndef

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>      // 法向量估计
#include <pcl/filters/extract_indices.h> // 索引提取
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h> //模型定义头文件
#include <pcl/sample_consensus/model_types.h>  //随机参数估计方法头文件
#include <pcl/segmentation/sac_segmentation.h> // 基于采样一致性分割的类的头文件
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>

#define IN_door

#ifdef OUT_door
#define CLIP_HEIGHT 8 //截取掉高于雷达自身8米的点
#define MIN_DISTANCE 0.5
#define RADIAL_DIVIDER_ANGLE 0.18
#define SENSOR_HEIGHT 1.78
#endif

#ifdef IN_door
#define CLIP_HEIGHT 1.6 //截取掉高于雷达自身1.6米的点
#define MIN_DISTANCE 0.5
#define RADIAL_DIVIDER_ANGLE 0.199
#define SENSOR_HEIGHT 0.75
// 0.86
#endif

#define concentric_divider_distance_ 0.01 //0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

#define PI 3.1415926

class PclTestCore
{

private:
    ros::Subscriber sub_point_cloud_; //接受话题

    ros::Publisher pub_filtered_, pub_ground_, pub_no_ground_; //发布话题

    struct PointXYZIRTColor
    {
        pcl::PointXYZI point;

        float radius; //cylindric coords on XY Plane
        float theta;  //angle deg on XY plane

        size_t radial_div;     //index of the radial divsion to which this point belongs to
        size_t concentric_div; //index of the concentric division to which this points belongs to

        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor; //std::vector 是封装动态数组的顺序容器

    size_t radial_dividers_num_;
    size_t concentric_dividers_num_;

    void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

    void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out, bool PassThrough);

    void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void remove_outlier(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
    void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

    void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                          PointCloudXYZIRTColor &out_organized_points,
                          std::vector<pcl::PointIndices> &out_radial_divided_indices,
                          std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

    void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                     pcl::PointIndices &out_ground_indices,
                     pcl::PointIndices &out_no_ground_indices);

    void remove_ground_RANSAC(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr out_no_ground,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground, bool simplify);

    void remove_ground_designated(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr out_no_ground,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground);

    void remove_ground_Ray(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_no_ground,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr out_ground);

    void publish_cloud(const ros::Publisher &in_publisher,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                       const std_msgs::Header &in_header);

public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore();
    void Spin();
};