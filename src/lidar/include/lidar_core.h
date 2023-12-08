#pragma once // 防止重复引用造成二义性，类似#ifndef

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

class LidarCore
{
private:
    ros::Subscriber sub_point_cloud_;              // 接受话题
    ros::Publisher pub_filtered_, pub_depthImage_; // 发布话题
    std::vector<float> intrinsic, distortion, extrinsic;
    void publish_cloud(const ros::Publisher &in_publisher,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                       const std_msgs::Header &in_header);
    void viewFalseColor(cv::Mat &depth_map);
    void publish_image(const ros::Publisher &in_publisher, const cv::Mat &image);

    void getTheoreticalUV(float *theoryUV, const std::vector<float> &intrinsic, const std::vector<float> &extrinsic, double x, double y,
                          double z);
    void changeFOV(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered,
                   const std::pair<float, float> &yaw = std::make_pair(45.f, 135.f),
                   const std::pair<float, float> &pitch = std::make_pair(-30.f, 45.f));
    void createDepth(int width, int height, const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud);
    void callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

public:
    LidarCore(ros::NodeHandle &nh);
    ~LidarCore();
    void Spin();
};