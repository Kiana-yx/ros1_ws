#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

void callback(const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::ImageConstPtr &mask_msg)
{
    // ROS_INFO("both image get!");
    // 将ROS消息转换为OpenCV格式的图像
    cv_bridge::CvImagePtr cv_mask_ptr, cv_depth_ptr;
    try
    {
        cv_mask_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::MONO8);
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // // 获取图像分辨率
        // cv::Size size_mask = cv_mask_ptr->image.size();
        // cv::Size size_depth = cv_depth_ptr->image.size();
        // // 输出图像分辨率
        // std::cout << "RGBImage resolution: " << size_mask.width << " x " << size_mask.height << std::endl;
        // std::cout << "DepthImage resolution: " << size_depth.width << " x " << size_depth.height << std::endl;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mask;
    threshold(cv_mask_ptr->image, mask, 1, 255, cv::THRESH_BINARY); // 将灰度图像二值化

    // 应用掩膜到深度图像上
    cv::Mat masked_depth;
    cv_depth_ptr->image.copyTo(masked_depth, mask);

    // 显示叠加后的图像
    cv::imshow("Combined Image", masked_depth);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "united_node");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> sub_depth_(nh, "/depthImage", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_(nh, "/grayImage", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), sub_depth_, sub_mask_);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}