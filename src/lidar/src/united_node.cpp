#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "common.h"

#include <Eigen/Dense>

// 定义矩阵类型
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;

class UnitedSensor
{
public:
    UnitedSensor(ros::NodeHandle &nh);
    ~UnitedSensor();
    void Spin();

private:
    ros::NodeHandle nh;
    ros::Publisher pub_pcl_;
    void computeInverseTransform(void);
    void depthToPoint(const cv::Mat &depth_image);
    void callback(const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::ImageConstPtr &mask_msg);

    // 构建逆矩阵
    Matrix4d inverseTransform;
};

UnitedSensor::UnitedSensor(ros::NodeHandle &nh)
{
    computeInverseTransform();

    message_filters::Subscriber<sensor_msgs::Image> sub_depth_(nh, "/depthImage", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_(nh, "/grayImage", 1);
    pub_pcl_ = nh.advertise<sensor_msgs::PointCloud2>("depthToPoint", 1); // 添加的  发布点云

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), sub_depth_, sub_mask_);
    sync.registerCallback(boost::bind(&UnitedSensor::callback, this, _1, _2));

    ros::spin();
}

UnitedSensor::~UnitedSensor() {}

void UnitedSensor::Spin()
{
}

// 计算齐次变换矩阵的逆矩阵
void UnitedSensor::computeInverseTransform()
{
    inverseTransform.setIdentity(); // 初始化为单位矩阵

    string extrinsic_path = "/home/kiana/Downloads/data/parameters/extrinsic.txt";

    std::vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

    Matrix3d R;
    R << extrinsic[0], extrinsic[1], extrinsic[2],
        extrinsic[4], extrinsic[5], extrinsic[6],
        extrinsic[8], extrinsic[9], extrinsic[10];
    Vector3d T;
    T << extrinsic[3], extrinsic[7], extrinsic[11];

    // 设置旋转矩阵部分
    inverseTransform.block<3, 3>(0, 0) = R.transpose();

    // 设置平移向量部分
    inverseTransform.block<3, 1>(0, 3) = -R.transpose() * T;
}

void UnitedSensor::depthToPoint(const cv::Mat &depth_image)
{
    pcl::PointCloud<pcl::PointXYZ> tmp;

    for (size_t v = 0; v < depth_image.rows; v++)
    {
        for (size_t u = 0; u < depth_image.cols; u++)
        {
            float d = depth_image.ptr<float>(v)[u];
            if (d < 0.01 || d > 10)
                continue;

            // TODO:使用读取参数的方法
            double temp_x, temp_y, temp_z;
            temp_z = d;
            temp_x = (u - 324.1569838209711) * d / 533.519;
            temp_y = (v - 222.3341306960716) * d / 533.125;

            Eigen::Vector4d temp_vec;
            temp_vec << temp_x, temp_y, temp_z, 1.0; // 添加 1.0 作为第四个元素

            Eigen::Matrix<double, 4, 1> ans = inverseTransform * temp_vec;

            pcl::PointXYZ p;
            p.x = ans[0], p.y = ans[1], p.z = ans[2];
            // std::cout << p.z << " " << p.x << " " << p.y << std::endl;
            tmp.points.push_back(p);
        }
    }

    sensor_msgs::PointCloud2 output_msh;
    pcl::toROSMsg(tmp, output_msh);
    output_msh.header.frame_id = "livox_frame";
    output_msh.header.stamp = ros::Time::now();
    pub_pcl_.publish(output_msh);
}

void UnitedSensor::callback(const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::ImageConstPtr &mask_msg)
{
    // ROS_INFO("both image get!");
    // 将ROS消息转换为OpenCV格式的图像
    cv_bridge::CvImagePtr cv_mask_ptr, cv_depth_ptr;
    try
    {
        cv_mask_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::MONO8);
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv::Mat mask;
    // threshold(cv_mask_ptr->image, mask, 1, 255, cv::THRESH_BINARY); // 将灰度图像二值化

    // 应用掩膜到深度图像上
    cv::Mat masked_depth;
    cv_depth_ptr->image.copyTo(masked_depth, cv_mask_ptr->image);

    // DEBUG：显示叠加后的深度图
    cv::imshow("Combined Image", masked_depth);
    cv::waitKey(1);

    depthToPoint(masked_depth);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "united_node"); // 初始化节点，第三个参数 node_name

    ros::NodeHandle nh;

    UnitedSensor core(nh);

    return 0;
}