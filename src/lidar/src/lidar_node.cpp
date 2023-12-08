#include "lidar_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox"); // 初始化节点，第三个参数 node_name

    ros::NodeHandle nh;

    LidarCore core(nh);

    return 0;
}