//
// Created by Kiana on 22-2-10.
//

#include "pcl_test_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");  //初始化节点，第三个参数 node_name

    ros::NodeHandle nh;

    PclTestCore core(nh);
    // core.Spin();
    return 0;
}