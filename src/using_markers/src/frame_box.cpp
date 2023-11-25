
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

class Frame
{
private:
    /* data */
public:
    float min_x;
    float min_y;
    float min_z;
    float max_x;
    float max_y;
    float max_z;
    geometry_msgs::Point p[8];
    visualization_msgs::Marker marker;
    void get_param(void);
    void get_point(void);
    void marker_init(void);
    void paint(void);
};

void Frame::get_param(void)
{
    ros::param::get("/dynamic_reconfigure_node/min_x", min_x);
    ros::param::get("/dynamic_reconfigure_node/min_y", min_y);
    ros::param::get("/dynamic_reconfigure_node/min_z", min_z);
    ros::param::get("/dynamic_reconfigure_node/max_x", max_x);
    ros::param::get("/dynamic_reconfigure_node/max_y", max_y);
    ros::param::get("/dynamic_reconfigure_node/max_z", max_z);
}

void Frame::get_point(void)
{
    p[0].x = min_x;
    p[0].y = min_y;
    p[0].z = min_z;
    p[1].x = max_x;
    p[1].y = min_y;
    p[1].z = min_z;
    p[2].x = max_x;
    p[2].y = max_y;
    p[2].z = min_z;
    p[3].x = min_x;
    p[3].y = max_y;
    p[3].z = min_z;

    p[4].x = min_x;
    p[4].y = min_y;
    p[4].z = max_z;
    p[5].x = max_x;
    p[5].y = min_y;
    p[5].z = max_z;
    p[6].x = max_x;
    p[6].y = max_y;
    p[6].z = max_z;
    p[7].x = min_x;
    p[7].y = max_y;
    p[7].z = max_z;
}

void Frame::marker_init(void)
{
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points_and_lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.scale.x = 0.05;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Line list is red
    marker.color.g = 1.0;
    marker.color.a = 1.0;
}

void Frame::paint(void)
{
    for (uint32_t i = 0; i < 4; ++i)
    {
        marker.points.push_back(p[i % 4]);
        marker.points.push_back(p[(i + 1) % 4]);
    }
    marker.points.push_back(p[0]);
    marker.points.push_back(p[4]);
    for (uint32_t i = 4; i < 7; ++i)
    {
        marker.points.push_back(p[i]);
        marker.points.push_back(p[i + 1]);
    }
    marker.points.push_back(p[7]);
    marker.points.push_back(p[4]);
    for (uint32_t i = 0; i < 4; ++i)
    {
        marker.points.push_back(p[i]);
        marker.points.push_back(p[i + 4]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher safety_pub_ = n.advertise<visualization_msgs::Marker>("safety_frame", 10);

    ros::Rate r(10); //设置循环频率为10hz

    

    while (ros::ok())
    {
        Frame safety_frame;
        safety_frame.get_param();
        safety_frame.get_point();
        safety_frame.marker_init();
        safety_frame.paint();
        
        safety_pub_.publish(safety_frame.marker);

        r.sleep();
    }
}