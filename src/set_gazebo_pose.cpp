#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>




int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_gazebo_pose");
    ros::NodeHandle nh;

    return 0;
}