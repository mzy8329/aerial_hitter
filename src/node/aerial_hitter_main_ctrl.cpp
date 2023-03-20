#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include "common_tools.h"
#include "UAV.h"


#define CTRL_FREQ 100.0

double arm_length[2] = {0.106, 0.160};
double arm_offset[2] = {-2.6374, 0};

double arm_start[2] = {-2.3, -1.8};  
double arm_end[2] = {-0.3, 0};
double arm_time_pass[2] = {0.05, 0.05};

Eigen::Vector3d axis2link = {-0.00732, 0, -0.4};
Eigen::Vector3d arm2base = {0, 0, -0.012};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aerial_hitter_main_ctrl");
    ros::NodeHandle nh;
    ros::Rate loop_rate(CTRL_FREQ);

    UAV aerialHitter(nh, CTRL_FREQ);
    aerialHitter.setArmParam(arm_length, arm_offset, axis2link, arm2base, arm_start, arm_end, arm_time_pass);

    sleep(1.0);
    aerialHitter.mainLoop();

    return 0;
}