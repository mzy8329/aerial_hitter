#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include "common_tools.h"
#include "UAV_sim.h"


#define CTRL_FREQ 100.0

double arm_length[2] = {0.106, 0.160};
double arm_offset[2] = {-2.6374, 0};

double arm_start[2] = {-2.3, -1.8};  
double arm_end[2] = {-0.3, 0};

Eigen::Vector3d axis2link = {-0.00732, 0, -0.04};
Eigen::Vector3d arm2base = {0, 0, -0.012};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aerial_hitter_main_ctrl");
    ros::NodeHandle nh;
    ros::Rate loop_rate(CTRL_FREQ);

    UAV_sim aerialHitter(nh, CTRL_FREQ);
    sleep(1.0);
    aerialHitter.mainLoop();

    return 0;
}