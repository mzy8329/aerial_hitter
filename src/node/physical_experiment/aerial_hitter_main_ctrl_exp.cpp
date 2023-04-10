#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include "common_tools.h"
#include "UAV_inkind.h"


#define CTRL_FREQ 100.0


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aerial_hitter_main_ctrl");
    ros::NodeHandle nh;
    ros::Rate loop_rate(CTRL_FREQ);

    UAV_inkind aerialHitter(nh, CTRL_FREQ);
    sleep(1.0);
    aerialHitter.mainLoop();

    return 0;
}