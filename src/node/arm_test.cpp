#include "AerialArm.h"
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    AerialArm arm(nh);

    sleep(0.1);
    arm.GetSet();

    sleep(3.0);

    arm.toZero();



    return 0;
}