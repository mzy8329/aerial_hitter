#include "AerialArm_sim.h"
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

#include "common_tools.h"


int main(int argc, char** argv)
{
    // ros::init(argc, argv, "arm_test");
    // ros::NodeHandle nh;
    // ros::Rate loop_rate(100);

    // AerialArm arm(nh);

    // sleep(0.1);
    // arm.GetSet();

    // sleep(3.0);

    // arm.toZero();
    std::vector<Eigen::Vector3d> a;
    Eigen::Vector3d pt{0,1,1};
    for(int i = 0; i < 10; i++)
    {
        a.push_back(pt);
    }

    char name[] = "data/1.txt";
    common_tools::writeFile(name, a);


    return 0;
}