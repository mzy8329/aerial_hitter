#include "AerialArm_sim.h"
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

#include "common_tools.h"


int main(int argc, char** argv)
{
    std::vector<Eigen::Vector3d> a;
    Eigen::Vector3d pt{0,1,1};
    for(int i = 0; i < 10; i++)
    {
        a.push_back(pt);
    }

    char name[] = "/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/sim/AAA";

    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char file_name[100];
    std::strcpy(file_name, (std::string(name)+"-"+std::ctime(&now_c)+".txt").c_str());
    std::cout<<file_name<<std::endl;

    common_tools::writeFile(name, a, file_add);


    return 0;
}