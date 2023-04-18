#include "AerialArm_sim.h"
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <iostream>

#include "common_tools.h"


int main(int argc, char** argv)
{
    std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string time_now(std::ctime(&now_c));
    std::replace(time_now.begin(), time_now.end(), ':', '-');
    std::replace(time_now.begin(), time_now.end(), ' ', '-');
    time_now.erase(remove(time_now.begin(), time_now.end(), '\n'), time_now.end());


    std::cout<<time_now.c_str()<<std::endl;
    std::cout<<common_tools::getTimenow()<<std::endl;



    return 0;
}