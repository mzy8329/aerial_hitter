#ifndef COMMON_TOOLS_H
#define COMMON_TOOLS_H

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>

#include <iostream>
#include <fstream>


namespace common_tools
{
    Eigen::Vector3d quaternion2Euler(Eigen::Vector4d q);
    Eigen::Vector4d Euler2quaternion(Eigen::Vector3d Euler);

    Eigen::Vector3d orientation2Euler(geometry_msgs::Quaternion q);
    geometry_msgs::Quaternion Euler2orientation(Eigen::Vector3d Euler);

    std::vector<Eigen::Vector3d> triangleProfile(Eigen::Vector3d pt_start, Eigen::Vector3d pt_end, Eigen::Vector3d pt_pass, double dt);
    std::vector<Eigen::Vector3d> triangleProfile(Eigen::Vector3d pt_start, Eigen::Vector3d pt_end, double dt);



    void writeFile(char* name, std::vector<Eigen::Vector3d> data);

} // namespace common_tools

#endif