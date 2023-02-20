#ifndef COMMON_TOOLS_H
#define COMMON_TOOLS_H

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>

#define CTRL_FREQ 100.0


namespace common_tools
{
    Eigen::Vector3d quaternion2Euler(Eigen::Vector4d q);
    Eigen::Vector4d Euler2quaternion(Eigen::Vector3d Euler);

    Eigen::Vector3d orientation2Euler(geometry_msgs::Quaternion q);
    geometry_msgs::Quaternion Euler2orientation(Eigen::Vector3d Euler);

    std::vector<Eigen::Vector2d> triangleProfile(Eigen::Vector2d pt_start, Eigen::Vector2d pt_end, Eigen::Vector2d pt_pass, double vel_pass);
   
} // namespace common_tools

#endif