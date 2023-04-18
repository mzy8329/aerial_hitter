#ifndef COMMON_TOOLS_H
#define COMMON_TOOLS_H

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <algorithm>
#include <string>


typedef enum
{
    file_new,
    file_add
}fWrite_mode_e;

namespace common_tools
{
    Eigen::Vector3d quaternion2Euler(Eigen::Vector4d q);
    Eigen::Vector4d Euler2quaternion(Eigen::Vector3d Euler);

    Eigen::Vector3d orientation2Euler(geometry_msgs::Quaternion q);
    geometry_msgs::Quaternion Euler2orientation(Eigen::Vector3d Euler);

    std::vector<Eigen::Vector3d> triangleProfile(Eigen::Vector3d pt_start, Eigen::Vector3d pt_end, Eigen::Vector3d pt_pass, double dt);
    std::vector<Eigen::Vector3d> triangleProfile(Eigen::Vector3d pt_start, Eigen::Vector3d pt_end, double dt);

    const char* getTimenow();

    void writeFile(char* name, std::vector<Eigen::Vector2d> data, fWrite_mode_e mode);
    void writeFile(char* name, std::vector<Eigen::Vector3d> data, fWrite_mode_e mode);
    void writeFile(char* name, std::vector<Eigen::Vector4d> data, fWrite_mode_e mode);
    void writeFile(char* name, Eigen::VectorXd data, fWrite_mode_e mode);
    void writeFile(char* name, std::string data, fWrite_mode_e mode);




} // namespace common_tools

#endif