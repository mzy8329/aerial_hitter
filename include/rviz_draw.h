#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


#include <eigen3/Eigen/Eigen>
#include <vector>



namespace rviz_draw
{
    visualization_msgs::Marker draw(
        std::vector<Eigen::Vector3d> points,
        float* colar,
        const char* frameId="temp", 
        int id=1,     
        uint type=visualization_msgs::Marker::SPHERE_LIST, 
        uint action=visualization_msgs::Marker::ADD,
        int duration=0);

    visualization_msgs::Marker draw(
        std::vector<Eigen::Vector4d> points,
        float* colar,
        const char* frameId="temp", 
        int id=1,     
        uint type=visualization_msgs::Marker::SPHERE_LIST, 
        uint action=visualization_msgs::Marker::ADD,
        int duration=0);

    visualization_msgs::Marker draw(
        Eigen::Vector3f points,
        float* colar,
        const char* frameId="temp", 
        int id=1,     
        uint type=visualization_msgs::Marker::SPHERE, 
        uint action=visualization_msgs::Marker::ADD,
        int duration=0);

    visualization_msgs::Marker draw(
        Eigen::Vector4d points,
        float* colar,
        const char* frameId="temp", 
        int id=1,     
        uint type=visualization_msgs::Marker::SPHERE, 
        uint action=visualization_msgs::Marker::ADD,
        int duration=0);

    visualization_msgs::Marker draw(
        Eigen::VectorXd points,
        float* colar,
        const char* frameId="temp", 
        int id=1,     
        uint type=visualization_msgs::Marker::ARROW, 
        uint action=visualization_msgs::Marker::ADD,
        int duration=0);

};