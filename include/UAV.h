#ifndef UAV_H
#define UAV_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Eigen>


#include "common_tools.h"
#include "AerialArm.h"


typedef enum{
    wait,
    take_off,
    hover,
    hit,
    land,
    manual,
}UAV_model_e;


class UAV
{
public:
    UAV(ros::NodeHandle nh, double ctrl_period = 50);
    ~UAV(){;;}

    UAV_model_e _model;

    void state_Callback(const mavros_msgs::StateConstPtr &msg);
    void currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg);
    void targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
    void hitPoint_Callback(const nav_msgs::OdometryConstPtr &msg);


    void setArmParam(double* arm_length, double* arm_offset, Eigen::Vector3d axis2link, Eigen::Vector3d arm2base, double* arm_start, double* arm_end, double* arm_time_pass);
    void takeOff();
    void Hover();
    void Hit();

    void printData();
    void hit2base();

    void mainLoop();

    geometry_msgs::PoseStamped _pose_hover;


private:
    ros::NodeHandle _nh;

    ros::Subscriber _state_sub;
    ros::Subscriber _target_pose_sub;
    ros::Subscriber _current_pose_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _hitPoint_sub;


    ros::Publisher _local_pose_pub;
    ros::Publisher _local_traj_pub;

    ros::ServiceClient _set_mode_client;
    ros::ServiceClient _arming_client;
    ros::ServiceClient _landing_client;

    mavros_msgs::State _current_state;
    trajectory_msgs::MultiDOFJointTrajectory _targetPose;
    geometry_msgs::PoseStamped _targetPoint;
    geometry_msgs::PoseStamped _currentPose;
    trajectory_msgs::MultiDOFJointTrajectory _plannedTraj;


    double _ctrl_rate;

    AerialArm _Arm;
    double _arm_length[2];
    double _arm_offset[2];
    double _arm_start[2];
    double _arm_end[2];
    double _arm_time_pass[2];

    double _arm_hit_pos[2];
    std::vector<Eigen::Vector2d> _arm_pos_target[2];
    int _arm_pos_target_index;

    Eigen::Vector3d _axis2link;
    Eigen::Vector3d _arm2base;

    Eigen::VectorXd _base_pose;
    Eigen::VectorXd _hit_pose;

};





#endif