#ifndef UAV_H
#define UAV_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
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
    void currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg);
    void targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
    void hitPoint_Callback(const nav_msgs::OdometryConstPtr &msg);


    void setArmParam(double arm_0_length, double arm_1_length, double arm_0_offset, double arm_1_offset, Eigen::Vector3d axis2link);
    


private:
    ros::NodeHandle _nh;

    ros::Subscriber _target_pose_sub;
    ros::Subscriber _current_pose_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _hitPoint_sub;

    ros::Publisher _local_pose_pub;
    ros::Publisher _local_traj_pub;

    ros::ServiceClient _set_mode_client;
    ros::ServiceClient _arming_client;
    ros::ServiceClient _landing_client;

    trajectory_msgs::MultiDOFJointTrajectory _targetPose;
    geometry_msgs::PoseStamped _currentPose;
    trajectory_msgs::MultiDOFJointTrajectory _plannedTraj;


    double _ctrl_rate;

    AerialArm _Arm;
    double _arm_length[2];
    double _arm_offset[2];

    Eigen::Vector3d _axis2link;

};





#endif