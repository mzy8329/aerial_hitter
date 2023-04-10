#ifndef UAV_INKIND_H
#define UAV_INKIND_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Eigen>
#include <algorithm>


#include "common_tools.h"
#include "AerialArm_inkind.h"
#include "TrajPredict.h"
#include "rviz_draw.h"


#define FLY 0

typedef enum{
    wait,
    take_off,
    hover,
    move,
    hit,
    land,
    manual,
}UAV_model_e;


class UAV_inkind
{
public:
    UAV_inkind(ros::NodeHandle nh, double ctrl_period = 50, double UAV_vel = 2.0);
    ~UAV_inkind(){;;}

    UAV_model_e _mode;

    void state_Callback(const mavros_msgs::StateConstPtr &msg);
    void currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg);
    void targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
    void ballPoseVrpnCallBack(const geometry_msgs::PoseStampedConstPtr &body_msg);

    bool hitingAllow_Callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    void initParam(ros::NodeHandle nh);
    
    void takeOff();
    void Hover();
    void Move();
    void Hit();
    void Land();

    void printData();
    void hit2base();

    template<typename T>
    T pt2SafeBox(T pt);
    template<typename T>
    bool checkSafeBox(T pt);
    

    void mainLoop();

    geometry_msgs::PoseStamped _pose_hover;


private:
    ros::NodeHandle _nh;

    ros::Subscriber _state_sub;
    ros::Subscriber _target_pose_sub;
    ros::Subscriber _current_pose_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _ballPoseVrpn_sub;

#if FLY
    ros::Publisher _local_pose_pub;
    ros::Publisher _local_traj_pub;
#endif
    ros::Publisher _rviz_marker_pub;
    ros::Publisher _hitPoint_pub;

    ros::ServiceServer _hitingAllow_service;

    ros::ServiceClient _set_mode_client;
    ros::ServiceClient _arming_client;
    ros::ServiceClient _landing_client;

    mavros_msgs::State _current_state;
    trajectory_msgs::MultiDOFJointTrajectory _targetPose;
    std::vector<Eigen::Vector3d> _targetTraj_xyz[3];
    geometry_msgs::PoseStamped _targetPoint;
    geometry_msgs::PoseStamped _currentPose;

    double _ctrl_rate;
    double _UAV_Vel;
    bool _hitingAllow;

    Eigen::VectorXd _base_pose;
    Eigen::VectorXd _hit_pose;
    Eigen::Vector4d _hit_pose_temp;
    Eigen::Vector3d _point_target;

    Eigen::Vector3d _axis2link = {-0.00732, 0, -0.04};
    Eigen::Vector3d _arm2base = {0, 0, -0.012};

    double _arm_ctrl_ratio = 1.5;
    double _move_ctrl_ratio = 2.0;
    double _hit_ctrl_ratio = 2.0;

    struct
    {
        AerialArm_inkind Arm;
        double arm_length[2] = {0.106, 0.190};
        double arm_offset[2] = {-2.6374, 0};
        double arm_start[2] = {-2.3, -1.8};
        double arm_end[2] = {-0.3, 0};
        double arm_resolution[2] = {2, 1.5};
        double arm_hit_pos[2];
        std::vector<Eigen::Vector3d> arm_pos_target[2];
        bool isSet;
    }_Arm;

    struct
    {
        int fit_len;
        int check_len;
        int fitKd_len;
        float freeFallCheck_err;
        float pre_time;
        int pre_size;
        double beta;

        TrajPredict trajPredict;
    }_Predict;

    struct
    {
        visualization_msgs::Marker mark;
        
        float colar_pose[4] = {1, 0, 0.8, 0.5};
        float colar_traj[4] = {1, 0.5, 0.2, 0.2};
        float colar_hit[4] = {1, 0.2, 0.2, 0.6};
        float colar_hitPoint[4] = {0.8, 1.0, 0.2, 0.6};
    }_Rviz;

    struct
    {
        double x_lim[2];
        double y_lim[2];
        double z_lim[2];
    }_SafeBox;
};

#endif