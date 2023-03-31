#ifndef UAV_H
#define UAV_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
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
#include "AerialArm.h"
#include "TrajPredict.h"
#include "rviz_draw.h"


typedef enum{
    wait,
    take_off,
    hover,
    move,
    hit,
    land,
    manual,
}UAV_model_e;


class UAV
{
public:
    UAV(ros::NodeHandle nh, double ctrl_period = 50, double UAV_vel = 2.0);
    ~UAV(){;;}

    UAV_model_e _mode;

    void state_Callback(const mavros_msgs::StateConstPtr &msg);
    void currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg);
    void targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
    void ballPoseVrpnCallBack(const geometry_msgs::PoseStampedConstPtr &body_msg);
    void ballPoseGazeboCallBack(const nav_msgs::OdometryConstPtr &body_msg);


    void setArmParam(double* arm_length, double* arm_offset, Eigen::Vector3d axis2link, Eigen::Vector3d arm2base, double* arm_start, double* arm_end, double* arm_time_pass);
    void initParam(ros::NodeHandle nh);
    
    void takeOff();
    void Hover();
    void Move();
    void Hit();

    void printData();
    void hit2base();
    template<typename T>
    T checkSafeBox(T pt);

    void mainLoop();

    geometry_msgs::PoseStamped _pose_hover;


private:
    ros::NodeHandle _nh;

    ros::Subscriber _state_sub;
    ros::Subscriber _target_pose_sub;
    ros::Subscriber _current_pose_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _ballPoseVrpn_sub;
    ros::Subscriber _ballPoseGazebo_sub;

    ros::Publisher _local_pose_pub;
    ros::Publisher _local_traj_pub;
    ros::Publisher _rviz_marker_pub;
    ros::Publisher _hitPoint_pub;

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

    AerialArm _Arm;
    double _arm_length[2];
    double _arm_offset[2];
    double _arm_start[2];
    double _arm_end[2];
    double _arm_time_pass[2];
    double _arm_hit_pos[2];
    std::vector<Eigen::Vector3d> _arm_pos_target[2];
    bool _isSet;

    Eigen::Vector3d _axis2link;
    Eigen::Vector3d _arm2base;

    Eigen::VectorXd _base_pose;
    Eigen::VectorXd _hit_pose;
    Eigen::Vector4d _hit_pose_temp;

    

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

    Eigen::Vector3d _point_target = {1.0, 0.2, 0};

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
        double x_lim[2] = {-2, 2};
        double y_lim[2] = {-2, 2};
        double z_lim[2] = {0.5, 2};
    }_SafeBox;

};





#endif