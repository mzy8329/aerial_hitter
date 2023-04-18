#ifndef UAV_SIM_H
#define UAV_SIM_H

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
#include <string>


#include "common_tools.h"
#include "AerialArm_sim.h"
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


class UAV_sim
{
public:
    UAV_sim(ros::NodeHandle nh, double ctrl_period = 50, double UAV_vel = 2.0);
    ~UAV_sim(){;;}

    UAV_model_e _mode;

    void state_Callback(const mavros_msgs::StateConstPtr &msg);
    void currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg);
    void ballPoseVrpnCallBack(const geometry_msgs::PoseStampedConstPtr &body_msg);
    void ballPoseGazeboCallBack(const nav_msgs::OdometryConstPtr &body_msg);

    bool hitingAllow_Callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    void initParam(ros::NodeHandle nh);
    
    void takeOff();
    void Hover();
    void Move();
    void Hit();

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
    ros::Subscriber _ballPoseGazebo_sub;

    ros::Publisher _local_pose_pub;
    ros::Publisher _local_traj_pub;
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
    double _vel_ratio = 1.0;

    struct
    {
        AerialArm_sim Arm;
        double arm_length[2] = {0.106, 0.160};
        double arm_offset[2] = {-2.6374, 0.0};
        double arm_start[2] = {-2.3, -1.8};
        double arm_end[2] = {-0.3, 0};
        double arm_resolution[2] = {1, 1};
        double arm_hit_pos[2];
        std::vector<Eigen::Vector3d> arm_pos_target[2];
        bool isSet;
        double time_ff = 0;
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


    struct
    {
        /* data */
        const char* origin_folder = "/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/sim/";
        char folder_name[50] = {""};
        char subfolder_name[50] = {""};
        char file_name[150] = {""};

        char uav_plan_x_name[20] = "/uav_plan_x.txt";
        char uav_plan_y_name[20] = "/uav_plan_y.txt";
        char uav_plan_z_name[20] = "/uav_plan_z.txt";
        char arm_plan_0_name[20] = "/arm_plan_0.txt";
        char arm_plan_1_name[20] = "/arm_plan_1.txt";

        char uav_traj_name[20] = "/uav_traj.txt";
        char arm_traj_name[20] = "/arm_traj.txt";

        char uav_pose_all_name[20] = "/uav_pose_all.txt";
        char ball_pose_all_name[20] = "/ball_pose_all.txt";
        char arm_pose_all_name[20] = "/arm_pose_all.txt";

        char hit_data_name[20] = "/hit_data.txt";
        char time_data_name[20] = "/time_data.txt";
        

        
        std::vector<Eigen::Vector4d> uav_traj;
        std::vector<Eigen::Vector3d> arm_traj;

        std::vector<Eigen::Vector4d> uav_pose_all;
        std::vector<Eigen::Vector3d> arm_pose_all;
        std::vector<Eigen::Vector4d> ball_pose_all;


    }_DebugInfo;
    
};

#endif