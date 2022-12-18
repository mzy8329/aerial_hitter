#include "UAV.h"


UAV::UAV(ros::NodeHandle nh, double ctrl_period)
{
    _nh = nh;
    _ctrl_period = ctrl_period;

    _target_pose_sub = nh.subscribe("/UAV/target_pose", 1, &UAV::targetPose_Callback, this);
    _current_pose_sub = nh.subscribe("/UAV/pose", 1, &UAV::currentPose_Callback, this);

    _local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    _local_traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/mavros/setpoint_trajectory/local", 1);

    _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _landing_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/land");
}

void UAV::currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    _currentPose = *msg;
}

void UAV::targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
{
    _targetPose = *msg;
}