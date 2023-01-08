#include "UAV.h"


UAV::UAV(ros::NodeHandle nh, double ctrl_freq)
{
    _nh = nh;
    _ctrl_rate = ctrl_freq;

    _target_pose_sub = nh.subscribe("/UAV/target_pose", 1, &UAV::targetPose_Callback, this);
    _current_pose_sub = nh.subscribe("/UAV/pose", 1, &UAV::currentPose_Callback, this);
    _hitPoint_sub = nh.subscribe("/UAV/hitPoint", 1, &UAV::hitPoint_Callback, this);


    _local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    _local_traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/mavros/setpoint_trajectory/local", 1);

    _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _landing_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/land");

    _Arm.init(nh);
}

void UAV::currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    _currentPose = *msg;
}

void UAV::targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
{
    _targetPose = *msg;
}

void UAV::hitPoint_Callback(const nav_msgs::OdometryConstPtr &msg)
{
    ;;
}


void UAV::setArmParam(double arm_0_length, double arm_1_length, double arm_0_offset, double arm_1_offset, Eigen::Vector3d axis2link)
{
    _arm_length[0] = arm_0_length, _arm_length[1] = arm_1_length;
    _arm_offset[0] = arm_0_offset, _arm_offset[1] = arm_1_length;

    _axis2link = axis2link;
}