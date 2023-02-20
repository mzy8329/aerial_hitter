#include "UAV.h"


UAV::UAV(ros::NodeHandle nh, double ctrl_freq)
{
    _nh = nh;
    _ctrl_rate = ctrl_freq;
    _model = wait;

    _state_sub = nh.subscribe("/mavros/state", 10, &UAV::state_Callback, this);
    _target_pose_sub = nh.subscribe("/UAV/target_pose", 1, &UAV::targetPose_Callback, this);
    _current_pose_sub = nh.subscribe("/UAV/pose", 1, &UAV::currentPose_Callback, this);
    _hitPoint_sub = nh.subscribe("/UAV/hitPoint", 1, &UAV::hitPoint_Callback, this);


    _local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    _local_traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/mavros/setpoint_trajectory/local", 1);

    _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _landing_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/land");

    _pose_hover.pose.position.x = 0;
    _pose_hover.pose.position.y = 0;
    _pose_hover.pose.position.z = 2;

    _base_pose.resize(7,1);
    _hit_pose.resize(7,1);

    _Arm.init(nh);
}


void UAV::state_Callback(const mavros_msgs::StateConstPtr &msg)
{
    _current_state = *msg;
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
    
    _hit_pose[0] = msg->pose.pose.position.x;
    _hit_pose[1] = msg->pose.pose.position.y;
    _hit_pose[2] = msg->pose.pose.position.z;
    _hit_pose[3] = msg->twist.twist.linear.x;
    _hit_pose[4] = msg->twist.twist.linear.y;
    _hit_pose[5] = msg->twist.twist.linear.z;
    _hit_pose[6] = msg->header.stamp.toSec();

    hit2base();
    
    double vel_arm[2] = {0, sqrt(_hit_pose[3]*_hit_pose[3] + _hit_pose[4]*_hit_pose[4] + _hit_pose[5]*_hit_pose[5])};
    for(int i = 0; i < 2; i++)
    {
        Eigen::Vector2d P_s = {_arm_start[i], _hit_pose[6]-_arm_time_pass[0]-_arm_time_pass[1]};
        Eigen::Vector2d P_p = {_arm_hit_pos[i], _hit_pose[6]-_arm_time_pass[1]};
        Eigen::Vector2d P_e = {_arm_end[i], _hit_pose[6]};
        _arm_pos_target[i] =  common_tools::triangleProfile(P_s, P_e, P_p, vel_arm[i]);
    }


    _arm_pos_target_index = 0;

    _model = hit;
}


void UAV::setArmParam(double* arm_length, double* arm_offset, Eigen::Vector3d axis2link, Eigen::Vector3d arm2base, double* arm_start, double* arm_end, double* arm_time_pass)
{
    _arm_length[0] = arm_length[0], _arm_length[1] = arm_length[1];
    _arm_offset[0] = arm_offset[0], _arm_offset[1] = arm_offset[1];

    _axis2link = axis2link;
    _arm2base = arm2base;

    _arm_start[0] = arm_start[0], _arm_start[1] = arm_start[1];
    _arm_end[0] = arm_end[0], _arm_end[0] = arm_end[0];
    _arm_time_pass[0] = arm_time_pass[0], _arm_time_pass[1] = arm_time_pass[1];
}

void UAV::takeOff()
{
    static int i = 0;
    if(i++ < 100)
    {
        _local_pose_pub.publish(_pose_hover);
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    static ros::Time last_request = ros::Time::now();

    if( _current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(3.0))){
        if( _set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    } else {
        if( !_current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( _arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }

    _pose_hover.header.stamp = ros::Time::now();
    _local_pose_pub.publish(_pose_hover);


    static int cnt = 0;
    if(abs(_currentPose.pose.position.z - _pose_hover.pose.position.z) < 0.2)
    {
        cnt++;
    }
    if(cnt > 100)
    {
        _model = hover;
        cnt = 0;
    }
}

void UAV::Hover()
{
    _pose_hover.header.stamp = ros::Time::now();

    // Eigen::Vector3d rpy = {0, 0, 1.68};
    // geometry_msgs::Pose temp;
    // temp.orientation = common_tools::Euler2orientation(rpy);
    // _pose_hover.pose.orientation = temp.orientation;

    _local_pose_pub.publish(_pose_hover);

    static bool isSet = false;
    if(!isSet)
    {
        isSet = _Arm.GetSet();
    }

}

void UAV::Hit()
{    
    _targetPoint.header.stamp = ros::Time::now();
    _targetPoint.pose.position.x = _base_pose[0];
    _targetPoint.pose.position.y = _base_pose[1];
    _targetPoint.pose.position.z = _base_pose[2];
    // ROS_INFO("x:%f, y:%f, z:%f", _base_pose[0], _base_pose[1], _base_pose[2]);
    
    // Eigen::Vector3d rpy = {_base_pose[3], _base_pose[4], _base_pose[5]};
    // _targetPoint.pose.orientation = common_tools::Euler2orientation(rpy);

    _local_pose_pub.publish(_targetPoint);

    // _Arm.ctrlArm(_arm_pos_target[0][_arm_pos_target_index][0], _arm_pos_target[1][_arm_pos_target_index][0]);
    _arm_pos_target_index += 1;
}


void UAV::printData()
{
    switch (_model)
    {
    case wait:
        ROS_INFO("wait...");
        break;
    case take_off:
        ROS_INFO("take_off");
        break;
    case hover:
        ROS_INFO("hover");
        break;
    case hit:
        ROS_INFO("hit");
        break;
    case land:
        ROS_INFO("land");
        break;
    case manual:
        ROS_INFO("mannual");
        break;
    default:
        break;
    }
}

void UAV::hit2base()
{
    double a = 0.5;
    double b = 1 - a;    

    Eigen::Vector3d Pt = {_hit_pose[0], _hit_pose[1], _hit_pose[2]};
    double m1_h = _hit_pose[3];
    double n1_h = _hit_pose[4];
    double p1_h = -(_hit_pose[3]*_hit_pose[3] + _hit_pose[4]*_hit_pose[3])/_hit_pose[5];
    Eigen::Vector3d e1_h = {m1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h), n1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h), p1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h)};
    Eigen::Vector3d Pa_h = -_arm_length[1]*e1_h + Pt;
    _arm_hit_pos[1] = atan2(p1_h, sqrt(m1_h*m1_h+n1_h*n1_h)) + 6.28;


    Eigen::Vector3d P_base = {_currentPose.pose.position.x, _currentPose.pose.position.y, _currentPose.pose.position.z};
    Eigen::Vector3d Pb_k = P_base + _arm2base + _axis2link;
    double tan0 = tan(_Arm._arm_current_pos[0] + _arm_offset[0]);
    double p2_k = -sqrt(tan0*tan0/(1+tan0*tan0));
    double p2 = (a*_arm_length[0]*(Pa_h[2]-Pb_k[2]) + b*p2_k)/(a*_arm_length[0]*_arm_length[0] + b);
    Eigen::Vector3d e2_h = {m1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h), n1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h), p2};
    e2_h.normalize();
    Eigen::Vector3d Pb_h = -_arm_length[0]*e2_h + Pa_h;
    _arm_hit_pos[0] = atan2(p2, sqrt(m1_h*m1_h+n1_h*n1_h)) + 6.28;

    Eigen::Vector3d P_base_h = Pb_h - _arm2base - _axis2link;
    _base_pose[0] = P_base_h[0], _base_pose[1] = P_base_h[1], _base_pose[2] = P_base_h[2]; 
    _base_pose[3] = 0, _base_pose[4] = 0, _base_pose[5] = tan(m1_h/n1_h);
    _base_pose[6] = _hit_pose[6];

    
}


void UAV::mainLoop()
{
    ros::Rate loop_rate(_ctrl_rate);

    int i = 0;
    while(ros::ok())
    {
        switch (_model)
        {
        case wait:
            if(_current_state.connected)
            {
                _model = take_off;
            }
            break;
        
        case take_off:
            takeOff();
            break;

        case hover:
            Hover();
            break;
        
        case hit:
            Hit();
            break;

        default:
            break;
        }

        if(i++ > 500)
        {
            printData();
            i = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}