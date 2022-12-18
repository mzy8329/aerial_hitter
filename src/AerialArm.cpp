#include "AerialArm.h"


AerialArm::AerialArm(ros::NodeHandle nh)
{
    _nh = nh;    

    _arm_pos_pub[0] = _nh.advertise<std_msgs::Float32>("/hummingbird_arm/aerial_arm/arm_0_joint/pos_cmd", 1);
    _arm_pos_pub[1] = _nh.advertise<std_msgs::Float32>("/hummingbird_arm/aerial_arm/arm_1_joint/pos_cmd", 1);

    _arm_pos_sub[0] = _nh.subscribe("/AerialArm/arm_0_joint/pose", 10, &AerialArm::arm0PosCallback, this);
    _arm_pos_sub[1] = _nh.subscribe("/AerialArm/arm_1_joint/pose", 10, &AerialArm::arm1PosCallback, this);
}


bool AerialArm::GetSet()
{
    double ctrl_rate = 10;
    ros::Rate loop_rate(ctrl_rate);

    double arm_pos_err[2] = {999, 999};
    std_msgs::Float32 pos_pub[2];


    for(int i = 0; i < ArmNum; i++)
    {
        _arm_target_pos[i] = Arm_POS_SET[i];
        _arm_target_vel[i] = Arm_VEL_SET[i];
        
        arm_pos_err[i] = (_arm_target_pos[i] - _arm_current_pos[i]);
        pos_pub[i].data = _arm_current_pos[i];
    }
    
    while(abs(arm_pos_err[0]) > 0.1 || abs(arm_pos_err[1]) > 0.1)
    {
        for(int i = 0; i < ArmNum; i++)
        {
            arm_pos_err[i] = _arm_target_pos[i] - _arm_current_pos[i];
            if(abs(arm_pos_err[i]) > 0.1)
            {
                pos_pub[i].data += 1/ctrl_rate*_arm_target_vel[i] * SIGN(arm_pos_err[i]);
                _arm_pos_pub[i].publish(pos_pub[i]);
            }
        }
        // ROS_INFO("%lf, %lf, %lf, %lf", pos_pub[0].data, _arm_current_pos[0], pos_pub[1].data, _arm_current_pos[1]);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}

bool AerialArm::toZero()
{
    double ctrl_rate = 10;
    ros::Rate loop_rate(ctrl_rate);

    double arm_pos_err[2] = {999, 999};
    std_msgs::Float32 pos_pub[2];

    for(int i = 0; i < ArmNum; i++)
    {
        _arm_target_pos[i] = 0;
        _arm_target_vel[i] = Arm_VEL_SET[i];
        
        arm_pos_err[i] = (_arm_target_pos[i] - _arm_current_pos[i]);
        pos_pub[i].data = _arm_current_pos[i];
    }
    
    while(abs(arm_pos_err[0]) > 0.1 || abs(arm_pos_err[1]) > 0.1)
    {
        for(int i = 0; i < ArmNum; i++)
        {
            arm_pos_err[i] = _arm_target_pos[i] - _arm_current_pos[i];
            if(abs(arm_pos_err[i]) > 0.1)
            {
                pos_pub[i].data += 1/ctrl_rate*_arm_target_vel[i] * SIGN(arm_pos_err[i]);
                _arm_pos_pub[i].publish(pos_pub[i]);
            }
        }
        // ROS_INFO("%lf, %lf, %lf, %lf", pos_pub[0].data, _arm_current_pos[0], pos_pub[1].data, _arm_current_pos[1]);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}


void AerialArm::arm0PosCallback(const std_msgs::Float32ConstPtr &msg)
{
    _arm_current_pos[0] = msg->data;
}

void AerialArm::arm1PosCallback(const std_msgs::Float32ConstPtr &msg)
{
    _arm_current_pos[1] = msg->data;
}