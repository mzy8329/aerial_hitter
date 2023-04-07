#include "AerialArm_sim.h"


AerialArm_sim::AerialArm_sim(ros::NodeHandle nh)
{
    _nh = nh;    

    _arm_pos_pub[0] = _nh.advertise<std_msgs::Float32>("/hummingbird_arm/aerial_arm/arm_0_joint/pos_cmd", 1);
    _arm_pos_pub[1] = _nh.advertise<std_msgs::Float32>("/hummingbird_arm/aerial_arm/arm_1_joint/pos_cmd", 1);

    _arm_pos_sub[0] = _nh.subscribe("/AerialArm_sim/arm_0_joint/pose", 10, &AerialArm_sim::arm0PosCallback, this);
    _arm_pos_sub[1] = _nh.subscribe("/AerialArm_sim/arm_1_joint/pose", 10, &AerialArm_sim::arm1PosCallback, this);
}

void AerialArm_sim::init(ros::NodeHandle nh, double ctrl_freq)
{
    _nh = nh;    

    _arm_pos_pub[0] = _nh.advertise<std_msgs::Float32>("/hummingbird_arm/aerial_arm/arm_0_joint/pos_cmd", 1);
    _arm_pos_pub[1] = _nh.advertise<std_msgs::Float32>("/hummingbird_arm/aerial_arm/arm_1_joint/pos_cmd", 1);

    _arm_pos_sub[0] = _nh.subscribe("/AerialArm_sim/arm_0_joint/pose", 10, &AerialArm_sim::arm0PosCallback, this);
    _arm_pos_sub[1] = _nh.subscribe("/AerialArm_sim/arm_1_joint/pose", 10, &AerialArm_sim::arm1PosCallback, this);

    _ctrl_rate = ctrl_freq;
}

bool AerialArm_sim::GetSet(double pos_0_s, double pos_1_s)
{
    _arm_target_pos[0] = Arm_POS_SET[0], _arm_target_pos[1] = Arm_POS_SET[1];
    _arm_target_vel[0] = Arm_VEL_SET[0], _arm_target_vel[1] = Arm_VEL_SET[1];
    double arm_pos_err[2] = {(_arm_target_pos[0] - _arm_current_pos[0]), (_arm_target_pos[1] - _arm_current_pos[1])};
    
    if(abs(arm_pos_err[0]) < (2.0/_ctrl_rate * _arm_target_vel[0]) && abs(arm_pos_err[1]) < (2.0/_ctrl_rate * _arm_target_vel[1])) 
    {
        std::cout<<"---------- Set Over ----------"<<std::endl;
        ctrlArm(_arm_target_pos[0],_arm_target_pos[1]);
        
        return true;
    }

    static double pos[2] = {0, 0};
    if(pos_0_s != 0 || pos_1_s != 0)
    {
        pos[0] = pos_0_s, pos[1] = pos_1_s;
    }

    for(int i = 0; i < 2; i++)
    {
        if(abs(arm_pos_err[i]) >= 2.0/_ctrl_rate * _arm_target_vel[i])
        {
            pos[i] += 1.0/_ctrl_rate * _arm_target_vel[i] * SIGN(arm_pos_err[i]);
        }
        else
        {
            pos[i] = _arm_target_pos[i];
        }
    }
    ctrlArm(pos[0],pos[1]);

    return false;
}

bool AerialArm_sim::toZero(double pos_0_s, double pos_1_s)
{
    _arm_target_pos[0] = 0, _arm_target_pos[1] = 0;
    _arm_target_vel[0] = Arm_VEL_SET[0], _arm_target_vel[1] = Arm_VEL_SET[1];
    double arm_pos_err[2] = {(_arm_target_pos[0] - _arm_current_pos[0]), (_arm_target_pos[1] - _arm_current_pos[1])};
    
    if(abs(arm_pos_err[0]) < (2.0/_ctrl_rate * _arm_target_vel[0]) && abs(arm_pos_err[1]) < (2.0/_ctrl_rate * _arm_target_vel[1])) 
    {
        std::cout<<"---------- toZero Over ----------"<<std::endl;
        ctrlArm(_arm_target_pos[0],_arm_target_pos[1]);
        
        return true;
    }


    double pos[2] = {pos_0_s, pos_1_s};
    for(int i = 0; i < 2; i++)
    {
        if(abs(arm_pos_err[i]) >= 2.0/_ctrl_rate * _arm_target_vel[i])
        {
            pos[i] += 1.0/_ctrl_rate * _arm_target_vel[i] * SIGN(arm_pos_err[i]);
        }
        else
        {
            pos[i] = _arm_target_pos[i];
        }
    }
    ctrlArm(pos[0],pos[1]);

    return false;
}

void AerialArm_sim::ctrlArm(double pos_0, double pos_1)
{
    std_msgs::Float32 temp_data;
    temp_data.data = float(pos_0);
    _arm_pos_pub[0].publish(temp_data);

    temp_data.data = float(pos_1 - pos_0);
    _arm_pos_pub[1].publish(temp_data);

    ros::spinOnce();
}


void AerialArm_sim::arm0PosCallback(const std_msgs::Float32ConstPtr &msg)
{
    _arm_current_pos[0] = msg->data;
}

void AerialArm_sim::arm1PosCallback(const std_msgs::Float32ConstPtr &msg)
{
    _arm_current_pos[1] = msg->data;
}