#ifndef AERIALARM_H
#define AERIALARM_H


#include <ros/ros.h>
#include <std_msgs/Float32.h>

#define SIGN(x) (x>0 ? 1:(x<0 ? -1:0))

#define ArmNum 2

double Arm_POS_SET[2] = {0.5, -1.5};
double Arm_VEL_SET[2] = {0.3, 0.4};


class AerialArm
{
public:
    AerialArm(ros::NodeHandle nh);
    ~AerialArm(){};

    void arm0PosCallback(const std_msgs::Float32ConstPtr &msg);
    void arm1PosCallback(const std_msgs::Float32ConstPtr &msg);


    bool GetSet();
    bool toZero();
    
    double _arm_current_pos[ArmNum];
    double _arm_target_pos[ArmNum];
    double _arm_target_vel[ArmNum];


private:
    ros::NodeHandle _nh;
    ros::Subscriber _arm_pos_sub[ArmNum];
    ros::Publisher _arm_pos_pub[ArmNum];

};





#endif