#ifndef AERIALARM_SIM_H
#define AERIALARM_SIM_H


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Eigen>


#include "common_tools.h"

#define SIGN(x) (x>0 ? 1:(x<0 ? -1:0))


double Arm_POS_SET[2] = {0.3, -1.8};
double Arm_VEL_SET[2] = {1.2, 1.2};



class AerialArm_sim
{
public:
    AerialArm_sim(){;;};
    AerialArm_sim(ros::NodeHandle nh);
    ~AerialArm_sim(){};

    void init(ros::NodeHandle nh, double ctrl_freq);

    bool setArmParam(double arm_0_length, double arm_1_length, double arm_0_offset, double arm_1_offset, Eigen::Vector3d axis2link);

    void arm0PosCallback(const std_msgs::Float32ConstPtr &msg);
    void arm1PosCallback(const std_msgs::Float32ConstPtr &msg);


    bool GetSet(double pos_0_s = 0, double pos_1_s = 0);
    bool toZero(double pos_0_s, double pos_1_s);

    void ctrlArm(double pos_0, double pos_1);
    
    double _arm_current_pos[2];
    double _arm_target_pos[2];
    double _arm_target_vel[2];

    double _hit_start_pos[2];
    double _hit_end_pos[2];

private:
    ros::NodeHandle _nh;
    ros::Subscriber _arm_pos_sub[2];
    ros::Publisher _arm_pos_pub[2];

    double _ctrl_rate;

    

};





#endif