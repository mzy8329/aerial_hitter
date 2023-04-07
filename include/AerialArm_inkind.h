#ifndef AERIALARM_INKIND_H
#define AERIALARM_INKIND_H


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Eigen>


#include "common_tools.h"
#include "motor_serial/motor_data.h"
#include "motor_serial/motor_ctrl.h"

#define SIGN(x) (x>0 ? 1:(x<0 ? -1:0))


double Arm_POS_SET[2] = {40, -150};


class AerialArm_inkind
{
public:
    AerialArm_inkind(){;;};
    AerialArm_inkind(ros::NodeHandle nh, double ctrl_freq);
    ~AerialArm_inkind(){};

    void init(ros::NodeHandle nh, double ctrl_freq);

    void _motorDataCallback(const motor_serial::motor_dataConstPtr &msg);

    bool GetSet();
    bool toZero();

    void ctrlArm(double pos_0, double pos_1);
    

    motor_serial::motor_data _motor[2];
    double _arm_target_pos[2];
    double _arm_target_vel[2];

    double _hit_start_pos[2];
    double _hit_end_pos[2];

private:
    ros::NodeHandle _nh;
    ros::Subscriber _motorData_sub;
    ros::Publisher _ctrlData_pub;

    double _ctrl_rate;

    

};





#endif