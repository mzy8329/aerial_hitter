#include "AerialArm_inkind.h"


AerialArm_inkind::AerialArm_inkind(ros::NodeHandle nh, double ctrl_freq)
{
    _nh = nh;

    _ctrlData_pub = nh.advertise<motor_serial::motor_ctrl>("/motor_serial/ctrl_data", 10);
    _motorData_sub = nh.subscribe("/motor_serial/motor_data", 10, &AerialArm_inkind::_motorDataCallback, this);

    _ctrl_rate = ctrl_freq;

    _motor[0].id = 1, _motor[1].id = 2;
}

void AerialArm_inkind::init(ros::NodeHandle nh, double ctrl_freq)
{
    _nh = nh;

    _ctrlData_pub = nh.advertise<motor_serial::motor_ctrl>("/motor_serial/ctrl_data", 10);
    _motorData_sub = nh.subscribe("/motor_serial/motor_data", 10, &AerialArm_inkind::_motorDataCallback, this);

    _ctrl_rate = ctrl_freq;

    _motor[0].id = 1, _motor[1].id = 2;
}



bool AerialArm_inkind::GetSet()
{
    ctrlArm(Arm_POS_SET[0], Arm_POS_SET[1]);
    if(abs(_motor[0].angle_fdb-Arm_POS_SET[0])<5
    && abs(_motor[1].angle_fdb-Arm_POS_SET[1])<5)
    {
        return true;
    }
    return false;
}

bool AerialArm_inkind::toZero()
{
    ctrlArm(0, 0);
    if(abs(_motor[0].angle_fdb)<5
    && abs(_motor[1].angle_fdb)<5)
    {
        return true;
    }
    return false;
}

void AerialArm_inkind::ctrlArm(double pos_0, double pos_1)
{
    motor_serial::motor_ctrl ctrl_data[2];
    for(int i = 0; i < 2; i++)
    {
        ctrl_data[i].id = _motor[i].id;
        ctrl_data[i].angle_ref = pos_0;
        ctrl_data[i].rpm_ref = 0;
        ctrl_data[i].current_ref = 0;

        _ctrlData_pub.publish(ctrl_data[i]);
    }

    ros::spinOnce();
}



void AerialArm_inkind::_motorDataCallback(const motor_serial::motor_data::ConstPtr &motor_msg)
{
    for(int i = 0; i < 2; i++)
    {
        if(motor_msg->id != _motor[i].id)
        {
            continue;
        }

        _motor[i].angle_fdb = motor_msg->angle_fdb;
        _motor[i].rpm_fdb = motor_msg->rpm_fdb;
        _motor[i].torque_fdb = motor_msg->torque_fdb;
    }
}