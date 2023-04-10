#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "motor_serial/motor_data.h"
#include "motor_serial/motor_ctrl.h"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "impe_ctrl");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);

    ros::Publisher ctrlData_pub = nh.advertise<motor_serial::motor_ctrl>("/motor_serial/ctrl_data", 10);

    motor_serial::motor_ctrl ctrlData;
    while (ros::ok())
    {
        ctrlData.id = 1;
        ctrlData.angle_ref = -1;
        ctrlData.rpm_ref = -1;
        ctrlData.current_ref = 0;
        ctrlData_pub.publish(ctrlData);

        ctrlData.id = 2;
        ctrlData.angle_ref = -1;
        ctrlData.rpm_ref = -1;
        ctrlData.current_ref = 0;
        ctrlData_pub.publish(ctrlData);

        loop_rate.sleep();
    }


    return 0;
}
