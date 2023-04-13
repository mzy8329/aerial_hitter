#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Trigger.h>
#include <gazebo_msgs/DeleteModel.h>

ros::Publisher throw_pub;


bool throwBall(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res)
{
    geometry_msgs::Wrench throw_force;
    throw_force.force.x = -0.035;
    throw_force.force.y = 0;
    throw_force.force.z = 0.120;
    for(int i = 0; i < 50; i++)
    {
        throw_pub.publish(throw_force);
        sleep(0.01);
    }
    
    throw_force.force.x = 0;
    throw_force.force.y = 0;
    throw_force.force.z = 0;
    throw_pub.publish(throw_force);


    res.success = true;

    return true;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "throw_ball");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // ros::ServiceClient ball_del_client = nh.serviceClient<gazebo_msgs::DeleteModel>("ball_del");
    // gazebo_msgs::DeleteModel red_ball;
    // red_ball.request.model_name = "red_ball";
    
    // if(ball_del_client.call(red_ball))
    // {
    //     ROS_INFO("delete success");
    // }

    throw_pub = nh.advertise<geometry_msgs::Wrench>("/ball_force", 1);
    ros::ServiceServer throw_srv = nh.advertiseService("/throw_ball/throw", throwBall);

    ros::spin();



    return 0;
}