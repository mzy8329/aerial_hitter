#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tfMessage.h>
#include <tf/LinearMath/Transform.h>


ros::Publisher arm_0_pos_pub;
ros::Publisher arm_1_pos_pub;

void arm0CallBack(const std_msgs::Float32ConstPtr &msg)
{
    // static ros::Time time_last = ros::Time::now();
    // if(ros::Time::now().toSec()-time_last.toSec() > 0.01)
    // {
    //     time_last = ros::Time::now();
        
        std_msgs::Float32 temp;
        temp.data = msg->data;
        arm_0_pos_pub.publish(temp);
    // }

    
}

void arm1CallBack(const std_msgs::Float32ConstPtr &msg)
{
    std_msgs::Float32 temp;
    temp.data = msg->data;
    arm_1_pos_pub.publish(temp);

    static ros::Time time_last = ros::Time::now();
    if(ros::Time::now().toSec()-time_last.toSec() > 0.01)
    {
        time_last = ros::Time::now();
        
        std_msgs::Float32 temp;
        temp.data = msg->data;
        arm_1_pos_pub.publish(temp);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_gazebo_pose");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher UAV_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/UAV/pose", 1);
    arm_0_pos_pub = nh.advertise<std_msgs::Float32>("/AerialArm/arm_0_joint/pose", 1);
    arm_1_pos_pub = nh.advertise<std_msgs::Float32>("/AerialArm/arm_1_joint/pose", 1);
    ros::ServiceClient UAV_pos_client = nh.serviceClient<gazebo_msgs::GetLinkState>(
        "/gazebo/get_link_state"
    );
    ros::Subscriber arm_0_pos_sub = nh.subscribe(
        "/hummingbird_arm/aerial_arm/arm_0_joint/pose",
        1,
        arm0CallBack
    );
    ros::Subscriber arm_1_pos_sub = nh.subscribe(
        "/hummingbird_arm/aerial_arm/arm_1_joint/pose",
        1,
        arm1CallBack
    );


    gazebo_msgs::GetLinkState base_link_request;
    gazebo_msgs::LinkState base_link_state;
    geometry_msgs::PoseStamped pose;

    base_link_request.request.link_name = "hummingbird_arm::base_link";
    base_link_request.request.reference_frame = "world";

    while(ros::ok())
    {
        UAV_pos_client.call(base_link_request);
        base_link_state = base_link_request.response.link_state;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";

        pose.pose.position.x = base_link_state.pose.position.x;
        pose.pose.position.y = base_link_state.pose.position.y;
        pose.pose.position.z = base_link_state.pose.position.z;

        pose.pose.orientation.w = base_link_state.pose.orientation.w;
        pose.pose.orientation.x = base_link_state.pose.orientation.x;
        pose.pose.orientation.y = base_link_state.pose.orientation.y;
        pose.pose.orientation.z = base_link_state.pose.orientation.z;

        UAV_pos_pub.publish(pose);

        static tf::TransformBroadcaster br;
        tf::Transform uav_tf;

        uav_tf.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.pose.orientation, q);
        uav_tf.setRotation(q);
        br.sendTransform(tf::StampedTransform(uav_tf, ros::Time::now(), "world", "uav"));


        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}