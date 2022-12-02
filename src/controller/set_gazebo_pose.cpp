/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

constexpr double INIT_X = 0.;
constexpr double INIT_Y = 0.;
constexpr double INIT_Z = 1.5;

constexpr double FINAL_X = 0.0;
constexpr double FINAL_Y = 0.;
constexpr double FINAL_Z = INIT_Z;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped cmd_pose;
trajectory_msgs::MultiDOFJointTrajectory cmd_traj;
std_msgs::Bool planned;
std_msgs::Bool landed;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
}

void cmd_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    cmd_pose = *msg;
}

void cmd_traj_cb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg)
{
    cmd_traj = *msg;
}
void key_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
}

void plan_cb(const std_msgs::Bool::ConstPtr &msg)
{
    planned = *msg;
}

void land_cb(const std_msgs::Bool::ConstPtr &msg)
{
    landed = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_gazebo_pose_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pose_cb);
    ros::Subscriber key_sub = nh.subscribe<geometry_msgs::Twist>("key_vel", 10, key_cb);
    ros::Subscriber plan_sub = nh.subscribe<std_msgs::Bool>("plan_result", 10, plan_cb);
    ros::Subscriber land_sub = nh.subscribe<std_msgs::Bool>("/aerial_catcher/land", 10, land_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aerial_catcher/pose", 10, cmd_pose_cb);
    ros::Subscriber waypoint_sub_ = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/aerial_catcher/setpoint_trajectory", 10, cmd_traj_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher waypoint_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("mavros/setpoint_trajectory/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/land");

    bool in_flight_flag = false;
    landed.data = false;
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = INIT_X;
    pose.pose.position.y = INIT_Y;
    pose.pose.position.z = INIT_Z;
    pose.pose.orientation.x = 0.;
    pose.pose.orientation.y = 0.;
    pose.pose.orientation.z = 0.;
    pose.pose.orientation.w = 1.;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    ros::V_string AllNodes;
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if (arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (ros::Time::now() - last_request > ros::Duration(5.0))
            {
                pose.pose.position.x = FINAL_X;
                pose.pose.position.y = FINAL_Y;
                pose.pose.position.z = FINAL_Z;
            }
        }

        ros::master::getNodes(AllNodes);
        for (auto &node_name : AllNodes)
        {
            if (node_name == "/scenario_ardrone_pose_tracking_node" || node_name == "/mpc_uav_trajectory_publisher")
            {
                std::cout << node_name << std::endl;
                exit(0);
            }
            else if (planned.data == true)
            {
                std::cout << "get a plan" << std::endl;
                // exit(0);
                break;
            }
        }
        if (planned.data == true)
        {
            std::cout << "get a plan" << std::endl;
            break;
        }
        local_pos_pub.publish(pose);
        if (landed.data)
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Land sent");
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    while (ros::ok())
    {
        if (landed.data)
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Land sent");
                break;
            }
        }
        else
        {
            waypoint_pub_.publish(cmd_traj);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}