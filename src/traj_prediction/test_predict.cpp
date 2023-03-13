#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "rviz_draw.h"
#include "TrajPredict.h"


ros::Publisher marker_pub;
ros::Publisher hitPoint_pub;

visualization_msgs::Marker mark;
float colar_pose[4] = {1, 0, 0.8, 0.5};
float colar_traj[4] = {1, 0.5, 0.2, 0.2};
float colar_hit[4] = {1, 0.2, 0.2, 0.6};
float colar_hitPoint[4] = {0.8, 1.0, 0.2, 0.6};

TrajPredict flyPre;
int fit_len = 20;
int check_len = 10;
int fitKd_len = 20;
float freeFallCheck_err = 1.75;
float pre_time = 1.0;
int pre_size = 10;
double beta = 0.3;

Eigen::Vector3d point_target = {1.0, 0.5, 0};

void ballPoseVrpnCallBack(const geometry_msgs::PoseStampedConstPtr &body_msg)
{
    static int i_1 = 0;
    static int i_2 = 0;
    Eigen::Vector4d point;
    point << body_msg->pose.position.x, body_msg->pose.position.y, body_msg->pose.position.z, body_msg->header.stamp.toSec();

    if(i_2++ > 3)
    {
        mark = rviz_draw::draw(point, colar_pose, "traj_view", i_1++);
        marker_pub.publish(mark);

        i_2 = 0;
    }
    
    flyPre.pushNewPoint(point);
}

void ballPoseGazeboCallBack(const nav_msgs::OdometryConstPtr &body_msg)
{
    static int i_1 = 0;
    static int i_2 = 0;
    Eigen::Vector4d point;
    point << body_msg->pose.pose.position.x, body_msg->pose.pose.position.y, body_msg->pose.pose.position.z, body_msg->header.stamp.toSec();

    if(i_2++ > 3)
    {
        mark = rviz_draw::draw(point, colar_pose, "traj_view", i_1++);
        marker_pub.publish(mark);

        i_2 = 0;
    }
    
    flyPre.pushNewPoint(point);
}

void param_Init(ros::NodeHandle nh)
{
    nh.getParam("/traj_prediction/fit_len", fit_len);
    nh.getParam("/traj_prediction/check_len", check_len);
    nh.getParam("/traj_prediction/fitKd_len", fitKd_len);

    nh.getParam("/freeFallCheck_err", freeFallCheck_err);
    nh.getParam("/pre_time", pre_time);
    nh.getParam("/pre_size", pre_size);
    nh.getParam("/beta", beta);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pretict_catch");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(50);

    ros::Subscriber ballPoseVrpn_sub = nh.subscribe("/vrpn_client_node/RigidBody3/pose", 10, ballPoseVrpnCallBack);
    ros::Subscriber ballPoseGazebo_sub = nh.subscribe("/ball_odom", 10, ballPoseGazeboCallBack);

    marker_pub = nh.advertise<visualization_msgs::Marker>("/traj_view", 1);
    hitPoint_pub = nh.advertise<geometry_msgs::PoseArray>("/UAV/hitPoint", 1);

    param_Init(nh);
    flyPre.init(fit_len, check_len, freeFallCheck_err, beta);

    while (ros::ok())
    {
        /* code */             
        if(flyPre.predictTraj_gOnly(pre_time, pre_size))
        {
            std::cout<<"predict"<<std::endl;
            std::vector<Eigen::Vector4d> traj_predict = flyPre.getTraj();

            mark = rviz_draw::draw(traj_predict, colar_traj, "traj_predict");
            marker_pub.publish(mark);

            
            for(int i = 0; i < traj_predict.size(); i++)
            {
                if(flyPre.getTraj()[i][2]<0.5) break;;
                if(flyPre.predictTraj_hit(i, point_target))
                {
                    mark = rviz_draw::draw(flyPre.getTraj_hit(), colar_hit, "traj_hit");
                    marker_pub.publish(mark);
                    
                    Eigen::VectorXd hitPoint = flyPre.getPoint_hit();
                    mark = rviz_draw::draw(hitPoint, colar_hitPoint, "traj_hitPoint");
                    marker_pub.publish(mark);


                    nav_msgs::Odometry hit_point;
                    hit_point.header.stamp = ros::Time(hitPoint[6]);
                    hit_point.pose.pose.position.x = hitPoint[0];
                    hit_point.pose.pose.position.y = hitPoint[1];
                    hit_point.pose.pose.position.z = hitPoint[2];
                    hit_point.twist.twist.linear.x = hitPoint[3];
                    hit_point.twist.twist.linear.y = hitPoint[4];
                    hit_point.twist.twist.linear.z = hitPoint[5];
                    
                    if(isnan(hitPoint[3]) || isnan(hitPoint[4]) || isnan(hitPoint[5])
                    || hitPoint[2] < 0.1
                    )
                    {
                        ;;
                    }
                    else
                    {
                        hitPoint_pub.publish(hit_point);
                    }
                    
                }
            }
            
        }

        // if(flyPre.fitKd(fitKd_len))
        // {
        //     double* Kd = flyPre.getKd();
        //     std::cout<<Kd[0]<<"  "<<Kd[1]<<std::endl;
        // }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}