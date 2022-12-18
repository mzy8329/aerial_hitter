#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "rviz_draw.h"
#include "TrajPredict.h"


ros::Publisher marker_pub;
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

Eigen::Vector3d point_target = {0.5, 0.1, 0};

void ballPoseCallBack(const geometry_msgs::PoseStampedConstPtr &body_msg)
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

    ros::Subscriber ballPose_sub = nh.subscribe("/vrpn_client_node/RigidBody3/pose", 10, ballPoseCallBack);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/traj_view", 0);

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
                    
                    mark = rviz_draw::draw(flyPre.getPoint_hit(), colar_hitPoint, "traj_hitPoint");
                    marker_pub.publish(mark);
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