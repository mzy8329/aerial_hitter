#include "rviz_draw.h"



namespace rviz_draw
{
    visualization_msgs::Marker draw(
        std::vector<Eigen::Vector3d> points,
        float* colar,
        const char* frameId, 
        int id,     
        uint type, 
        uint action,
        int duration)
    {

        visualization_msgs::Marker visual;
        visual.header.frame_id = frameId;
        visual.header.stamp = ros::Time::now();

        visual.ns = frameId;
        visual.id = id;

        visual.type = type;
        visual.action = action;
        visual.color.a = colar[0];
        visual.color.r = colar[1];
        visual.color.g = colar[2];
        visual.color.b = colar[3];

        visual.scale.x = 0.05;
        visual.scale.y = 0.05;
        visual.scale.z = 0.05;

        visual.pose.orientation.x = 0.0;
        visual.pose.orientation.y = 0.0;
        visual.pose.orientation.z = 0.0;
        visual.pose.orientation.w = 1.0;

        visual.lifetime = ros::Duration(duration);
        visual.points.clear();

        geometry_msgs::Point pt;
        for(int i = 0; i < points.size(); i++)
        {
            pt.x = points[i][0];
            pt.y = points[i][1];
            pt.z = points[i][2];
            visual.points.push_back(pt);
        }

        return visual;
    }

    visualization_msgs::Marker draw(
        std::vector<Eigen::Vector4d> points,
        float* colar,
        const char* frameId, 
        int id,     
        uint type, 
        uint action,
        int duration)
    {

        visualization_msgs::Marker visual;
        visual.header.frame_id = frameId;
        visual.header.stamp = ros::Time::now();

        visual.ns = frameId;
        visual.id = id;

        visual.type = type;
        visual.action = action;
        visual.color.a = colar[0];
        visual.color.r = colar[1];
        visual.color.g = colar[2];
        visual.color.b = colar[3];

        visual.scale.x = 0.1;
        visual.scale.y = 0.1;
        visual.scale.z = 0.1;

        visual.pose.orientation.x = 0.0;
        visual.pose.orientation.y = 0.0;
        visual.pose.orientation.z = 0.0;
        visual.pose.orientation.w = 1.0;

        visual.lifetime = ros::Duration(duration);
        visual.points.clear();

        geometry_msgs::Point pt;
        for(int i = 0; i < points.size(); i++)
        {
            pt.x = points[i][0];
            pt.y = points[i][1];
            pt.z = points[i][2];
            visual.points.push_back(pt);
        }

        return visual;
    }


    visualization_msgs::Marker draw(
        Eigen::Vector3f point,
        float* colar,
        const char* frameId, 
        int id,     
        uint type, 
        uint action,
        int duration)
    {

        visualization_msgs::Marker visual;
        visual.header.frame_id = frameId;
        visual.header.stamp = ros::Time::now();

        visual.ns = frameId;
        visual.id = id;

        visual.type = type;
        visual.action = action;
        visual.color.a = colar[0];
        visual.color.r = colar[1];
        visual.color.g = colar[2];
        visual.color.b = colar[3];

        visual.scale.x = 0.02;
        visual.scale.y = 0.02;
        visual.scale.z = 0.02;


        visual.pose.position.x = point[0];
        visual.pose.position.y = point[1];
        visual.pose.position.z = point[2];

        visual.pose.orientation.x = 0.0;
        visual.pose.orientation.y = 0.0;
        visual.pose.orientation.z = 0.0;
        visual.pose.orientation.w = 1.0;

        visual.lifetime = ros::Duration(duration);
        visual.points.clear();

        // geometry_msgs::Point pt;
 
        // pt.x = point[0];
        // pt.y = point[1];
        // pt.z = point[2];
        // visual.points.push_back(pt);


        return visual;
    }

    visualization_msgs::Marker draw(
        Eigen::Vector4d point,
        float* colar,
        const char* frameId, 
        int id,     
        uint type, 
        uint action,
        int duration)
    {

        visualization_msgs::Marker visual;
        visual.header.frame_id = frameId;
        visual.header.stamp = ros::Time::now();

        visual.ns = frameId;
        visual.id = id;

        visual.type = type;
        visual.action = action;
        visual.color.a = colar[0];
        visual.color.r = colar[1];
        visual.color.g = colar[2];
        visual.color.b = colar[3];

        visual.scale.x = 0.05;
        visual.scale.y = 0.05;
        visual.scale.z = 0.05;


        visual.pose.position.x = point[0];
        visual.pose.position.y = point[1];
        visual.pose.position.z = point[2];

        visual.pose.orientation.x = 0.0;
        visual.pose.orientation.y = 0.0;
        visual.pose.orientation.z = 0.0;
        visual.pose.orientation.w = 1.0;

        visual.lifetime = ros::Duration(duration);
        visual.points.clear();

        geometry_msgs::Point pt;
 
        pt.x = point[0];
        pt.y = point[1];
        pt.z = point[2];
        visual.points.push_back(pt);


        return visual;
    }

    visualization_msgs::Marker draw(
        Eigen::VectorXd point,
        float* colar,
        const char* frameId, 
        int id,     
        uint type, 
        uint action,
        int duration)
    {

        visualization_msgs::Marker visual;
        visual.header.frame_id = frameId;
        visual.header.stamp = ros::Time::now();

        visual.ns = frameId;
        visual.id = id;

        visual.type = type;
        visual.action = action;
        visual.color.a = colar[0];
        visual.color.r = colar[1];
        visual.color.g = colar[2];
        visual.color.b = colar[3];

        visual.scale.x = 0.02;
        visual.scale.y = 0.05;
        visual.scale.z = 0.0;

        // visual.pose.position.x = point[0];
        // visual.pose.position.y = point[1];
        // visual.pose.position.z = point[2];

        visual.pose.orientation.x = 0.0;
        visual.pose.orientation.y = 0.0;
        visual.pose.orientation.z = 0.0;
        visual.pose.orientation.w = 1.0;

        visual.points.clear();
        geometry_msgs::Point pt;
        pt.x = point[0], pt.y = point[1], pt.z = point[2];
        visual.points.push_back(pt);
        pt.x = point[0]+point[3]*0.1, pt.y = point[1]+point[4]*0.1, pt.z = point[2]+point[5]*0.1;
        visual.points.push_back(pt);
        visual.lifetime = ros::Duration(duration);


        return visual;
    }




} // namespace rviz_draw
