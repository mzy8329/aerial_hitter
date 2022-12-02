#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_data_feedback_node");
  ros::NodeHandle nh;

  ros::Publisher pos_pub =
    nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
  ros::ServiceClient get_current_pose =
    nh.serviceClient<gazebo_msgs::GetLinkState>(
      "/gazebo/get_link_state");  // "hummingbird" should be noticed

  ros::Rate rate(50.0);

  geometry_msgs::PoseStamped pose;
  gazebo_msgs::LinkState base_link_state;
  gazebo_msgs::GetLinkState link_state_request;

  link_state_request.request.link_name =
    "hummingbird::base_link";  // notice we should specify the model's name
  link_state_request.request.reference_frame = "world";

  while (ros::ok()) {
    get_current_pose.call(link_state_request);
    base_link_state = link_state_request.response.link_state;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";

    pose.pose.position.x = base_link_state.pose.position.x;
    pose.pose.position.y = base_link_state.pose.position.y;
    pose.pose.position.z = base_link_state.pose.position.z;

    pose.pose.orientation.w = base_link_state.pose.orientation.w;
    pose.pose.orientation.x = base_link_state.pose.orientation.x;
    pose.pose.orientation.y = base_link_state.pose.orientation.y;
    pose.pose.orientation.z = base_link_state.pose.orientation.z;

    pos_pub.publish(pose);

    static tf::TransformBroadcaster br;
    tf::Transform uav_tf;

    // publish uav tf in world frame
    uav_tf.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y,
                                 pose.pose.position.z));
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    uav_tf.setRotation(q);
    br.sendTransform(
      // tf::StampedTransform(uav_tf, ros::Time::now(), "world", "hummingbird"));
      tf::StampedTransform(uav_tf, ros::Time::now(), "world", "uav"));

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
