#ifndef _JOINT_PLUGIN_HH_
#define _JOINT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include <iostream>
#include <string>
#include <boost/algorithm/string/replace.hpp>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class JointPlugin : public ModelPlugin
  {
    public: JointPlugin() {}
    public: virtual void Load(physics::ModelPtr _mode, sdf::ElementPtr _sdf)
    {
      if (_mode->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, position plugin not loaded\n";
        return;
      }
      
      double position = 0.0, p_gain_pos, i_gain_pos, d_gain_pos, p_gain_vel, i_gain_vel, d_gain_vel;

      if (_sdf->HasElement("jointname"))
        joint_name_ori = _sdf->Get<std::string>("jointname");
        joint_name_gz = _mode->GetScopedName() + "::" + joint_name_ori;
        joint_name = _mode->GetScopedName() + "/"  + joint_name_ori; 
        boost::replace_all(joint_name, "::", "/");

        std::cerr << "We find the joint [" <<
        joint_name << "]\n";

      if (_sdf->HasElement("p_gain_pos"))
        p_gain_pos = _sdf->Get<double>("p_gain_pos");
      if (_sdf->HasElement("i_gain_pos"))
        i_gain_pos = _sdf->Get<double>("i_gain_pos");
      if (_sdf->HasElement("d_gain_pos"))
        d_gain_pos = _sdf->Get<double>("d_gain_pos");

      if (_sdf->HasElement("p_gain_vel"))
        p_gain_vel = _sdf->Get<double>("p_gain_vel");
      if (_sdf->HasElement("i_gain_vel"))
        i_gain_vel = _sdf->Get<double>("i_gain_vel");
      if (_sdf->HasElement("d_gain_vel"))
        d_gain_vel = _sdf->Get<double>("d_gain_vel");

      this->model = _mode;
      std::cerr << "\n The model's name is [" <<
        _mode->GetScopedName() << "]\n";
      this->joint = _mode->GetJoint(joint_name_gz);

      this->pid_pos = common::PID(p_gain_pos, i_gain_pos, d_gain_pos);
      this->pid_vel = common::PID(p_gain_vel, i_gain_vel, d_gain_vel);

      this->model->GetJointController()->SetPositionPID(joint_name_gz, this->pid_pos);
      this->model->GetJointController()->SetVelocityPID(joint_name_gz, this->pid_vel);
      
      SetPosition(position);
        
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, joint_name + "_" + "node",
            ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle(joint_name + "_" + "Handle"));


      ros::SubscribeOptions pos_so = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + joint_name + "/pos_cmd",
            1,
            boost::bind(&JointPlugin::OnPosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      
      ros::SubscribeOptions vel_so = ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + joint_name + "/vel_cmd",
            1,
            boost::bind(&JointPlugin::OnVelMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);

      this->pos_rosSub = this->rosNode->subscribe(pos_so);
      this->vel_rosSub = this->rosNode->subscribe(vel_so);

      ros::AdvertiseOptions pos_ao = ros::AdvertiseOptions::create<std_msgs::Float32>(
            "/" + joint_name + "/pose",
            1,
            boost::bind(&JointPlugin::ConnectCb, this),
            boost::bind(&JointPlugin::disConnectCb, this),
            ros::VoidPtr(), &this->rosQueue);
      this->pos_rosPub = this->rosNode->advertise(pos_ao);

      this->rosQueueThread = std::thread(std::bind(&JointPlugin::QueueThread, this));

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointPlugin::OnUpdate, this));
    }
    public: void OnUpdate()
    {
      pose.data = this->joint->Position();
      pos_rosPub.publish(pose);
    }

    public: void SetPosition(const double &_pos)
    {
      this->model->GetJointController()->SetPositionTarget(
          joint_name_gz, _pos);
    }

    public: void SetVeolicity(const double &_vel)
    {
      this->model->GetJointController()->SetVelocityTarget(
          joint_name_gz, _vel);
    }

    public: void OnPosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetPosition(_msg->data);
    }

    public: void OnVelMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetVeolicity(_msg->data);
    }

    public: void ConnectCb(){}
    public: void disConnectCb(){}

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  
    private: event::ConnectionPtr updateConnection;
    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: common::PID pid_pos, pid_vel;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber pos_rosSub;
    private: ros::Subscriber vel_rosSub;
    private: ros::Publisher pos_rosPub;

    private: std_msgs::Float32 pose;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    public: std::string joint_name, joint_name_ori, joint_name_gz;
  };

  GZ_REGISTER_MODEL_PLUGIN(JointPlugin)
}
#endif