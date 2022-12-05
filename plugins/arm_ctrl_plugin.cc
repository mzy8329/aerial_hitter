#ifndef _GAZEBO_ARM_CONTROLLER_HH_
#define _GAZEBO_ARM_CONTROLLER_HH_


#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>


#define ARM_JOINGT_1 9
#define ARM_JOINGT_2 10


namespace gazebo
{
	class ArmCtrlPlugin : public ModelPlugin
	{
	public:
		ArmCtrlPlugin() {}
		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			std::bind(&ArmCtrlPlugin::OnUpdate, this));

			this->model = _model;
			this->arm_joint[0] = _model->GetJoints()[ARM_JOINGT_1];
			this->arm_joint[1] = _model->GetJoints()[ARM_JOINGT_2];

			this->arm_pid[0] = common::PID(0.1, 0, 0);
			this->arm_pid[1] = common::PID(0.1, 0, 0);

			this->model->GetJointController()->SetVelocityPID(this->arm_joint[0]->GetScopedName(), this->arm_pid[0]);
			this->model->GetJointController()->SetVelocityPID(this->arm_joint[1]->GetScopedName(), this->arm_pid[1]);


			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "arm_ctrl_plugin");
			}
			// this->rosNode.reset(new ros::NodeHandle("arm_ctrl_plugin"));

			// this->model->GetJointController()->SetJointPosition(arm_joint[0], 0);
			
		}

		void OnUpdate()
		{
			this->model->GetJointController()->SetVelocityTarget(this->arm_joint[0]->GetScopedName(), 0.0);
			this->model->GetJointController()->SetVelocityTarget(this->arm_joint[1]->GetScopedName(), 0.0);


			double pose1 = this->arm_joint[0]->Position();
			double pose2 = this->arm_joint[1]->Position();
			
		}


	private:
		event::ConnectionPtr updateConnection;

		physics::ModelPtr model;
		physics::JointPtr arm_joint[2];
		common::PID arm_pid[2];

		std::unique_ptr<ros::NodeHandle> nh;	
		ros::Subscriber ctrl_data_sub;
	};


  	GZ_REGISTER_MODEL_PLUGIN(ArmCtrlPlugin)
}
#endif