#pragma once

#include <ros/ros.h>

#include <mantis_controller_joints/controller.h>

#include <sensor_msgs/JointState.h>
#include <vector>

class Spawner {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;
		ros::Subscriber sub_state_;
		ros::Publisher pub_goal_;

		std::vector<Controller*> controllers_;

		double param_update_rate_;

		sensor_msgs::JointState joint_states_;
		bool got_states_;

	public:
		Spawner( const std::vector<std::string> &c_names );

		~Spawner( void );

		void callback_timer(const ros::TimerEvent& e);
		void callback_state(const sensor_msgs::JointState::ConstPtr& msg_in);
};
