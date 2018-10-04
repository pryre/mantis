#pragma once

#include <ros/ros.h>

#include <pid_controller_lib/pidController.h>

#include <std_msgs/Float64.h>
#include <mantis_msgs/JointTrajectoryGoal.h>

#include <string>

namespace MantisControllerJoints {

class Controller {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_output_;
		ros::Subscriber sub_reference_pos_;
		ros::Subscriber sub_reference_traj_;

		pidController pid_;

		std::string name_;

		double state_position_;
		double state_velocity_;

		bool have_reference_;

		double output_;
		double ref_pos_;
		double ref_vel_;
		ros::Time ref_update_t_;
		ros::Duration ref_timeout_;

	public:
		Controller( const ros::NodeHandle& nh, std::string joint_name, double traj_timeout );

		~Controller( void );

		void callback_reference_pos( const std_msgs::Float64::ConstPtr& msg_in );
		void callback_reference_traj( const mantis_msgs::JointTrajectoryGoal::ConstPtr& msg_in );

		void set_state(double position, double velocity );

		void do_control( double dt );

		double output( void );
		double ref_pos( void );
		double ref_vel( void );
		std::string name( void );
};

}
