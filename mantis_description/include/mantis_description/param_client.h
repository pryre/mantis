#pragma once

#include <ros/ros.h>
#include <mantis_msgs/Params.h>
#include <mantis_msgs/BodyInertial.h>
#include <dh_parameters/JointDescription.h>

#include <string>
#include <vector>

class MantisParamClient {
	private:
		ros::NodeHandle *nh_;
		ros::Subscriber sub_params_;
		mantis_msgs::Params params_;

		int num_dynamic_joints_;

	public:
		MantisParamClient( ros::NodeHandle *nh );

		~MantisParamClient( void );

		ros::Time time_updated( void );

		int pwm_min( void );
		int pwm_max( void );

		double base_arm_length( void );
		double motor_num( void );
		double motor_kv( void );
		double rpm_thrust_m( void );
		double rpm_thrust_c( void );
		double motor_drag_max( void );

		int get_body_num( void );
		mantis_msgs::BodyInertial body_inertial( const unsigned int i );

		int get_joint_num( void );
		int get_dynamic_joint_num( void );
		dh_parameters::JointDescription joint(const unsigned int i);

		bool ok( void );

	private:
		void callback_params(const mantis_msgs::Params::ConstPtr &msg_in);
};
