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

		bool wait_for_params( void );

		const ros::Time& time_updated( void );

		const std::string& airframe_type( void );
		const int16_t& pwm_min( void );
		const int16_t& pwm_max( void );

		const double& base_arm_length( void );
		const int32_t& motor_num( void );
		const double& motor_kv( void );
		const double& rpm_thrust_m( void );
		const double& rpm_thrust_c( void );
		const double& motor_drag_max( void );

		int get_body_num( void );
		double get_total_mass( void );
		const mantis_msgs::BodyInertial& body_inertial( const unsigned int i );

		int get_joint_num( void );
		int get_dynamic_joint_num( void );
		const dh_parameters::JointDescription& joint(const unsigned int i);

		bool ok( void );

	private:
		void callback_params(const mantis_msgs::Params::ConstPtr &msg_in);
};
