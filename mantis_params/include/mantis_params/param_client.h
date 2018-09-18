#pragma once

#include <ros/ros.h>
#include <mantis_msgs/Parameters.h>
#include <mantis_msgs/BodyInertial.h>
#include <dh_parameters/JointDescription.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

class MantisParamClient {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_params_;

		int num_dynamic_joints_;

		Eigen::MatrixXd mixer_;
		int32_t motor_num_;

		mantis_msgs::Parameters p_;
		/*
		Stamps identify to the current parmeter listing

		This allows for some parameters to be updated
		online without causing a race condition for
		other nodes that are configuration-dependant
		A practical application is that it allows
		inertial properties to be changed without requring
		the a new state estimate as the body configuration
		of the MM-UAV haven't actually changed

		The configuration stamp identifies the last time
		changes were made that impact the current state or
		physical configuration in some manner.
		(e.g. adding a new body or changing motor mapping)
		*/
		ros::Time configuration_stamp_;
		/*
		The parametric stamp identifies the last time
		changes were made that only modfied dynamic or
		limiting parameters.
		(e.g. thrust coefficients or inertial properties)
		*/
		ros::Time parametric_stamp_;

	public:
		MantisParamClient( const ros::NodeHandle& nh );

		~MantisParamClient( void );

		bool wait_for_params( void );

		const ros::Time& time_updated( void );
		const ros::Time& time_configuration_change( void );
		const ros::Time& time_parametric_change( void );

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

		const Eigen::MatrixXd& get_mixer( void );

		bool ok( void );

	private:
		void callback_params(const mantis_msgs::Parameters::ConstPtr &msg_in);
		const bool compare_bodies( const mantis_msgs::BodyInertial& b1, const mantis_msgs::BodyInertial& b2 );
		const bool compare_joints( const dh_parameters::JointDescription& j1, const dh_parameters::JointDescription& j2 );

};
