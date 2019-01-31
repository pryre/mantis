#pragma once

#include <ros/ros.h>
#include <mantis_params/params.h>
#include <mantis_msgs/Parameters.h>
#include <mantis_msgs/BodyInertial.h>
#include <dh_parameters/JointDescription.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

namespace MantisParams {
class Client {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_params_;
		ros::ServiceClient srv_reload_;

		uint64_t num_bodies_;
		uint64_t num_joints_;
		uint64_t num_dynamic_joints_;

		Eigen::MatrixXd mixer_;
		uint64_t num_motors_;

		double total_mass_;

		ParamsAirframeTypeList airframe_type_;

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
		Client( const ros::NodeHandle& nh);

		~Client( void );

		bool wait_for_params( void );
		bool ok( void );

		bool reload_params( void );

		const ros::Time& get(const ParamsTime param_id);
		const uint16_t& get(const ParamsUint16 param_id);
		const uint64_t& get(const ParamsUint64 param_id);
		const double& get(const ParamsDouble param_id);
		const ParamsAirframeTypeList& get(const ParamsAirframeType param_id);
		const std::string& get(const ParamsString param_id);
		const mantis_msgs::BodyInertial& get(const ParamsBodyInertial param_id, const unsigned int body );
		const dh_parameters::JointDescription& get(const ParamsJointDescription param_id, const unsigned int joint );
		const Eigen::MatrixXd& get(const ParamsMatrixXd param_id);

		const bool set(const ParamsTime param_id, const ros::Time& param);
		const bool set(const ParamsUint16 param_id, const uint16_t& param);
		const bool set(const ParamsUint64 param_id, const uint64_t& param);
		const bool set(const ParamsDouble param_id, const double& param);
		const bool set(const ParamsAirframeType param_id, const ParamsAirframeTypeList& param);
		const bool set(const ParamsString param_id, const std::string& param);
		const bool set(const ParamsBodyInertial param_id, const unsigned int body, const mantis_msgs::BodyInertial& param);
		const bool set(const ParamsJointDescription param_id, const unsigned int joint, const dh_parameters::JointDescription& param);
		const bool set(const ParamsMatrixXd param_id, const Eigen::MatrixXd& param );

	private:
		void callback_params(const mantis_msgs::Parameters::ConstPtr &msg_in);
		const bool compare_bodies( const mantis_msgs::BodyInertial& b1, const mantis_msgs::BodyInertial& b2 );
		const bool compare_joints( const dh_parameters::JointDescription& j1, const dh_parameters::JointDescription& j2 );

		void warn_get_failed(const char* param_name);
		void warn_set_failed(const char* param_name);
};
};
