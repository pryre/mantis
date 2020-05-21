/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <ros/ros.h>

#include <dh_parameters/JointDescription.h>
#include <mantis_msgs/BodyInertial.h>
#include <mantis_msgs/Parameters.h>

#include <mantis_description/mixer_maps.h>
#include <mantis_params/param_client.h>

#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Dense>

#include <string>

namespace MantisParams {

Client::Client( const ros::NodeHandle& nh )
	: nh_( nh )
	, num_motors_( 0 )
	, num_dynamic_joints_( 0 )
	, total_mass_(0.0)
	, airframe_type_(PARAM_AIRFRAME_TYPE_UNSUPPORTED) {

	sub_params_ = nh_.subscribe<mantis_msgs::Parameters>(
		"params", 1, &Client::callback_params, this );
	srv_reload_ = nh_.serviceClient<std_srvs::Empty>("reload_params");
}

Client::~Client() {
}

bool Client::ok( void ) {
	return ( get(PARAM_TIME_UPDATED) != ros::Time( 0 ) );
}

bool Client::wait_for_params( void ) {
	while ( ros::ok() && ( !ok() ) ) {
		ros::spinOnce();
		ros::Rate( 10 ).sleep();
	}

	return ok();
}

bool Client::reload_params( void ) {
	std_srvs::Empty srv;
	return srv_reload_.call(srv);
}

void Client::callback_params(
	const mantis_msgs::Parameters::ConstPtr& msg_in ) {
	// At each step, configuration change is checked to see if we can skip ahead.
	// If there is a configuration change, then a parameter change is set as well
	// later
	bool configuration_change = false;
	bool parametric_change = false;

	// Check for simple configuration changes
	if ( ( msg_in->airframe_name != p_.airframe_name ) ||
		 ( msg_in->bodies.size() != p_.bodies.size() ) ||
		 ( msg_in->joints.size() != p_.joints.size() ) ||
		 ( msg_in->base_arm_length != p_.base_arm_length ) ||
		 ( msg_in->base_motor_cant != p_.base_motor_cant ) ) {

		configuration_change = true;
	}

	// Check for simple parametric changes
	if ( !configuration_change ) {
		if ( ( msg_in->pwm_min != p_.pwm_min ) || ( msg_in->pwm_max != p_.pwm_max ) ||
			//(msg_in->motor_kv != p_.motor_kv) ||
			//(msg_in->rpm_thrust_m != p_.rpm_thrust_m) ||
			//(msg_in->rpm_thrust_c != p_.rpm_thrust_c) ||
			( msg_in->motor_thrust_max != p_.motor_thrust_max ) || ( msg_in->motor_drag_max != p_.motor_drag_max ) ) {

			parametric_change = true;
		}
	}

	// If nothing obvious, do a deep scan of each of the joints and bodies for
	// changes
	// It is already ensured that the vector sizes are equal
	if ( !configuration_change ) {
		for ( int i = 0; i < msg_in->bodies.size(); i++ ) {
			if ( msg_in->bodies[i].name != p_.bodies[i].name ) {
				configuration_change = true;
				break;
			}

			// Names already must be the same, but some of the other params may have
			// changed
			if ( !compare_bodies( msg_in->bodies[i], p_.bodies[i] ) )
				parametric_change = true;
		}
	}

	if ( !configuration_change ) {
		for ( int i = 0; i < msg_in->joints.size(); i++ ) {
			if ( ( msg_in->joints[i].name != p_.joints[i].name ) || ( msg_in->joints[i].type != p_.joints[i].type ) ) {
				configuration_change = true;
				break;
			}

			// Names already must be the same, but some of the other params may have
			// changed
			if ( !compare_joints( msg_in->joints[i], p_.joints[i] ) )
				parametric_change = true;
		}
	}

	if ( configuration_change ) {
		configuration_stamp_ = msg_in->header.stamp;
		parametric_change = true; // Can't hurt to assume that paramters have most
		// likely changed here as well
	}

	if ( parametric_change )
		parametric_stamp_ = msg_in->header.stamp;

	// Update the data all in one go if there was a change, just because lazy
	if ( configuration_change || parametric_change ) {
		p_ = *msg_in;

		//Update our total mass as it has probably changed in either case
		total_mass_ = 0.0;
		for ( int i = 0; i < p_.bodies.size(); i++ ) {
			total_mass_ += p_.bodies[i].mass;
		}
	}

	// Handle generated variables if changed
	if ( configuration_change ) {
		num_bodies_ = p_.bodies.size();
		num_joints_ = p_.joints.size();

		num_dynamic_joints_ = 0;
		for ( int i = 0; i < p_.joints.size(); i++ ) {
			if ( p_.joints[i].type != "static" )
				num_dynamic_joints_++;
		}

		if ( p_.airframe_name == "quad_x4" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_QUAD_X4;
			num_motors_ = MDTools::mixer_generate_quad_x4( mixer_ );
		} else if ( p_.airframe_name == "quad_p4" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_QUAD_P4;
			num_motors_ = MDTools::mixer_generate_quad_p4( mixer_ );
		} else if ( p_.airframe_name == "hex_x6" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_HEX_X6;
			num_motors_ = MDTools::mixer_generate_hex_x6( mixer_ );
		} else if ( p_.airframe_name == "hex_p6" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_HEX_P6;
			num_motors_ = MDTools::mixer_generate_hex_p6( mixer_ );
		} else if ( p_.airframe_name == "hex_fa" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_HEX_FA;
			num_motors_ = MDTools::mixer_generate_hex_fa( mixer_, p_.base_arm_length, p_.base_motor_cant );
		} else if ( p_.airframe_name == "octo_x8" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_OCTO_X8;
			num_motors_ = MDTools::mixer_generate_octo_x8( mixer_ );
		} else if ( p_.airframe_name == "octo_p8" ) {
			airframe_type_ = PARAM_AIRFRAME_TYPE_OCTO_P8;
			num_motors_ = MDTools::mixer_generate_octo_p8( mixer_ );
		} else {
			mixer_ = Eigen::MatrixXd();
			airframe_type_ = PARAM_AIRFRAME_TYPE_UNSUPPORTED;
			ROS_ERROR( "Unsupported mixer: %s", p_.airframe_name.c_str() );
		}
	}
}

//==-- Parameter get calls
const ros::Time& Client::get(const ParamsTime param_id) {
	switch(param_id) {
		case PARAM_TIME_UPDATED: {
			return p_.header.stamp;
		}
		case PARAM_TIME_CHANGE_CONFIG: {
			return configuration_stamp_;
		}
		case PARAM_TIME_CHANGE_PARAMETRIC: {
			return parametric_stamp_;
		}
		default: {
			warn_get_failed(ParamsTimeName[param_id]);
			throw std::bad_typeid();
		}
	}
}

const uint16_t& Client::get(const ParamsUint16 param_id) {
	switch(param_id) {
		case PARAM_PWM_MIN: {
			return p_.pwm_min;
		}
		case PARAM_PWM_MAX: {
			return p_.pwm_max;
		}
		default: {
			warn_get_failed(ParamsUint16Name[param_id]);
			throw std::bad_typeid();
		}
	}
}

const uint64_t& Client::get(const ParamsUint64 param_id) {
	switch(param_id) {
		case PARAM_MOTOR_NUM: {
			return num_motors_;
		}
		case PARAM_BODY_NUM: {
			return num_bodies_;
		}
		case PARAM_JOINT_NUM: {
			return num_joints_;
		}
		case PARAM_JOINT_NUM_DYNAMIC: {
			return num_dynamic_joints_;
		}
		default: {
			warn_get_failed(ParamsUint64Name[param_id]);
			throw std::bad_typeid();
		}
	}
}

const double& Client::get(const ParamsDouble param_id) {
	switch(param_id) {
		case PARAM_BASE_ARM_LENGTH: {
			return p_.base_arm_length;
		}
		case PARAM_BASE_MOTOR_CANT: {
			return p_.base_motor_cant;
		}
		case PARAM_MOTOR_MAX_THRUST: {
			return p_.motor_thrust_max;
		}
		case PARAM_MOTOR_MAX_DRAG: {
			return p_.motor_drag_max;
		}
		case PARAM_TOTAL_MASS: {
			return total_mass_;
		}
		default: {
			warn_get_failed(ParamsDoubleName[param_id]);
			throw std::bad_typeid();
		}
	}
}

const ParamsAirframeTypeList& Client::get(const ParamsAirframeType param_id) {
	switch(param_id) {
		case PARAM_AIRFRAME_TYPE: {
			return airframe_type_;
		}
		default: {
			warn_get_failed(ParamsAirframeTypeName[param_id]);
			throw std::bad_typeid();
		}
	}
}

const std::string& Client::get(const ParamsString param_id) {
	switch(param_id) {
		case PARAM_AIRFRAME_NAME: {
			return p_.airframe_name;
		}
		case PARAM_MODEL_ID: {
			return p_.header.frame_id;
		}
		default: {
			warn_get_failed(ParamsStringName[param_id]);
			throw std::bad_typeid();
		}
	}
}

const mantis_msgs::BodyInertial& Client::get(const ParamsBodyInertial param_id, const unsigned int body ) {
	switch(param_id) {
		case PARAM_BODY_INERTIAL: {
			return p_.bodies[body];
		}
		default: {
			warn_get_failed(ParamsBodyInertialName[param_id]);
			throw std::bad_typeid();
		}
	}
}

const dh_parameters::JointDescription& Client::get(const ParamsJointDescription param_id, const unsigned int joint ) {
	switch(param_id) {
		case PARAM_JOINT_DESCRIPTION: {
			return p_.joints[joint];
		}
		default: {
			warn_get_failed(ParamsJointDescriptionName[param_id]);
			throw std::bad_typeid();
		}
	}
}

const Eigen::MatrixXd& Client::get(const ParamsMatrixXd param_id) {
	switch(param_id) {
		case PARAM_MIXER: {
			return mixer_;
		}
		default: {
			warn_get_failed(ParamsMatrixXdName[param_id]);
			throw std::bad_typeid();
		}
	}
}

//==-- Parameter set calls
const bool Client::set(const ParamsTime param_id, const ros::Time& param) {
	bool success;

	switch(param_id) {
		//XXX: None of these parameters can be set manually
		/*
		case PARAM_TIME_UPDATED: {
			return p_.header.stamp;
		}
		case PARAM_TIME_CHANGE_CONFIG: {
			return configuration_stamp_;
		}
		case PARAM_TIME_CHANGE_PARAMETRIC: {
			return parametric_stamp_;
		}
		*/
		default: {
			warn_set_failed(ParamsTimeName[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsUint16 param_id, const uint16_t& param) {
	bool success = false;

	switch(param_id) {
		case PARAM_PWM_MIN: {
			nh_.setParam( "pwm/min", param );
			success = true;
		}
		case PARAM_PWM_MAX: {
			nh_.setParam( "pwm/max", param );
			success = true;
		}
		default: {
			warn_set_failed(ParamsUint16Name[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsUint64 param_id, const uint64_t& param) {
	bool success = false;

	switch(param_id) {
		/*
		case PARAM_MOTOR_NUM: {
			return num_motors_;
		}
		*/
		case PARAM_BODY_NUM: {
			nh_.setParam( "body/num", (int)param);
			success = true;
		}
		/*
		case PARAM_JOINT_NUM: {
			return num_joints_;
		}
		case PARAM_JOINT_NUM_DYNAMIC: {
			return num_dynamic_joints_;
		}
		*/
		default: {
			warn_set_failed(ParamsUint64Name[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsDouble param_id, const double& param) {
	bool success = false;

	switch(param_id) {
		case PARAM_BASE_ARM_LENGTH: {
			nh_.setParam( "motor/arm_len", param );
			success = true;
		}
		case PARAM_BASE_MOTOR_CANT: {
			nh_.setParam( "motor/cant", param );
			success = true;
		}
		case PARAM_MOTOR_MAX_THRUST: {
			nh_.setParam( "motor/thrust_max", param );
			success = true;
		}
		case PARAM_MOTOR_MAX_DRAG: {
			nh_.setParam( "motor/drag_max", param );
			success = true;
		}
		/*
		case PARAM_TOTAL_MASS: {
			return total_mass_;
		}
		*/
		default: {
			warn_set_failed(ParamsDoubleName[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsAirframeType param_id, const ParamsAirframeTypeList& param) {
	bool success = false;

	switch(param_id) {
		//XXX: Set using the airframe name parameter
		/*
		case PARAM_AIRFRAME_TYPE: {
			nh_.setParam( "airframe", param );
			success = true;
		}
		*/
		default: {
			warn_set_failed(ParamsStringName[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsString param_id, const std::string& param) {
	bool success = false;

	switch(param_id) {
		case PARAM_AIRFRAME_NAME: {
			nh_.setParam( "airframe_name", param );
			success = true;
		}
		default: {
			warn_set_failed(ParamsStringName[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsBodyInertial param_id, const unsigned int body, const mantis_msgs::BodyInertial& param) {
	bool success = false;

	switch(param_id) {
		case PARAM_BODY_INERTIAL: {
			std::string bp = "body/b" + std::to_string(body) + "/";

			nh_.setParam(bp + "name", param.name);
			nh_.setParam(bp + "mass/m", param.mass);
			nh_.setParam(bp + "mass/Ixx", param.Ixx);
			nh_.setParam(bp + "mass/Ixy", param.Ixy);
			nh_.setParam(bp + "mass/Ixz", param.Ixz);
			nh_.setParam(bp + "mass/Iyy", param.Iyy);
			nh_.setParam(bp + "mass/Iyz", param.Iyz);
			nh_.setParam(bp + "mass/Izz", param.Izz);
			nh_.setParam(bp + "mass/com", param.com);

			success = true;
		}
		default: {
			warn_set_failed(ParamsBodyInertialName[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsJointDescription param_id, const unsigned int joint, const dh_parameters::JointDescription& param) {
	bool success = false;

	switch(param_id) {
		case PARAM_JOINT_DESCRIPTION: {
			std::string bp = "body/b" + std::to_string(joint) + "/";

			nh_.setParam(bp + "link/type", param.type);
			nh_.setParam(bp + "link/name", param.name);
			nh_.setParam(bp + "link/d", param.d);
			nh_.setParam(bp + "link/t", param.t);
			nh_.setParam(bp + "link/r", param.r);
			nh_.setParam(bp + "link/a", param.a);
			nh_.setParam(bp + "link/q", param.q);
			nh_.setParam(bp + "link/beta", param.beta);

			success = true;
		}
		default: {
			warn_set_failed(ParamsJointDescriptionName[param_id]);
		}
	}

	return success;
}

const bool Client::set(const ParamsMatrixXd param_id, const Eigen::MatrixXd& param ) {
	bool success = false;

	switch(param_id) {
		/*
		case PARAM_MIXER: {
			return mixer_;
		}
		*/
		default: {
			warn_set_failed(ParamsMatrixXdName[param_id]);
		}
	}

	return success;
}

const bool Client::compare_bodies( const mantis_msgs::BodyInertial& b1,
	const mantis_msgs::BodyInertial& b2 ) {
	return ( ( b1.name == b2.name ) && ( b1.com == b2.com ) && ( b1.mass == b2.mass ) && ( b1.Ixx == b2.Ixx ) && ( b1.Ixy == b2.Ixy ) && ( b1.Ixz == b2.Ixz ) && ( b1.Iyy == b2.Iyy ) && ( b1.Iyz == b2.Iyz ) && ( b1.Izz == b2.Izz ) );
}

const bool Client::compare_joints( const dh_parameters::JointDescription& j1,
	const dh_parameters::JointDescription& j2 ) {
	return ( ( j1.name == j2.name ) && ( j1.type == j2.type ) && ( j1.d == j2.d ) && ( j1.t == j2.t ) && ( j1.r == j2.r ) && ( j1.a == j2.a ) && ( j1.q == j2.q ) && ( j1.beta == j2.beta ) );
}

void Client::warn_get_failed(const char* param_name) {
	ROS_WARN("Tried to get unsupported param_id: %s", param_name);
}

void Client::warn_set_failed(const char* param_name) {
	ROS_WARN("Tried to set unsupported param_id: %s", param_name);
}

};
