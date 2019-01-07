#include <ros/ros.h>

#include <dh_parameters/JointDescription.h>
#include <mantis_msgs/BodyInertial.h>
#include <mantis_msgs/Parameters.h>

#include <mantis_description/mixer_maps.h>
#include <mantis_params/param_client.h>

#include <eigen3/Eigen/Dense>

#include <string>

MantisParamClient::MantisParamClient( const ros::NodeHandle& nh )
	: nh_( nh )
	, num_dynamic_joints_( 0 ) {

	sub_params_ = nh_.subscribe<mantis_msgs::Parameters>(
		"params", 1, &MantisParamClient::callback_params, this );
}

MantisParamClient::~MantisParamClient() {
}

bool MantisParamClient::ok( void ) {
	return ( time_updated() != ros::Time( 0 ) );
}

bool MantisParamClient::wait_for_params( void ) {
	while ( ros::ok() && ( !ok() ) ) {
		ros::spinOnce();
		ros::Rate( 10 ).sleep();
	}

	return ok();
}

void MantisParamClient::callback_params(
	const mantis_msgs::Parameters::ConstPtr& msg_in ) {
	// At each step, configuration change is checked to see if we can skip ahead.
	// If there is a configuration change, then a parameter change is set as well
	// later
	bool configuration_change = false;
	bool parametric_change = false;

	// Check for simple configuration changes
	if ( ( msg_in->airframe_type != p_.airframe_type ) || ( msg_in->bodies.size() != p_.bodies.size() ) || ( msg_in->joints.size() != p_.joints.size() ) ) {

		configuration_change = true;
	}

	// Check for simple parametric changes
	if ( !configuration_change ) {
		if ( ( msg_in->pwm_min != p_.pwm_min ) || ( msg_in->pwm_max != p_.pwm_max ) || ( msg_in->base_arm_length != p_.base_arm_length ) ||
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
	if ( configuration_change || parametric_change )
		p_ = *msg_in;

	// Handle generated variables if changed
	if ( configuration_change ) {
		num_dynamic_joints_ = 0;
		for ( int i = 0; i < p_.joints.size(); i++ ) {
			if ( p_.joints[i].type != "static" )
				num_dynamic_joints_++;
		}

		if ( p_.airframe_type == "quad_x4" ) {
			motor_num_ = MDTools::mixer_generate_quad_x4( mixer_ );
		} else if ( p_.airframe_type == "quad_+4" ) {
			motor_num_ = MDTools::mixer_generate_quad_p4( mixer_ );
		} else if ( p_.airframe_type == "hex_x6" ) {
			motor_num_ = MDTools::mixer_generate_hex_x6( mixer_ );
		} else if ( p_.airframe_type == "hex_p6" ) {
			motor_num_ = MDTools::mixer_generate_hex_p6( mixer_ );
		} else if ( p_.airframe_type == "octo_x8" ) {
			motor_num_ = MDTools::mixer_generate_octo_x8( mixer_ );
		} else if ( p_.airframe_type == "octo_+8" ) {
			motor_num_ = MDTools::mixer_generate_octo_p8( mixer_ );
		} else {
			mixer_ = Eigen::MatrixXd();
			ROS_ERROR_THROTTLE( 2.0, "Unsupported mixer: %s",
				p_.airframe_type.c_str() );
		}
	}
}

const ros::Time& MantisParamClient::time_updated( void ) {
	return p_.header.stamp;
}

const ros::Time& MantisParamClient::time_configuration_change( void ) {
	return configuration_stamp_;
}

const ros::Time& MantisParamClient::time_parametric_change( void ) {
	return parametric_stamp_;
}

const std::string& MantisParamClient::airframe_type( void ) {
	return p_.airframe_type;
}

const int16_t& MantisParamClient::pwm_min( void ) {
	return p_.pwm_min;
}

const int16_t& MantisParamClient::pwm_max( void ) {
	return p_.pwm_max;
}

const double& MantisParamClient::base_arm_length( void ) {
	return p_.base_arm_length;
}

const int32_t& MantisParamClient::motor_num( void ) {
	return motor_num_;
}
/*
const double& MantisParamClient::motor_kv( void ) {
        return p_.motor_kv;
}

const double& MantisParamClient::rpm_thrust_m( void ) {
        return p_.rpm_thrust_m;
}

const double& MantisParamClient::rpm_thrust_c( void ) {
        return p_.rpm_thrust_c;
}
*/

const double& MantisParamClient::motor_thrust_max( void ) {
	return p_.motor_thrust_max;
}

const double& MantisParamClient::motor_drag_max( void ) {
	return p_.motor_drag_max;
}

int MantisParamClient::get_body_num( void ) {
	return p_.bodies.size();
}

double MantisParamClient::get_total_mass( void ) {
	double m = 0.0;

	for ( int i = 0; i < p_.bodies.size(); i++ ) {
		m += p_.bodies[i].mass;
	}

	return m;
}

const mantis_msgs::BodyInertial&
MantisParamClient::body_inertial( const unsigned int i ) {
	return p_.bodies[i];
}

int MantisParamClient::get_joint_num( void ) {
	return p_.joints.size();
}

int MantisParamClient::get_dynamic_joint_num( void ) {
	return num_dynamic_joints_;
}

const dh_parameters::JointDescription&
MantisParamClient::joint( const unsigned int i ) {
	return p_.joints[i];
}

const Eigen::MatrixXd& MantisParamClient::get_mixer( void ) {
	return mixer_;
}

const bool
MantisParamClient::compare_bodies( const mantis_msgs::BodyInertial& b1,
	const mantis_msgs::BodyInertial& b2 ) {
	return ( ( b1.name == b2.name ) && ( b1.com == b2.com ) && ( b1.mass == b2.mass ) && ( b1.Ixx == b2.Ixx ) && ( b1.Ixy == b2.Ixy ) && ( b1.Ixz == b2.Ixz ) && ( b1.Iyy == b2.Iyy ) && ( b1.Iyz == b2.Iyz ) && ( b1.Izz == b2.Izz ) );
}

const bool
MantisParamClient::compare_joints( const dh_parameters::JointDescription& j1,
	const dh_parameters::JointDescription& j2 ) {
	return ( ( j1.name == j2.name ) && ( j1.type == j2.type ) && ( j1.d == j2.d ) && ( j1.t == j2.t ) && ( j1.r == j2.r ) && ( j1.a == j2.a ) && ( j1.q == j2.q ) && ( j1.beta == j2.beta ) );
}
