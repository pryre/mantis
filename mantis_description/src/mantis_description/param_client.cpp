#include <ros/ros.h>

#include <mantis_msgs/Parameters.h>
#include <mantis_msgs/BodyInertial.h>
#include <dh_parameters/JointDescription.h>

#include <mantis_description/param_client.h>
#include <mantis_description/mixer_maps.h>

#include <eigen3/Eigen/Dense>

#include <string>

MantisParamClient::MantisParamClient(const ros::NodeHandle& nh) :
	nh_(nh),
	num_dynamic_joints_(0) {

	sub_params_ = nh_.subscribe<mantis_msgs::Parameters>( "params", 1, &MantisParamClient::callback_params, this );
}

MantisParamClient::~MantisParamClient() {
}

bool MantisParamClient::ok( void ) {
	return ( time_updated() != ros::Time(0) );
}

bool MantisParamClient::wait_for_params( void ) {
	while( ros::ok() && ( !ok() ) ) {
			ros::spinOnce();
			ros::Rate(10).sleep();
	}

	return ok();
}

void MantisParamClient::callback_params(const mantis_msgs::Parameters::ConstPtr &msg_in) {
	bool configuration_change = false;
	bool parametric_change = false;

	//XXX: DO THIS PROPERLY
	configuration_change = true;
	parametric_change = true;
	//XXX: DO THIS PROPERLY

	if(configuration_change)
		configuration_stamp_ = msg_in->header.stamp;

	if(parametric_change)
		parametric_stamp_ = msg_in->header.stamp;

	//Update the data all in one go if there was a change, just because lazy
	if(configuration_change || parametric_change)
		p_ = *msg_in;

	//Handle generated variables if changed
	if(configuration_change) {
		num_dynamic_joints_ = 0;
		for(int i=0; i<p_.joints.size(); i++) {
			if(p_.joints[i].type != "static")
				num_dynamic_joints_++;
		}

		if(p_.airframe_type == "quad_x4") {
			motor_num_ = MDTools::mixer_generate_quad_x4(mixer_);
		} else if(p_.airframe_type == "quad_+4") {
			motor_num_ = MDTools::mixer_generate_quad_p4(mixer_);
		} else if(p_.airframe_type == "hex_x6") {
			motor_num_ = MDTools::mixer_generate_hex_x6(mixer_);
		} else if(p_.airframe_type == "hex_p6") {
			motor_num_ = MDTools::mixer_generate_hex_p6(mixer_);
		} else if(p_.airframe_type == "octo_x8") {
			motor_num_ = MDTools::mixer_generate_octo_x8(mixer_);
		} else if(p_.airframe_type == "octo_+8") {
			motor_num_= MDTools::mixer_generate_octo_p8(mixer_);
		} else {
			mixer_ = Eigen::MatrixXd();
			ROS_ERROR_THROTTLE(2.0, "Unsupported mixer: %s", p_.airframe_type.c_str());
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

const double& MantisParamClient::motor_kv( void ) {
	return p_.motor_kv;
}

const double& MantisParamClient::rpm_thrust_m( void ) {
	return p_.rpm_thrust_m;
}

const double& MantisParamClient::rpm_thrust_c( void ) {
	return p_.rpm_thrust_c;
}

const double& MantisParamClient::motor_drag_max( void ) {
	return p_.motor_drag_max;
}

int MantisParamClient::get_body_num( void ) {
	return p_.bodies.size();
}

double MantisParamClient::get_total_mass( void ) {
	double m = 0.0;

	for(int i=0; i<p_.bodies.size(); i++) {
		m += p_.bodies[i].mass;
	}

	return m;
}

const mantis_msgs::BodyInertial& MantisParamClient::body_inertial( const unsigned int i ) {
	return p_.bodies[i];
}

const std::string& MantisParamClient::body_name( const unsigned int i ) {
	return p_.body_names[i];
}

int MantisParamClient::get_joint_num( void ) {
	return p_.joints.size();
}

int MantisParamClient::get_dynamic_joint_num( void ) {
	return num_dynamic_joints_;
}

const dh_parameters::JointDescription& MantisParamClient::joint( const unsigned int i ) {
	return p_.joints[i];
}

const Eigen::MatrixXd& MantisParamClient::get_mixer( void ) {
	return mixer_;
}
