#include <ros/ros.h>

#include <mantis_msgs/Params.h>
#include <mantis_msgs/BodyInertial.h>
#include <dh_parameters/JointDescription.h>

#include <mantis_description/param_client.h>

MantisParamClient::MantisParamClient(ros::NodeHandle *nh) :
	nh_(nh),
	num_dynamic_joints_(0) {

	sub_params_ = nh_->subscribe<mantis_msgs::Params>( "params", 1, &MantisParamClient::callback_params, this );
}

MantisParamClient::~MantisParamClient() {
}

bool MantisParamClient::ok( void ) {
	return ( time_updated() != ros::Time(0) );
}

void MantisParamClient::callback_params(const mantis_msgs::Params::ConstPtr &msg_in) {
	params_ = *msg_in;

	num_dynamic_joints_ = 0;
	for(int i=0; i<params_.joints.size(); i++) {
		if(params_.joints[i].type != "static")
			num_dynamic_joints_++;
	}
}

ros::Time MantisParamClient::time_updated( void ) {
	return params_.header.stamp;
}

int MantisParamClient::pwm_min( void ) {
	return params_.pwm_min;
}

int MantisParamClient::pwm_max( void ) {
	return params_.pwm_max;
}

double MantisParamClient::base_arm_length( void ) {
	return params_.base_arm_length;
}

double MantisParamClient::motor_num( void ) {
	return params_.motor_num;
}

double MantisParamClient::motor_kv( void ) {
	return params_.motor_kv;
}

double MantisParamClient::rpm_thrust_m( void ) {
	return params_.rpm_thrust_m;
}

double MantisParamClient::rpm_thrust_c( void ) {
	return params_.rpm_thrust_c;
}

double MantisParamClient::motor_drag_max( void ) {
	return params_.motor_drag_max;
}

int MantisParamClient::get_body_num( void ) {
	return params_.bodies.size();
}

mantis_msgs::BodyInertial MantisParamClient::body_inertial( const unsigned int i ) {
	mantis_msgs::BodyInertial i_out;

	if(i < params_.bodies.size() ) {
		i_out = params_.bodies[i];
	}

	return i_out;
}

int MantisParamClient::get_joint_num( void ) {
	return params_.joints.size();
}

int MantisParamClient::get_dynamic_joint_num( void ) {
	return num_dynamic_joints_;
}

dh_parameters::JointDescription MantisParamClient::joint( const unsigned int i ) {
	dh_parameters::JointDescription j_out;

	if(i < params_.joints.size() ) {
		j_out = params_.joints[i];
	}

	return j_out;
}
