#include <ros/ros.h>

#include <mantis_msgs/Params.h>
#include <mantis_msgs/BodyInertial.h>
#include <dh_parameters/JointDescription.h>

#include <mantis_description/param_client.h>
#include <mantis_description/mixer_maps.h>

#include <eigen3/Eigen/Dense>

#include <string>

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

bool MantisParamClient::wait_for_params( void ) {
	while( ros::ok() && ( !ok() ) ) {
			ros::spinOnce();
			ros::Rate(10).sleep();
	}

	return ok();
}

void MantisParamClient::callback_params(const mantis_msgs::Params::ConstPtr &msg_in) {
	params_ = *msg_in;

	num_dynamic_joints_ = 0;
	for(int i=0; i<params_.joints.size(); i++) {
		if(params_.joints[i].type != "static")
			num_dynamic_joints_++;
	}

	if(params_.airframe_type == "quad_x4") {
		motor_num_ = mixer_generate_quad_x4(mixer_);
	} else if(params_.airframe_type == "quad_+4") {
		motor_num_ = mixer_generate_quad_p4(mixer_);
	} else if(params_.airframe_type == "hex_x6") {
		motor_num_ = mixer_generate_hex_x6(mixer_);
	} else if(params_.airframe_type == "hex_p6") {
		motor_num_ = mixer_generate_hex_p6(mixer_);
	} else if(params_.airframe_type == "octo_x8") {
		motor_num_ = mixer_generate_octo_x8(mixer_);
	} else if(params_.airframe_type == "octo_+8") {
		motor_num_= mixer_generate_octo_p8(mixer_);
	} else {
		mixer_ = Eigen::MatrixXd();
		ROS_ERROR_THROTTLE(2.0, "Unsupported mixer: %s", params_.airframe_type.c_str());
	}
}

const ros::Time& MantisParamClient::time_updated( void ) {
	return params_.header.stamp;
}

const std::string& MantisParamClient::airframe_type( void ) {
	return params_.airframe_type;
}

const int16_t& MantisParamClient::pwm_min( void ) {
	return params_.pwm_min;
}

const int16_t& MantisParamClient::pwm_max( void ) {
	return params_.pwm_max;
}

const double& MantisParamClient::base_arm_length( void ) {
	return params_.base_arm_length;
}

const int32_t& MantisParamClient::motor_num( void ) {
	return motor_num_;
}

const double& MantisParamClient::motor_kv( void ) {
	return params_.motor_kv;
}

const double& MantisParamClient::rpm_thrust_m( void ) {
	return params_.rpm_thrust_m;
}

const double& MantisParamClient::rpm_thrust_c( void ) {
	return params_.rpm_thrust_c;
}

const double& MantisParamClient::motor_drag_max( void ) {
	return params_.motor_drag_max;
}

int MantisParamClient::get_body_num( void ) {
	return params_.bodies.size();
}

double MantisParamClient::get_total_mass( void ) {
	double m = 0.0;

	for(int i=0; i<params_.bodies.size(); i++) {
		m += params_.bodies[i].mass;
	}

	return m;
}

const mantis_msgs::BodyInertial& MantisParamClient::body_inertial( const unsigned int i ) {
	return params_.bodies[i];
}

const std::string& MantisParamClient::body_name( const unsigned int i ) {
	return params_.body_names[i];
}

int MantisParamClient::get_joint_num( void ) {
	return params_.joints.size();
}

int MantisParamClient::get_dynamic_joint_num( void ) {
	return num_dynamic_joints_;
}

const dh_parameters::JointDescription& MantisParamClient::joint( const unsigned int i ) {
	return params_.joints[i];
}

const Eigen::MatrixXd& MantisParamClient::get_mixer( void ) {
	return mixer_;
}
