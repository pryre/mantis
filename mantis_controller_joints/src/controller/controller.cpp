#include <ros/ros.h>

#include <controller/controller.h>
#include <pidController/pidController.h>

#include <std_msgs/Float64.h>

#include <string>


Controller::Controller( void ) :
	have_reference_(false),
	param_limit_min_(0.0),
	param_limit_max_(0.0),
	param_gain_p_(0.0),
	param_gain_i_(0.0),
	param_gain_d_(0.0),
	output_(0.0),
	ref_(0.0) {
}

Controller::~Controller() {
}

bool Controller::init( ros::NodeHandle *nh, std::string controller_name, std::string joint_name ) {
	name_ = joint_name;

	std::string fc_name = controller_name  + "/" + joint_name;
	std::string command;

	bool success = nh->getParam(fc_name + "/command", command);

	if(success) {
		pub_output_ = nh->advertise<std_msgs::Float64>( command + "/command", 10);

		nh->param(fc_name + "/pid/p", param_gain_p_, param_gain_p_);
		nh->param(fc_name + "/pid/i", param_gain_i_, param_gain_i_);
		nh->param(fc_name + "/pid/d", param_gain_d_, param_gain_d_);
		nh->param(fc_name + "/limits/min", param_limit_min_, param_limit_min_);
		nh->param(fc_name + "/limits/max", param_limit_max_, param_limit_max_);

		pid_.setGains( param_gain_p_, param_gain_i_, param_gain_d_, 0.0);
		pid_.setOutputMinMax( param_limit_min_, param_limit_max_);

		sub_reference_ = nh->subscribe<std_msgs::Float64>( fc_name + "/command", 10, &Controller::callback_reference, this);
	}

	return success;
}

void Controller::callback_reference( const std_msgs::Float64::ConstPtr& msg_in ) {
	ref_ = msg_in->data;

	have_reference_ = true;
}

void Controller::set_state(double position, double velocity) {
	state_position_ = position;
	state_velocity_ = velocity;
}

void Controller::do_control( double dt ) {
	if( have_reference_ ) {
		output_ = pid_.step( dt, ref_, state_position_, state_velocity_ );

		std_msgs::Float64 msg_out;
		msg_out.data = output_;
		pub_output_.publish(msg_out);
	} else {
		output_ = 0.0;
		pid_.reset();
	}
}

double Controller::reference( void ) {
	return ref_;
}

double Controller::output( void ) {
	return output_;
}

std::string Controller::name( void ) {
	return name_;
}
