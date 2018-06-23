#include <ros/ros.h>

#include <mantis_controller_joints/controller.h>
#include <pid_controller_lib/pidController.h>

#include <std_msgs/Float64.h>

#include <string>

Controller::Controller( ros::NodeHandle *nhp, std::string joint_name ) :
	nhp_(nhp),
	have_reference_(false),
	output_(0.0),
	ref_(0.0),
	pid_(nhp_, joint_name ) {

	name_ = joint_name;

	pub_output_ = nhp->advertise<std_msgs::Float64>( name_ + "/command", 10);
	sub_reference_ = nhp->subscribe<std_msgs::Float64>( name_ + "/reference", 10, &Controller::callback_reference, this);
}

Controller::~Controller() {
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
		output_ = pid_.step( dt, ref_, state_position_ );

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
