#include <ros/ros.h>

#include <mantis_controller_joints/controller.h>
#include <pid_controller_lib/pidController.h>

#include <std_msgs/Float64.h>
#include <mantis_msgs/JointTrajectoryGoal.h>

#include <string>

Controller::Controller( ros::NodeHandle *nhp, std::string joint_name, double traj_timeout ) :
	nhp_(nhp),
	have_reference_(false),
	output_(0.0),
	ref_pos_(0.0),
	ref_vel_(0.0),
	ref_update_t_(0),
	ref_timeout_(traj_timeout),
	pid_(nhp_, joint_name ) {

	name_ = joint_name;

	pub_output_ = nhp->advertise<std_msgs::Float64>( name_ + "/command", 10);
	sub_reference_pos_ = nhp->subscribe<std_msgs::Float64>( name_ + "/reference/pos", 10, &Controller::callback_reference_pos, this);
	sub_reference_traj_ = nhp->subscribe<mantis_msgs::JointTrajectoryGoal>( name_ + "/reference/traj", 10, &Controller::callback_reference_traj, this);
}

Controller::~Controller() {
}

void Controller::callback_reference_pos( const std_msgs::Float64::ConstPtr& msg_in ) {
	ref_pos_ = msg_in->data;
	ref_vel_ = 0.0;

	if(!have_reference_ || (ref_update_t_ != ros::Time(0)) ) {
		ROS_INFO("Joint controller %s switching to position mode", name_.c_str());
		ref_update_t_ = ros::Time(0);
	}

	ref_update_t_ = ros::Time(0);

	have_reference_ = true;
}

void Controller::callback_reference_traj( const mantis_msgs::JointTrajectoryGoal::ConstPtr& msg_in ) {
	ref_pos_ = msg_in->position;
	ref_vel_ = msg_in->velocity;
	if(!have_reference_ || (ref_update_t_ == ros::Time(0)) ) {
		ROS_INFO("Joint controller %s switching to trajectory mode", name_.c_str());
	}

	ref_update_t_ = msg_in->header.stamp;

	have_reference_ = true;
}

void Controller::set_state(double position, double velocity) {
	state_position_ = position;
	state_velocity_ = velocity;
}

void Controller::do_control( double dt ) {
	if( have_reference_ ) {
		output_ = pid_.step( dt, ref_pos_, state_position_ );

		//If the trajectory target is within our timeout,
		//  then add in the velocity constant
		if( (ros::Time::now() - ref_update_t_) < ref_timeout_ ) {
			output_ += ref_vel_;
		}

		std_msgs::Float64 msg_out;
		msg_out.data = output_;
		pub_output_.publish(msg_out);
	} else {
		output_ = 0.0;
		pid_.reset();
	}
}

double Controller::ref_pos( void ) {
	return ref_pos_;
}

double Controller::ref_vel( void ) {
	return ref_vel_;
}

double Controller::output( void ) {
	return output_;
}

std::string Controller::name( void ) {
	return name_;
}
