#include <ros/ros.h>

#include <controller/controller.h>
#include <pidController/pidController.h>

#include <std_msgs/Float64.h>

#include <vector>
#include <string>


Controller::Controller( void ) :
	have_setpoint_(false),
	param_ang_ff_(0.0),
	param_vel_max_(0.000001),
	param_vel_kp_(0.0),
	param_vel_kd_(0.0),
	param_vel_tau_(0.0),
	param_effort_max_(0.000001) {
}

Controller::~Controller() {
}

void Controller::init( ros::NodeHandle *nh, std::string model_name, std::string joint_name ) {
	sub_sp_ = nh->subscribe<std_msgs::Float64>( "/" + model_name + "/command/servo/angle/" + joint_name, 10, &Controller::callback_setpoint, this);
	pub_goal_ = nh->advertise<std_msgs::Float64>( "/" + model_name + "/command/servo/torque/" + joint_name, 10);


	nh->param("joint/" + joint_name + "/pos/ff", param_ang_ff_, param_ang_ff_);
	nh->param("joint/" + joint_name + "/vel/max", param_vel_max_, param_vel_max_);
	nh->param("joint/" + joint_name + "/vel/kp", param_vel_kp_, param_vel_kp_);
	nh->param("joint/" + joint_name + "/vel/ki", param_vel_ki_, param_vel_ki_);
	nh->param("joint/" + joint_name + "/vel/kd", param_vel_kd_, param_vel_kd_);
	nh->param("joint/" + joint_name + "/vel/tau", param_vel_tau_, param_vel_tau_);
	nh->param("joint/" + joint_name + "/eff/max", param_effort_max_, param_effort_max_);

	controller_velocity_.setGains( param_vel_kp_, param_vel_ki_, param_vel_kd_, param_vel_tau_ );
	controller_velocity_.setOutputMinMax( -param_effort_max_, param_effort_max_ );
}

void Controller::callback_setpoint( const std_msgs::Float64::ConstPtr& msg_in ) {
	sp_angle_ = msg_in->data;

	have_setpoint_ = true;
}

void Controller::set_states(double ref_angle, double ref_velocity, double ref_effort) {
	state_angle_ = ref_angle;
	state_velocity_ = ref_velocity;
	state_effort_ = ref_effort;
}

void Controller::do_control( double dt ) {
	if(have_setpoint_) {
		//Angle Controller
		double error_angle = sp_angle_ - state_angle_;
		sp_velocity_ = param_ang_ff_ * error_angle;

		sp_velocity_ = double_constrain(sp_velocity_, -param_vel_max_, param_vel_max_);

		//Velocity Controller
		sp_effort_ = controller_velocity_.step( dt, sp_velocity_, state_velocity_ );	//TODO: use effort state as x_dot?

	} else {
		sp_angle_ = 0.0;
		sp_velocity_ = 0.0;
		sp_effort_ = 0.0;

		controller_velocity_.reset();
	}

	msg_out_.data = sp_effort_;
	pub_goal_.publish(msg_out_);
}

double Controller::get_goal_angle( void ) {
	return sp_angle_;
}

double Controller::get_goal_velocity( void ) {
	return sp_velocity_;
}

double Controller::get_goal_effort( void ) {
	return sp_effort_;
}

double Controller::double_constrain(const double x, const double min, const double max) {
	return (x < min) ? min : (x > max) ? max : x;
}
