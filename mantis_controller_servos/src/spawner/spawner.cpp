#include <ros/ros.h>

#include <spawner/spawner.h>
#include <controller/controller.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

Spawner::Spawner() :
	nh_("~"),
	param_model_name_("robot"),
	param_servo_update_rate_(20.0) {

	nh_.param("model_name", param_model_name_, param_model_name_);
	nh_.param("servo_update_rate", param_servo_update_rate_, param_servo_update_rate_);

	sub_state_ = nh_.subscribe<sensor_msgs::JointState>( "/" + param_model_name_ + "/state/joint_states", 10, &Spawner::callback_state, this );
	pub_goal_ = nh_.advertise<sensor_msgs::JointState>( "/" + param_model_name_ + "/goal/joint_states", 10);

	ROS_INFO("Waiting for joint states...");

	while(ros::ok() && !got_states_) {
		ros::spinOnce();
		ros::Duration( 1 / param_servo_update_rate_ ).sleep();
	}

	servo_goals_.header = servo_states_.header;
	servo_goals_.name = servo_states_.name;
	servo_goals_.position.resize( servo_goals_.name.size() );
	servo_goals_.velocity.resize( servo_goals_.name.size() );
	servo_goals_.effort.resize( servo_goals_.name.size() );

	ROS_INFO("Got joint states, spawning controllers...");

	controllers.resize( servo_states_.name.size() );

	for(int i = 0; i < servo_states_.name.size(); i++) {
		controllers[i].init( &nh_, param_model_name_, servo_states_.name[i] );
	}

	ROS_INFO("Started %i controllers!", int(servo_states_.name.size()));

	timer_ = nh_.createTimer(ros::Duration( 1 / param_servo_update_rate_ ), &Spawner::callback_timer, this );
}

Spawner::~Spawner() {
}

void Spawner::callback_timer(const ros::TimerEvent& e) {
	double dt = ( e.current_real - e.last_real ).toSec(); ;

	servo_goals_.header.stamp = e.current_real;

	//For all controllers
	for(int i = 0; i < controllers.size(); i++) {
		//Update states
		controllers[i].set_states( servo_states_.position[i], servo_states_.velocity[i], servo_states_.effort[i] );

		//Run controller
		controllers[i].do_control( dt );

		//Update goals
		servo_goals_.position[i] = controllers[i].get_goal_angle();
		servo_goals_.velocity[i] = controllers[i].get_goal_velocity();
		servo_goals_.effort[i] = controllers[i].get_goal_effort();
	}

	//Publish goals
	pub_goal_.publish(servo_goals_);
}

void Spawner::callback_state(const sensor_msgs::JointState::ConstPtr& msg_in) {
	servo_states_ = *msg_in;
	got_states_ = true;
}
