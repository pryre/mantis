#include <ros/ros.h>

#include <mantis_controller_joints/spawner.h>
#include <mantis_controller_joints/controller.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>

Spawner::Spawner( const std::vector<std::string> &c_names ) :
	nh_(),
	nhp_("~"),
	param_update_rate_(20.0) {

	double traj_timeout = 0.2;
	nhp_.param("trajectory_reference_timeout", traj_timeout, traj_timeout);
	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	sub_state_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &Spawner::callback_state, this );

	if( c_names.size() > 0) {
		controllers_.resize( c_names.size() );

		for(int i = 0; i < c_names.size(); i++) {
			controllers_[i] = new Controller( &nhp_, c_names[i], traj_timeout);
		}

		ROS_INFO_STREAM("Spawned " << controllers_.size() << " joint controllers");
		timer_ = nhp_.createTimer(ros::Duration( 1.0 / param_update_rate_ ), &Spawner::callback_timer, this );
	} else {
		ROS_ERROR("No controllers specified");
		ros::shutdown();
	}
}

Spawner::~Spawner() {
	for(int i=0; i<controllers_.size(); i++) {
		delete controllers_[i];
	}
}

void Spawner::callback_timer(const ros::TimerEvent& e) {
	if(got_states_) {
		double dt = ( e.current_real - e.last_real ).toSec();

		//For all controllers
		for(int i = 0; i < controllers_.size(); i++) {
			for(int j = 0; j < joint_states_.name.size(); j++) {
				if(controllers_[i]->name() == joint_states_.name[j]) {
					controllers_[i]->set_state( joint_states_.position[j], joint_states_.velocity[j]);
					controllers_[i]->do_control( dt );
					break;
				}
			}
		}
	}
}

void Spawner::callback_state(const sensor_msgs::JointState::ConstPtr& msg_in) {
	joint_states_ = *msg_in;
	got_states_ = true;
}
