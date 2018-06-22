#include <ros/ros.h>

#include <spawner/spawner.h>
#include <controller/controller.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>

Spawner::Spawner( const std::vector<std::string> &c_names ) :
	nh_(),
	nhp_("~"),
	param_controller_name_("position_controller"),
	param_update_rate_(20.0) {

	nhp_.param("controller_name", param_controller_name_, param_controller_name_);
	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	sub_state_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &Spawner::callback_state, this );

	if( c_names.size() > 0) {
		controllers_.resize( c_names.size() );

		bool success = true;

		for(int i = 0; i < c_names.size(); i++) {
			ROS_INFO("Loading: %s/%s", param_controller_name_.c_str(), c_names[i].c_str());
			success &= controllers_[i].init( &nh_, param_controller_name_, c_names[i]);

			if(!success)
				break;
		}

		if(success) {
			ROS_INFO("%li controllers configured", controllers_.size());

			timer_ = nh_.createTimer(ros::Duration( 1.0 / param_update_rate_ ), &Spawner::callback_timer, this );
		} else {
			ROS_ERROR("Failed to load a controller");
			ros::shutdown();
		}
	} else {
		ROS_ERROR("No controllers specified");
		ros::shutdown();
	}
}

Spawner::~Spawner() {
}

void Spawner::callback_timer(const ros::TimerEvent& e) {
	if(got_states_) {
		double dt = ( e.current_real - e.last_real ).toSec();

		//For all controllers
		for(int i = 0; i < controllers_.size(); i++) {
			for(int j = 0; j < joint_states_.name.size(); j++) {
				if(controllers_[i].name() == joint_states_.name[j]) {
					controllers_[i].set_state( joint_states_.position[j], joint_states_.velocity[j]);
					controllers_[i].do_control( dt );
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
