#pragma once

#include <ros/ros.h>

#include <mantis_params/param_client.h>
#include <mantis_state/state_client.h>

#include <mantis_controller_joints/controller.h>

#include <vector>

namespace MantisControllerJoints {

class Spawner {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;

		MantisParamClient p_;
		MantisStateClient s_;

		ros::Time spawn_stamp_;
		double traj_timeout_;
		double param_update_rate_;

		std::vector<MantisControllerJoints::Controller*> controllers_;

	public:
		Spawner( void );

		~Spawner( void );

		void configure_controllers( void );
		void remove_controllers( void );

		void callback_timer(const ros::TimerEvent& e);
};

}
