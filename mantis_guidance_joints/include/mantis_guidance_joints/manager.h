#pragma once

#include <ros/ros.h>

#include <mantis_guidance_joints/joint.h>
#include <mantis_params/param_client.h>

#include <vector>

namespace MantisGuidanceJoints {

class Manager {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;

		MantisParamClient p_;

		ros::Time spawn_stamp_;
		double param_update_rate_;

		std::vector<MantisGuidanceJoints::Joint*> routers_;

	public:
		Manager( void );

		~Manager( void );

		void configure_routers( void );
		void remove_routers( void );

		void callback_timer(const ros::TimerEvent& e);
};

}
