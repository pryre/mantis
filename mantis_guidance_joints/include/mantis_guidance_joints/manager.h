/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

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

		MantisParams::Client p_;

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
