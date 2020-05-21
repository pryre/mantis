/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_params/params.h>
#include <mantis_params/param_client.h>
#include <mantis_msgs/State.h>
#include <mantis_state/XacroTestConfig.h>

#include <vector>

class XacroTest {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;
		ros::Publisher pub_state_;

		mantis_msgs::State msg_out_;
		double param_rate_;

		MantisParams::Client p_;
		mantis_state::XacroTestConfig cfg_;
		dynamic_reconfigure::Server<mantis_state::XacroTestConfig> dyncfg_;

	public:
		XacroTest()
			: nh_()
			, nhp_("~")
			, p_( nh_ )
			, param_rate_( 30.0 )
			, dyncfg_( ros::NodeHandle( nhp_, "xacro_test" ) ) {

			nhp_.param( "update_rate", param_rate_, param_rate_ );

			dyncfg_.setCallback( boost::bind(&XacroTest::callback_cfg, this, _1, _2 ) );

			if( p_.wait_for_params() ) {
				msg_out_.header.frame_id = "map";
				msg_out_.pose.orientation.w = 1.0;

				pub_state_ = nh_.advertise<mantis_msgs::State>("state", 10);
				timer_ = nhp_.createTimer(ros::Duration(1/param_rate_), &XacroTest::callback, this );

				ROS_INFO("Running Mantis XACRO Test Node!");
			} else {
				ros::shutdown();
			}
		}

		~XacroTest() {
		}


		void callback_cfg(mantis_state::XacroTestConfig& config, uint32_t level ) {
			cfg_ = config;
		}

		void callback(const ros::TimerEvent& e) {
			msg_out_.header.stamp = ros::Time::now();
			msg_out_.child_frame_id = p_.get(MantisParams::PARAM_MODEL_ID);
			msg_out_.configuration_stamp = p_.get(MantisParams::PARAM_TIME_CHANGE_CONFIG);


			msg_out_.mav_safety_disengaged = cfg_.flight_ready;
			msg_out_.mav_ready = cfg_.flight_ready;

			msg_out_.pose.position.x = cfg_.px;
			msg_out_.pose.position.y = cfg_.py;
			msg_out_.pose.position.z = cfg_.pz;

			int n = p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC);
			int i = 0;
			msg_out_.r = std::vector<double>(n);	//Zero init
			msg_out_.rd = std::vector<double>(n);	//Zero init
			while( (i < n) && (i < 8) ) {
				switch(i) {
					case 0:
						msg_out_.r[i] = cfg_.r1;
						break;
					case 1:
						msg_out_.r[i] = cfg_.r2;
						break;
					case 2:
						msg_out_.r[i] = cfg_.r3;
						break;
					case 3:
						msg_out_.r[i] = cfg_.r4;
						break;
					case 4:
						msg_out_.r[i] = cfg_.r5;
						break;
					case 5:
						msg_out_.r[i] = cfg_.r6;
						break;
					case 6:
						msg_out_.r[i] = cfg_.r7;
						break;
					case 7:
						msg_out_.r[i] = cfg_.r8;
						break;
				}

				i++;
			}

			pub_state_.publish(msg_out_);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xacro_test_node");

	XacroTest xt;

	ros::spin();

	return 0;
}
