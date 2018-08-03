#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_joints/JointGoalsConfig.h>
#include <mantis_description/param_client.h>

#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/PathProgress.h>
#include <mantis_msgs/JointTrajectoryList.h>

#include <vector>
#include <string>

class MantisJoints {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_joints_;

		std::vector<ros::Publisher> pub_joints_;
		ros::Subscriber sub_discrete_progress_;
		ros::Subscriber sub_cubic_spline_;
		ros::Subscriber sub_joint_list_;

		std::string param_frame_id_;
		double param_update_rate_;
		ros::Time param_configuration_stamp_;
		dynamic_reconfigure::Server<mantis_joints::JointGoalsConfig> dyncfg_joint_goals_;

		MantisParamClient p_;

		std::vector<double> direct_joint_inputs_;

	public:
		MantisJoints( void );

		~MantisJoints( void );

		void callback_cfg_joint_goals(mantis_joints::JointGoalsConfig &config, uint32_t level);

		void callback_joints(const ros::TimerEvent& e);
		void callback_discrete_progress( const contrail_msgs::PathProgress::ConstPtr& msg_in );
		void callback_cubic_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in );
		void callback_joint_list( const mantis_msgs::JointTrajectoryList::ConstPtr& msg_in );
};
