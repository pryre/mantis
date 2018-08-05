#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_joints/JointGoalsConfig.h>
#include <mantis_description/param_client.h>

#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/PathProgress.h>
#include <mantis_msgs/JointTrajectoryList.h>

#include <mantis_joints/tinysplinecpp.h>

#include <vector>
#include <string>

class MantisJoints {
	public:
		enum LastJointReference {
			Undefined,
			DirectInput,
			DiscreteProgress,
			CubicSpline
		};

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

		LastJointReference tracked_input_;
		unsigned int current_discrete_point_;

		ros::Time spline_start_;
		ros::Duration spline_duration_;
		ros::Time spline_joint_stamp_;
		ros::Time spline_configuration_stamp_;
		std::vector<tinyspline::BSpline> spline_list_;

		MantisParamClient p_;
		mantis_msgs::JointTrajectoryList joint_list_;

	public:
		MantisJoints( void );

		~MantisJoints( void );

		void callback_joints(const ros::TimerEvent& e);

		void configure_publishers( void );
		bool get_direct_input_reference( double &pos_ref, double &vel_ref, const unsigned int index );
		bool get_discrete_progress_reference( double &pos_ref, double &vel_ref, const unsigned int index );
		bool get_cubic_spline_reference( double &pos_ref, double &vel_ref, const unsigned int index, const ros::Time& tc );

		void callback_cfg_joint_goals(mantis_joints::JointGoalsConfig &config, uint32_t level);
		void callback_discrete_progress( const contrail_msgs::PathProgress::ConstPtr& msg_in );
		void callback_cubic_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in );
		void callback_joint_list( const mantis_msgs::JointTrajectoryList::ConstPtr& msg_in );

		void generate_splines( void );
		bool check_msg_spline(const contrail_msgs::CubicSpline& spline, const ros::Time t );

		inline double normalize(double x, const double min, const double max) const {
			return (x - min) / (max - min);
		}
};
