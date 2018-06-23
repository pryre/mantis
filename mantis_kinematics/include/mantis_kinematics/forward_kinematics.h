#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <mantis_description/state_client.h>
#include <mantis_description/param_client.h>
#include <mantis_description/solver.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

class ForwardKinematics {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Publisher pub_end_;
		ros::Publisher pub_viz_;
		ros::Timer timer_;

		tf2_ros::Buffer tfBuffer_;
		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfsbr_;

		std::string param_frame_id_;
		std::string param_model_id_;
		double param_rate_;
		bool param_do_end_effector_pose_;
		bool param_do_viz_;
		bool param_done_viz_;

		MantisParamClient p_;
		MantisStateClient s_;
		MantisSolver solver_;

		ros::Time time_last_update_;

	public:
		ForwardKinematics( void );

		~ForwardKinematics( void );

		void do_reset();
		void check_update_time();

		void configure_static_joints();

		void callback_timer(const ros::TimerEvent& e);
		void do_viz( const std::vector<std::string> *arm_names );
};
