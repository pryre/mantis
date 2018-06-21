#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <controller_aug/controller_aug_params.h>
#include <controller_aug/controller_aug_state.h>
#include <contrail/path_extract.h>
#include <dh_parameters/dh_parameters.h>
#include <mantis_controller_aug/ControlParamsPathConfig.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <string>

class ControllerAugPath {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_path_;
		ros::Subscriber sub_state_joints_;
		ros::Subscriber sub_state_odom_;

		ros::Publisher pub_traj_;

		ros::Time msg_joints_tr_;
		ros::Time msg_odom_tr_;

		std::string param_frame_id_;
		std::string param_model_id_;
		bool param_track_end_;
		bool param_accurate_end_tracking_;
		double param_path_rate_;
		dynamic_reconfigure::Server<mantis_controller_aug::ControlParamsPathConfig> dyncfg_control_settings_;

		ControllerAugParams p_;
		ControllerAugState state_;
		std::vector<DHParameters,Eigen::aligned_allocator<DHParameters> > joints_;

		PathExtract ref_path_;

	public:
		ControllerAugPath( void );

		~ControllerAugPath( void );

		void callback_cfg_control_settings(mantis_controller_aug::ControlParamsPathConfig &config, uint32_t level);

		Eigen::Vector3d position_from_msg(const geometry_msgs::Point p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);

		geometry_msgs::Vector3 vector_from_eig(const Eigen::Vector3d &v);
		geometry_msgs::Point point_from_eig(const Eigen::Vector3d &p);
		geometry_msgs::Quaternion quaternion_from_eig(const Eigen::Quaterniond &q);
		geometry_msgs::Pose pose_from_eig(const Eigen::Affine3d &g);

		Eigen::Matrix3d extract_yaw_component(const Eigen::Matrix3d r);

		bool calc_goal_ge_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc);
		Eigen::Affine3d calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe);
		Eigen::Vector3d calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::MatrixXd &Je, const Eigen::VectorXd &rd);

		void callback_path(const ros::TimerEvent& e);
		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
