#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <controller_aug/controller_aug_params.h>
#include <controller_aug/controller_aug_state.h>
#include <dh_parameters/dh_parameters.h>
#include <mantis_controller_aug/ControlParamsLiteConfig.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <string>

class ControllerAugLite {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_est_;
		ros::Subscriber sub_state_attitude_target_;
		ros::Subscriber sub_state_battery_;
		ros::Subscriber sub_state_joints_;
		ros::Subscriber sub_state_imu_;

		ros::Publisher pub_force_target_;
		ros::Publisher pub_wrench_;

		ros::Time msg_attitude_target_tr_;
		ros::Time msg_battery_tr_;
		ros::Time msg_joints_tr_;
		ros::Time msg_imu_tr_;

		std::string param_frame_id_;
		std::string param_model_id_;
		bool param_wait_for_path_;
		bool param_force_compensation_;
		double param_force_comp_alpha_;
		bool param_coriolis_compensation_;
		bool param_track_end_;
		bool param_accurate_end_tracking_;
		bool param_reference_feedback_;
		double param_est_rate_;
		dynamic_reconfigure::Server<mantis_controller_aug::ControlParamsLiteConfig> dyncfg_control_settings_;

		ControllerAugParams p_;
		ControllerAugState state_;
		std::vector<DHParameters,Eigen::aligned_allocator<DHParameters> > joints_;

		double cmd_throttle_;
		Eigen::Vector3d uaug_f_;

	public:
		ControllerAugLite( void );

		~ControllerAugLite( void );

		void callback_cfg_control_settings(mantis_controller_aug::ControlParamsLiteConfig &config, uint32_t level);

		void callback_est(const ros::TimerEvent& e);
		void callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		void callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in);
		void callback_state_attitude_target(const mavros_msgs::AttitudeTarget::ConstPtr& msg_in);
};
