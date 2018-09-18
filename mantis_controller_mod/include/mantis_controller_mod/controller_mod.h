#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_params/param_client.h>
#include <mantis_state/state_client.h>
#include <mantis_kinematics/solver.h>
#include <mantis_controller_mod/ControlParamsConfig.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <string>

class ControllerMod {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_est_;
		ros::Subscriber sub_state_attitude_target_;

		ros::Publisher pub_np_force_;
		ros::Publisher pub_wrench_;

		ros::Time msg_attitude_target_tr_;

		std::string param_frame_id_;
		std::string param_model_id_;
		bool param_force_compensation_;
		double param_force_comp_alpha_;
		bool param_coriolis_compensation_;
		bool param_reference_feedback_;
		double param_est_rate_;
		dynamic_reconfigure::Server<mantis_controller_mod::ControlParamsConfig> dyncfg_control_settings_;

		MantisParamClient p_;
		MantisStateClient s_;
		MantisSolver solver_;

		double cmd_throttle_;
		Eigen::Vector3d uaug_f_;

	public:
		ControllerMod( void );

		~ControllerMod( void );

		void callback_cfg_control_settings(mantis_controller_mod::ControlParamsConfig &config, uint32_t level);

		void callback_est(const ros::TimerEvent& e);
		void callback_state_attitude_target(const mavros_msgs::AttitudeTarget::ConstPtr& msg_in);
};
