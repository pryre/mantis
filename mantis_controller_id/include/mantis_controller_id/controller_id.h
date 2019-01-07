#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <contrail/ContrailManager.h>
#include <pid_controller_lib/pidController.h>
#include <mantis_params/param_client.h>
#include <mantis_state/state_client.h>
#include <mantis_kinematics/solver.h>

#include <mantis_controller_id/ControlParamsConfig.h>
#include <mantis_controller_id/AngleGainsConfig.h>
#include <mantis_controller_id/controller_id.h>

#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <eigen3/Eigen/Dense>

#include <string>

class ControllerID {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_low_level_;

		ros::Publisher pub_actuators_;
		ros::Publisher pub_att_target_;
		//ros::Publisher pub_twist_;
		ros::Publisher pub_accel_;

		std::string param_frame_id_;
		std::string param_model_id_;
		bool param_reference_feedback_;
		ros::Duration param_setpoint_timeout_;
		double param_low_level_rate_;
		dynamic_reconfigure::Server<mantis_controller_id::ControlParamsConfig> dyncfg_control_settings_;
		dynamic_reconfigure::Server<mantis_controller_id::AngleGainsConfig> dyncfg_angle_gains_;

		bool was_flight_ready_;
		bool abort_flight_;

		MantisParamClient p_;
		MantisStateClient s_;
		MantisSolver solver_;

		Eigen::Matrix3d R_sp_;
		double yr_sp_;
		Eigen::Vector3d a_sp_;
		ros::Time tc_sp_;

		double param_ang_gain_;
		double param_ang_yaw_w_;
		pidController rate_pid_x_;
		pidController rate_pid_y_;
		pidController rate_pid_z_;

		message_filters::Subscriber<geometry_msgs::AccelStamped> sub_accel_;
		message_filters::Subscriber<mavros_msgs::AttitudeTarget> sub_attitude_target_;
		message_filters::TimeSynchronizer<geometry_msgs::AccelStamped, mavros_msgs::AttitudeTarget> sub_sp_sync_;

	public:
		ControllerID( void );

		~ControllerID( void );

	private:
		void callback_cfg_control_settings(mantis_controller_id::ControlParamsConfig &config, uint32_t level);
		void callback_cfg_angle_gains(mantis_controller_id::AngleGainsConfig &config, uint32_t level);

		Eigen::Vector3d calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R, const double yaw_w);

		void message_output_control(const ros::Time t, const Eigen::VectorXd &u);

		void callback_low_level(const ros::TimerEvent& e);
		void callback_setpoints(const geometry_msgs::AccelStampedConstPtr& accel, const mavros_msgs::AttitudeTargetConstPtr& attitude);
};
