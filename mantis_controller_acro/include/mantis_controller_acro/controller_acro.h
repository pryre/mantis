#pragma once

#include <ros/ros.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Imu.h>

#include <mantis_params/param_client.h>
#include <pid_controller_lib/pidController.h>

#include <eigen3/Eigen/Dense>

typedef struct controlCommand_s {
	double T;
	double r;
	double p;
	double y;
} controlCommand_t;

class ControllerAcro {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		MantisParams::Client p_;

		ros::Subscriber sub_attitude_target_;
		ros::Subscriber sub_actuator_control_;
		ros::Subscriber sub_imu_;
		ros::Publisher pub_rc_out_;
		ros::Publisher pub_attitude_target_;

		ros::Timer tmr_rc_out_;
		double pwm_update_rate_;
		std::string frame_id_;

		double force_comp_timeout_;

		std::string model_name_;
		sensor_msgs::Imu model_imu_;
		mavros_msgs::AttitudeTarget goal_att_;
		mavros_msgs::ActuatorControl force_compensation_;

		pidController controller_ang_x_;
		pidController controller_ang_y_;
		pidController controller_ang_z_;
		pidController controller_rates_x_;
		pidController controller_rates_y_;
		pidController controller_rates_z_;

		controlCommand_t control_angle_;
		controlCommand_t control_rates_;
		controlCommand_t control_forces_;

	public:
		ControllerAcro( void );
		~ControllerAcro( void );

	private:
		int32_t int32_constrain(const int32_t i, const int32_t min, const int32_t max);
		double double_constrain(const double i, const double min, const double max);

		Eigen::Vector3d calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R);

		void callback_mixer(const ros::TimerEvent& event);
		void callback_imu(const sensor_msgs::Imu::ConstPtr& msg_in);
		void callback_force_comp(const mavros_msgs::ActuatorControl::ConstPtr& msg_in);
		void callback_attitude( const mavros_msgs::AttitudeTarget::ConstPtr& msg_in );
};
