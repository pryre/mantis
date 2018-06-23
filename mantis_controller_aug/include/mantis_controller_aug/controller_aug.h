#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <pid_controller_lib/pidController.h>
#include <contrail/path_extract.h>

#include <mantis_description/state_client.h>
#include <mantis_description/param_client.h>
#include <mantis_description/solver.h>
#include <mantis_controller_aug/ControlParamsConfig.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <string>

class ControllerAug {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_high_level_;
		ros::Timer timer_low_level_;
		ros::Timer timer_ready_check_;

		ros::Publisher pub_rc_;
		ros::Publisher pub_accel_;
		ros::Publisher pub_twist_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_wrench_;

		std::string param_frame_id_;
		std::string param_model_id_;
		bool param_force_compensation_;
		double param_force_comp_alpha_;
		bool param_coriolis_compensation_;
		bool param_track_end_;
		bool param_accurate_end_tracking_;
		bool param_reference_feedback_;
		double param_safety_rate_;
		double param_high_level_rate_;
		double param_low_level_rate_;
		dynamic_reconfigure::Server<mantis_controller_aug::ControlParamsConfig> dyncfg_control_settings_;

		bool ready_for_flight_;

		MantisParamClient p_;
		MantisStateClient s_;
		MantisSolver solver_;

		Eigen::Affine3d g_sp_;
		Eigen::Vector3d gv_sp_;
		Eigen::Vector3d a_sp_;
		bool status_high_level_;

		PathExtract ref_path_;
		pidController pos_pid_x_;
		pidController pos_pid_y_;
		pidController pos_pid_z_;
		pidController vel_pid_x_;
		pidController vel_pid_y_;
		pidController vel_pid_z_;
		pidController ang_pid_x_;
		pidController ang_pid_y_;
		pidController ang_pid_z_;
		pidController rate_pid_x_;
		pidController rate_pid_y_;
		pidController rate_pid_z_;

		Eigen::MatrixXd mixer_;
		Eigen::Vector3d uaug_f_;

	public:
		ControllerAug( void );

		~ControllerAug( void );

		void callback_cfg_control_settings(mantis_controller_aug::ControlParamsConfig &config, uint32_t level);

		//double double_clamp(const double v, const double min, const double max);
		//void matrix_clamp(Eigen::MatrixXd m, const double min, const double max);
		//Eigen::VectorXd vector_interlace(const Eigen::VectorXd a, const Eigen::VectorXd b);

		//Eigen::Vector3d position_from_msg(const geometry_msgs::Point p);
		//Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		//Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);
		//Eigen::Vector3d vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha);
		//Eigen::Matrix3d extract_yaw_component(const Eigen::Matrix3d r);

		//bool calc_goal_ge_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc);
		Eigen::Affine3d calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe);
		Eigen::Vector3d calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::MatrixXd &Je, const Eigen::VectorXd &rd);
		Eigen::Vector3d calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R);

		int16_t map_pwm(double val);
		//void calc_motor_map(Eigen::MatrixXd &M);

		void message_output_control(const ros::Time t, const std::vector<uint16_t> &pwm);
		void message_output_feedback(const ros::Time t,
									 const Eigen::Affine3d &g_sp,	//Base position/orientation
									 const Eigen::Vector3d &pa,		//Base linear acceleration
									 const Eigen::Matrix3d &r_sp,	//Base rotation
									 const Eigen::Vector3d &g_bw,	//Base body rates
									 const Eigen::VectorXd &ua);	//Base accelerations

		void callback_ready_check(const ros::TimerEvent& e);
		void callback_high_level(const ros::TimerEvent& e);
		void callback_low_level(const ros::TimerEvent& e);
		//void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		//void callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in);
		//void callback_state_mav(const mavros_msgs::State::ConstPtr& msg_in);
		//void callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in);
		//void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		//void callback_goal_path(const nav_msgs::Path::ConstPtr& msg_in);
		//void callback_goal_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
