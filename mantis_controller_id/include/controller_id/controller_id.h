#pragma once

#include <ros/ros.h>

#include <controller_id/controller_id_params.h>
#include <pidController/pidController.h>
#include <mantis_paths/path_extract.h>
#include <dh_parameters/dh_parameters.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <eigen3/Eigen/StdVector>

class ControllerID {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;
		ros::Subscriber sub_state_imu_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_joints_;
		//ros::Subscriber sub_goal_path_;
		//ros::Subscriber sub_goal_joints_;

		ros::Publisher pub_rc_;
		ros::Publisher pub_joints_;
		ros::Publisher pub_accel_linear_;
		ros::Publisher pub_accel_body_;
		ros::Publisher pub_twist_base_;
		ros::Publisher pub_twist_end_;
		ros::Publisher pub_pose_base_;
		ros::Publisher pub_pose_end_;

		nav_msgs::Odometry msg_state_odom_;
		sensor_msgs::JointState msg_state_joints_;
		sensor_msgs::Imu msg_state_imu_;
		//nav_msgs::Path msg_goal_path_;
		//sensor_msgs::JointState msg_goal_joints_;

		std::string param_frame_id_;
		std::string param_model_id_;
		bool param_use_imu_state_;
		bool param_wait_for_path_;
		bool param_track_base_;
		bool param_track_j2_;
		bool param_accurate_z_tracking_;
		bool param_accurate_end_tracking_;
		bool param_reference_feedback_;
		double param_rate_;

		ControllerIDParams p_;
		std::vector<DHParameters,Eigen::aligned_allocator<DHParameters> > joints_;
		PathExtract ref_path_;
		pidController pos_pid_x_;
		pidController pos_pid_y_;
		pidController pos_pid_z_;

	public:
		ControllerID( void );

		~ControllerID( void );

		double double_clamp(const double v, const double min, const double max);
		void matrix_clamp(Eigen::MatrixXd m, const double min, const double max);
		Eigen::VectorXd vector_interlace(const Eigen::VectorXd a, const Eigen::VectorXd b);

		Eigen::Vector3d position_from_msg(const geometry_msgs::Point p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);
		//Eigen::Vector3d vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha);
		Eigen::Matrix3d extract_yaw_component(const Eigen::Matrix3d r);

		//bool calc_goal_ge_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc);
		Eigen::Affine3d calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe);
		Eigen::Vector3d calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::MatrixXd &Je, const Eigen::VectorXd &rd);
		Eigen::Vector3d calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R);

		int16_t map_pwm(double val);
		void calc_motor_map(Eigen::MatrixXd &M);

		void message_output_control(const ros::Time t, const std::vector<uint16_t> &pwm, const std::vector<double> &joints);
		void message_output_feedback(const ros::Time t,
									 const Eigen::Affine3d &g_sp,
									 const Eigen::Affine3d &ge_sp,
									 const Eigen::Vector3d &gv_sp,
									 const Eigen::Vector3d &gev_sp,
									 const Eigen::Vector3d &pa,
									 const Eigen::Matrix3d &r_sp,
									 const Eigen::VectorXd &ua);

		void callback_control(const ros::TimerEvent& e);
		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		//void callback_goal_path(const nav_msgs::Path::ConstPtr& msg_in);
		//void callback_goal_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
