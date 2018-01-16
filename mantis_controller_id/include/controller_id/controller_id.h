#pragma once

#include <ros/ros.h>

#include <controller_id/controller_id_params.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
#include <string>

class ControllerID {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Subscriber sub_state_odom_;
		ros::Subscriber sub_state_joints_;
		ros::Subscriber sub_goal_path_;
		ros::Subscriber sub_goal_joints_;

		ros::Publisher pub_rc_;
		ros::Publisher pub_r1_;
		ros::Publisher pub_r2_;
		ros::Publisher pub_accel_linear_;
		ros::Publisher pub_accel_body_;
		ros::Publisher pub_joints_;
		ros::Publisher pub_pose_;

		nav_msgs::Odometry msg_state_odom_;
		sensor_msgs::JointState msg_state_joints_;
		nav_msgs::Path msg_goal_path_;
		sensor_msgs::JointState msg_goal_joints_;

		Eigen::Affine3d latest_g_sp_;

		std::string param_frame_id_;
		std::string param_model_id_;
		double param_rate_;
		ControllerIDParams p_;

	public:
		ControllerID( void );

		~ControllerID( void );

		void matrix_clamp(Eigen::MatrixXd m, const double min, const double max);
		Eigen::VectorXd vector_interlace(const Eigen::VectorXd a, const Eigen::VectorXd b);

		Eigen::Vector3d position_from_msg(const geometry_msgs::Point p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);
		Eigen::Vector3d vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha);

		bool calc_goal_g_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc);
		Eigen::Vector3d calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R);

		int16_t map_pwm(double val);
		void calc_motor_map(Eigen::MatrixXd &M);

		void callback_control(const ros::TimerEvent& e);
		void callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in);
		void callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
		void callback_goal_path(const nav_msgs::Path::ConstPtr& msg_in);
		void callback_goal_joints(const sensor_msgs::JointState::ConstPtr& msg_in);
};
