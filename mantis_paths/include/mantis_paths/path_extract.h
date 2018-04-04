#pragma once

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>

class PathExtract {
	private:
		ros::NodeHandle *nh_;

		ros::Subscriber sub_path_;

		nav_msgs::Path p_c_;
		int path_hint_;
		bool have_path_;

		Eigen::Affine3d latest_g_;

	public:
		PathExtract( ros::NodeHandle *nh, Eigen::Affine3d init_pose = Eigen::Affine3d::Identity() );

		~PathExtract( void );

		void set_latest(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);

		bool get_ref_state(Eigen::Affine3d &g_c, Eigen::Vector3d &v_c, const ros::Time tc);

		bool received_valid_path( void );

	private:
		void callback_path(const nav_msgs::Path::ConstPtr& msg_in);

		void reset_hinting(void);

		Eigen::Vector3d position_from_msg(const geometry_msgs::Point p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);

		Eigen::Vector3d vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha);
};
