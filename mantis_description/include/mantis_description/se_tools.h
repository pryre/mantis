#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <mantis_msgs/BodyInertial.h>

#include <math.h>

//! The Mantis Description Tools Namespace
namespace MDTools {
	//Math Helpers
	Eigen::Matrix3d vee_up(const Eigen::Vector3d& w);
	Eigen::Vector3d vee_down(const Eigen::Matrix3d& W);
	Eigen::Matrix<double,6,6> adjoint(const Eigen::Affine3d& g);
	Eigen::Matrix<double,6,6> full_inertial(const mantis_msgs::BodyInertial& i);
	double double_clamp(const double v, const double min, const double max);
	void matrix_clamp(Eigen::MatrixXd &m, const double min, const double max);
	Eigen::Matrix3d extract_yaw_matrix(const Eigen::Matrix3d& r);
	double extract_yaw( const Eigen::Quaterniond &q );

	//Conversion Helpers
	Eigen::Vector3d vector_from_msg(const geometry_msgs::Vector3 &v);
	Eigen::Vector3d point_from_msg(const geometry_msgs::Point &p);
	Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion &q);
	Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose &pose);
	geometry_msgs::Vector3 vector_from_eig(const Eigen::Vector3d &v);
	geometry_msgs::Point point_from_eig(const Eigen::Vector3d &p);
	geometry_msgs::Quaternion quaternion_from_eig(const Eigen::Quaterniond &q);
	geometry_msgs::Pose pose_from_eig(const Eigen::Affine3d &g);
}
