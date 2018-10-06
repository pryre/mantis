#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <tinyspline_ros/tinysplinecpp.h>

#include <mantis_router_base/ControlParamsConfig.h>
#include <actionlib/server/simple_action_server.h>
#include <mantis_router_base/BaseMovementAction.h>

#include <mantis_params/param_client.h>
#include <mantis_state/state_client.h>
#include <mantis_kinematics/solver.h>

//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/StdVector>
#include <string>

class MantisRouterBase {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_path_;

		//ros::Publisher pub_traj_;
		ros::Publisher pub_tri_;
		ros::Publisher pub_pose_base_;
		ros::Publisher pub_pose_track_;

		std::string param_frame_id_;
		std::string param_model_id_;
		int param_tracked_frame_;
		double param_manipulator_jacobian_a_;
		bool param_send_reference_feedback_;
		double param_path_rate_;
		dynamic_reconfigure::Server<mantis_router_base::ControlParamsConfig> dyncfg_control_settings_;

		MantisParamClient p_;
		MantisStateClient s_;
		MantisSolver solver_;

		ros::Time spline_start_;
		ros::Duration spline_duration_;
		bool spline_in_progress_;
		Eigen::Vector3d spline_pos_start_;
		Eigen::Vector3d spline_pos_end_;
		double spline_rot_start_;
		double spline_rot_end_;
		tinyspline::BSpline spline_x_;
		tinyspline::BSpline spline_xd_;
		tinyspline::BSpline spline_y_;
		tinyspline::BSpline spline_yd_;
		tinyspline::BSpline spline_z_;
		tinyspline::BSpline spline_zd_;
		tinyspline::BSpline spline_r_;
		tinyspline::BSpline spline_rd_;
		bool use_dirty_derivative_;

		actionlib::SimpleActionServer<mantis_router_base::BaseMovementAction> as_;

		Eigen::Matrix<double,6,1> vbe_last_;

	public:
		MantisRouterBase( void );

		~MantisRouterBase( void );

		void callback_cfg_control_settings(mantis_router_base::ControlParamsConfig &config, uint32_t level);

		//Eigen::Vector3d position_from_msg(const geometry_msgs::Point p);
		//Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion q);
		//Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose pose);

		//geometry_msgs::Vector3 vector_from_eig(const Eigen::Vector3d &v);
		//geometry_msgs::Point point_from_eig(const Eigen::Vector3d &p);
		//geometry_msgs::Quaternion quaternion_from_eig(const Eigen::Quaterniond &q);
		//geometry_msgs::Pose pose_from_eig(const Eigen::Affine3d &g);

		bool calc_goal_ge_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc);
		Eigen::Affine3d calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe);
		Eigen::Vector3d calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::VectorXd &vbe);

		void callback_path(const ros::TimerEvent& e);

		void set_action_goal();

		bool get_reference(Eigen::Vector3d &pos, Eigen::Vector3d &vel, double &rpos, double &rrate, const ros::Time tc);
		void get_spline_reference(tinyspline::BSpline& spline, tinyspline::BSpline& splined, double& pos, double& vel, const double u);

		inline double normalize(double x, const double min, const double max) const {
			return (x - min) / (max - min);
		}
};
