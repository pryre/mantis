#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_guidance_full/ControlParamsConfig.h>

#include <mantis_params/param_client.h>
#include <mantis_state/state_client.h>
#include <mantis_kinematics/solver.h>
#include <contrail_manager/ContrailManager.h>

#include <mantis_guidance_full/joint.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace MantisGuidanceFull {

class Manager {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;

		ros::Publisher pub_tri_;
		ros::Publisher pub_pose_base_;
		ros::Publisher pub_pose_track_;

		ros::Time spawn_stamp_;
		double param_update_rate_;

		int param_tracked_frame_;
		double param_manipulator_jacobian_a_;
		bool param_send_reference_feedback_;
		dynamic_reconfigure::Server<mantis_guidance_full::ControlParamsConfig> dyncfg_control_settings_;

		std::vector<MantisGuidanceFull::Joint*> joint_routers_;

		MantisParams::Client p_;
		MantisState::Client s_;
		MantisSolver solver_;
		ContrailManager contrail_;

		Eigen::Matrix<double,6,1> vbe_last_;

	public:
		Manager( void );

		~Manager( void );

		void callback_timer(const ros::TimerEvent& e);
		void callback_cfg_control_settings(mantis_guidance_full::ControlParamsConfig &config, uint32_t level);

		void configure_joint_routers( void );
		void remove_joint_routers( void );

		bool calc_goal_ge_sp(Eigen::Affine3d &g_sp, Eigen::Vector3d &v_sp, const ros::Time tc);
		Eigen::Affine3d calc_goal_base_transform(const Eigen::Affine3d &ge_sp, const Eigen::Affine3d &gbe);
		Eigen::Vector3d calc_goal_base_velocity(const Eigen::Vector3d &gev_sp, const Eigen::Matrix3d &Re, const Eigen::VectorXd &vbe);

		void callback_path(const ros::TimerEvent& e);
};

}
