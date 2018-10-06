#pragma once

#include <ros/ros.h>

#include <mantis_params/param_client.h>
#include <mantis_msgs/JointTrajectoryGoal.h>

#include <tinyspline_ros/tinysplinecpp.h>

#include <actionlib/server/simple_action_server.h>
#include <mantis_router_joints/JointMovementAction.h>

#include <vector>
#include <string>

namespace MantisRouterJoints {

class Joint {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_traj_;

		std::string name_;

		ros::Time spline_start_;
		ros::Duration spline_duration_;
		bool spline_in_progress_;
		double spline_pos_start_;
		double spline_pos_end_;
		tinyspline::BSpline spline_;
		tinyspline::BSpline splined_;
		bool use_dirty_derivative_;

		actionlib::SimpleActionServer<mantis_router_joints::JointMovementAction> as_;

	public:
		Joint( const ros::NodeHandle& nh, std::string joint_name );

		~Joint();

		void update( ros::Time tc );

	private:
		//void action_goal_cb( const mantis_router_joints::JointMovementGoalConstPtr &goal );
		void set_action_goal();

		void get_spline_reference(double& pos, double& vel, const double u) const;

		inline double normalize(double x, const double min, const double max) const {
			return (x - min) / (max - min);
		}
};

}
