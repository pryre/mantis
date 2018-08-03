#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_joints/JointGoalsConfig.h>
#include <mantis_joints/mantis_joints.h>

#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/PathProgress.h>

MantisJoints::MantisJoints() :
	nh_(),
	nhp_("~"),
	p_(nh_),
	param_frame_id_("mantis_uav"),
	param_update_rate_(50.0),
	param_configuration_stamp_(0),
	dyncfg_joint_goals_(ros::NodeHandle(nhp_, "direct_position_control")) {

	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	dyncfg_joint_goals_.setCallback(boost::bind(&MantisJoints::callback_cfg_joint_goals, this, _1, _2));

	if( p_.wait_for_params() ) {
		sub_discrete_progress_ = nhp_.subscribe<contrail_msgs::PathProgress>( "reference/discrete_progress", 10, &MantisJoints::callback_discrete_progress, this );
		sub_cubic_spline_ = nhp_.subscribe<contrail_msgs::CubicSpline>( "reference/spline", 10, &MantisJoints::callback_cubic_spline, this );
		sub_joint_list_ = nhp_.subscribe<mantis_msgs::JointTrajectoryList>( "reference/joint_sepoints", 10, &MantisJoints::callback_joint_list, this );

		//for i in num joints...
			//pushback publisher...
			//pub_tri_ = nhp_.advertise<mavros_msgs::PositionTarget>("output/triplet", 10);

		timer_joints_ = nhp_.createTimer(ros::Duration(1.0/param_update_rate_), &MantisJoints::callback_joints, this );

		ROS_INFO_ONCE("Mantis joint planner loaded!");
	} else {
		ROS_WARN("Mantis joint planner shutting down.");
		ros::shutdown();
	}
}

MantisJoints::~MantisJoints( void ) {
}

void MantisJoints::callback_cfg_joint_goals(mantis_joints::JointGoalsConfig &config, uint32_t level) {
	direct_joint_inputs_.clear();

	direct_joint_inputs_.push_back(config.joint1);
	direct_joint_inputs_.push_back(config.joint2);
	direct_joint_inputs_.push_back(config.joint3);
	direct_joint_inputs_.push_back(config.joint4);
	direct_joint_inputs_.push_back(config.joint5);
	direct_joint_inputs_.push_back(config.joint6);
	direct_joint_inputs_.push_back(config.joint7);
	direct_joint_inputs_.push_back(config.joint8);
}

void MantisJoints::callback_joints(const ros::TimerEvent& e) {

}

void MantisJoints::callback_discrete_progress( const contrail_msgs::PathProgress::ConstPtr& msg_in ) {

}

void MantisJoints::callback_cubic_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in ) {

}

void MantisJoints::callback_joint_list( const mantis_msgs::JointTrajectoryList::ConstPtr& msg_in ) {

}

