#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_joints/JointGoalsConfig.h>
#include <mantis_joints/mantis_joints.h>

#include <mantis_msgs/JointTrajectoryGoal.h>
#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/PathProgress.h>

MantisJoints::MantisJoints() :
	nh_(),
	nhp_("~"),
	p_(nh_),
	tracked_input_(MantisJoints::LastJointReference::Undefined),
	param_frame_id_("mantis_uav"),
	param_update_rate_(50.0),
	param_configuration_stamp_(0),
	current_discrete_point_(0),
	dyncfg_joint_goals_(ros::NodeHandle(nhp_, "direct_position_control")) {

	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	dyncfg_joint_goals_.setCallback(boost::bind(&MantisJoints::callback_cfg_joint_goals, this, _1, _2));

	if( p_.wait_for_params() ) {
		sub_discrete_progress_ = nhp_.subscribe<contrail_msgs::PathProgress>( "reference/discrete_progress", 10, &MantisJoints::callback_discrete_progress, this );
		sub_cubic_spline_ = nhp_.subscribe<contrail_msgs::CubicSpline>( "reference/spline", 10, &MantisJoints::callback_cubic_spline, this );
		sub_joint_list_ = nhp_.subscribe<mantis_msgs::JointTrajectoryList>( "reference/joint_sepoints", 10, &MantisJoints::callback_joint_list, this );

		configure_publishers();

		timer_joints_ = nhp_.createTimer(ros::Duration(1.0/param_update_rate_), &MantisJoints::callback_joints, this );

		ROS_INFO_ONCE("Mantis joint planner loaded!");
	} else {
		ROS_WARN("Mantis joint planner shutting down.");
		ros::shutdown();
	}
}

MantisJoints::~MantisJoints( void ) {
}

void MantisJoints::configure_publishers( void ) {
	if(tracked_input_ != MantisJoints::LastJointReference::Undefined ) {
		param_configuration_stamp_ = p_.time_configuration_change();

		pub_joints_.clear();
		for(int i=0; i<p_.get_dynamic_joint_num(); i++) {
			pub_joints_.push_back(ros::Publisher());
			pub_joints_.back() = nhp_.advertise<mantis_msgs::JointTrajectoryGoal>("command/joint" + std::to_string(i), 10);
		}
	}
}

void MantisJoints::callback_joints(const ros::TimerEvent& e) {
	//Double check the configuration hasn't changed
	if( param_configuration_stamp_ != p_.time_configuration_change() )
		configure_publishers();

	//Loop through each dynamic joint
	for(int i=0; i<p_.get_dynamic_joint_num(); i++) {
		bool success = false;

		mantis_msgs::JointTrajectoryGoal msg_out;
		msg_out.header.frame_id = param_frame_id_;
		msg_out.header.stamp = e.current_real;

		if(tracked_input_ == MantisJoints::LastJointReference::DirectInput) {
			success = get_direct_input_reference(msg_out.position, msg_out.velocity, i);
		} else if(tracked_input_ == MantisJoints::LastJointReference::DiscreteProgress) {
			success = get_discrete_progress_reference(msg_out.position, msg_out.velocity, i);
		} else if(tracked_input_ == MantisJoints::LastJointReference::CubicSpline) {
			success = get_cubic_spline_reference(msg_out.position, msg_out.velocity, i);
		} //Undefined is already covered

		if(success)
			pub_joints_[i].publish(msg_out);
	}
}

void MantisJoints::callback_cfg_joint_goals(mantis_joints::JointGoalsConfig &config, uint32_t level) {
	joint_list_.header.frame_id = param_frame_id_;
	joint_list_.header.stamp = ros::Time::now();

	joint_list_.num_joints = 8;
	joint_list_.num_points = 1;

	joint_list_.joint_interp_points.clear();
	joint_list_.joint_interp_points.push_back(config.joint1);
	joint_list_.joint_interp_points.push_back(config.joint2);
	joint_list_.joint_interp_points.push_back(config.joint3);
	joint_list_.joint_interp_points.push_back(config.joint4);
	joint_list_.joint_interp_points.push_back(config.joint5);
	joint_list_.joint_interp_points.push_back(config.joint6);
	joint_list_.joint_interp_points.push_back(config.joint7);
	joint_list_.joint_interp_points.push_back(config.joint8);

	tracked_input_ = MantisJoints::LastJointReference::DirectInput;
}

bool MantisJoints::get_direct_input_reference( double &pos_ref, double &vel_ref, const unsigned int index ) {
	bool success = false;

	if( index < joint_list_.num_joints ) {
		//We only use a n*1 list for direct input, so directly extract reference
		pos_ref = joint_list_.joint_interp_points[index];
		vel_ref = 0.0;
	}

	return success;
}

bool MantisJoints::get_discrete_progress_reference( double &pos_ref, double &vel_ref, const unsigned int index ) {
	bool success = false;

	if( ( index < joint_list_.num_joints ) &&
		( current_discrete_point_ < joint_list_.num_points ) ) {
		pos_ref = joint_list_.joint_interp_points[index + (index*current_discrete_point_)];
		vel_ref = 0.0;

		success = true;
	}

	return success;
}

bool MantisJoints::get_cubic_spline_reference( double &pos_ref, double &vel_ref, const unsigned int index ) {
	bool success = false;

	//pos_ref = 0.0;
	//vel_ref = 0.0;

	return success;
}

void MantisJoints::callback_discrete_progress( const contrail_msgs::PathProgress::ConstPtr& msg_in ) {
	current_discrete_point_ = msg_in->current;

	tracked_input_ = MantisJoints::LastJointReference::DiscreteProgress;
}

void MantisJoints::callback_cubic_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in ) {

	tracked_input_ = MantisJoints::LastJointReference::CubicSpline;
}

void MantisJoints::callback_joint_list( const mantis_msgs::JointTrajectoryList::ConstPtr& msg_in ) {

}
