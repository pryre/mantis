#include <ros/ros.h>

#include <interface_dynamixel/interface_joint.h>

#include <std_msgs/Float64.h>

#include <string>

InterfaceJoint::InterfaceJoint( const ros::NodeHandle& nh,
	std::string joint_name, double ref_timeout_ )
	: nh_( ros::NodeHandle( nh, joint_name ) )
	, current_ref_( InterfaceJoint::CurrentReference::Unset )
	, ref_pos_( 0.0 )
	, ref_vel_( 0.0 )
	, ref_eff_( 0.0 )
	, ref_update_t_( 0 )
	, ref_timeout_( ref_timeout_ ) {

	name_ = joint_name;

	ROS_INFO( "Interface for joint %s initialized", name_.c_str() );

	sub_reference_pos_ = nh_.subscribe<std_msgs::Float64>(
		"reference/position", 10, &InterfaceJoint::callback_reference_pos, this );
	sub_reference_vel_ = nh_.subscribe<std_msgs::Float64>(
		"reference/velocity", 10, &InterfaceJoint::callback_reference_vel, this );
	sub_reference_eff_ = nh_.subscribe<std_msgs::Float64>(
		"reference/effort", 10, &InterfaceJoint::callback_reference_eff, this );
}

InterfaceJoint::~InterfaceJoint() {
}

void InterfaceJoint::callback_reference_pos(
	const std_msgs::Float64::ConstPtr& msg_in ) {
	ref_pos_ = msg_in->data;
	current_ref_ = InterfaceJoint::CurrentReference::Position;
	ref_update_t_ = ros::Time::now();
}

void InterfaceJoint::callback_reference_vel(
	const std_msgs::Float64::ConstPtr& msg_in ) {
	ref_vel_ = msg_in->data;
	current_ref_ = InterfaceJoint::CurrentReference::Velocity;
	ref_update_t_ = ros::Time::now();
}

void InterfaceJoint::callback_reference_eff(
	const std_msgs::Float64::ConstPtr& msg_in ) {
	ref_eff_ = msg_in->data;
	current_ref_ = InterfaceJoint::CurrentReference::Effort;
	ref_update_t_ = ros::Time::now();
}

bool InterfaceJoint::get_reference( double& ref ) {
	bool success = false;

	if ( ( ( ros::Time::now() - ref_update_t_ ) > ref_timeout_ ) && ( ref_update_t_ != ros::Time( 0 ) ) ) {
		if ( current_ref_ == InterfaceJoint::CurrentReference::Position ) {
			ref = ref_pos_;
			success = true;
		} else if ( current_ref_ == InterfaceJoint::CurrentReference::Velocity ) {
			ref = ref_vel_;
			success = true;
		} else if ( current_ref_ == InterfaceJoint::CurrentReference::Effort ) {
			ref = ref_eff_;
			success = true;
		}
	}

	return success;
}

InterfaceJoint::CurrentReference InterfaceJoint::get_reference_type( void ) {
	return current_ref_;
}

const std::string& InterfaceJoint::name( void ) {
	return name_;
}
