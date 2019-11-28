#include <eigen3/Eigen/Dense>

#include <mantis_description/se_tools.h>
#include <mantis_msgs/State.h>
#include <mantis_params/param_client.h>
#include <mantis_state/state_client.h>
#include <ros/ros.h>

namespace MantisState {

Client::Client( const ros::NodeHandle& nh,
	MantisParams::Client& p )
	: nh_( nh )
	, p_( p )
	, timestamp_( 0 )
	, voltage_( 0.0 )
	, frame_id_( "" )
	, child_frame_id_( "" )
	, mav_safety_disengaged_( false )
	, mav_armed_(false) {

	g_ = Eigen::Affine3d::Identity();
	bv_ = Eigen::Vector3d::Zero();
	ba_ = Eigen::Vector3d::Zero();
	bw_ = Eigen::Vector3d::Zero();
	//bwa_ = Eigen::Vector3d::Zero();

	sub_state_ = nh_.subscribe<mantis_msgs::State>( "state",
													1,
													&Client::callback_state,
													this );
}

Client::~Client() {
}

bool Client::ok( void ) {
	bool ok = true;

	if ( timestamp_ == ros::Time( 0 ) )
		ok = false;

	if ( configuration_stamp_ != p_.get(MantisParams::PARAM_TIME_CHANGE_CONFIG) )
		ok = false;

	return ok;
}

const ros::Time& Client::time_updated( void ) {
	return timestamp_;
}

const ros::Time& Client::state_configuration_stamp( void ) {
	return configuration_stamp_;
}

bool Client::wait_for_state( void ) {
	while ( ros::ok() && ( !ok() ) ) {
		ros::spinOnce();
		ros::Rate( 10 ).sleep();
	}

	return ok();
}

const std::string& Client::frame_id( void ) {
	return frame_id_;
}

const std::string& Client::model_id( void ) {
	return child_frame_id_;
}

const Eigen::Affine3d& Client::g( void ) {
	return g_;
}

const Eigen::Vector3d& Client::bv( void ) {
	return bv_;
}

Eigen::Vector3d Client::wv( void ) {
	return g_.linear() * bv_;
}

const Eigen::Vector3d& Client::bw( void ) {
	return bw_;
}

const Eigen::Vector3d& Client::ba( void ) {
	return ba_;
}
/*
const Eigen::Vector3d& Client::bwa( void ) {
	return bwa_;
}
*/
const Eigen::VectorXd& Client::r( void ) {
	return r_;
}

const Eigen::VectorXd& Client::rd( void ) {
	return rd_;
}
/*
const Eigen::VectorXd& Client::rdd( void ) {
	return rdd_;
}
*/
const double& Client::voltage( void ) {
	return voltage_;
}

const bool& Client::mav_safety_disengaged( void ) {
	return mav_safety_disengaged_;
}

const bool& Client::mav_safety_armed( void ) {
	return mav_armed_;
}

bool Client::flight_ready( void ) {
	return ( this->ok() &&
			 this->mav_safety_disengaged() &&
			 this->mav_safety_armed() );
}

void Client::callback_state(
	const mantis_msgs::State::ConstPtr& msg_in ) {
	timestamp_ = msg_in->header.stamp;
	configuration_stamp_ = msg_in->configuration_stamp;

	if(frame_id_.length() == 0)
		frame_id_ = msg_in->header.frame_id;
	if(child_frame_id_.length() == 0)
		child_frame_id_ = msg_in->child_frame_id;

	g_ = MDTools::affine_from_msg( msg_in->pose );
	bv_ = MDTools::vector_from_msg( msg_in->twist.linear );
	bw_ = MDTools::vector_from_msg( msg_in->twist.angular );
	ba_ = MDTools::vector_from_msg( msg_in->accel.linear );
	//bwa_ = MDTools::vector_from_msg( msg_in->accel.angular );

	int num_manip_states = msg_in->r.size();
	if ( r_.size() != num_manip_states ) {
		// Resize our vectors if the state changed
		r_ = Eigen::VectorXd::Zero( num_manip_states );
		rd_ = Eigen::VectorXd::Zero( num_manip_states );
		//rdd_ = Eigen::VectorXd::Zero( num_manip_states );
	}

	for ( int i = 0; i < num_manip_states; i++ ) {
		r_[i] = msg_in->r[i];
		rd_[i] = msg_in->rd[i];
		//rdd_[i] = msg_in->rdd[i];
	}

	voltage_ = msg_in->battery_voltage;
	mav_safety_disengaged_ = msg_in->mav_safety_disengaged;
	mav_armed_ = msg_in->mav_ready;
}

};
