#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <mantis_msgs/State.h>
#include <mantis_description/state_client.h>

MantisStateClient::MantisStateClient( ros::NodeHandle *nh ) :
	nh_(nh),
	timestamp_(0),
	voltage_(0.0),
	flight_ready_(false) {

	g_ = Eigen::Affine3d::Identity();
	bv_ = Eigen::Vector3d::Zero();
	ba_ = Eigen::Vector3d::Zero();
	bw_ = Eigen::Vector3d::Zero();
	bwa_ = Eigen::Vector3d::Zero();

	sub_state_ = nh_->subscribe<mantis_msgs::State>( "state", 1, &MantisStateClient::callback_state, this );
}

MantisStateClient::~MantisStateClient() {
}

bool MantisStateClient::ok( void ) {
	return ( timestamp_ != ros::Time(0) );
}

ros::Time MantisStateClient::time_updated( void ) {
	return timestamp_;
}

Eigen::Affine3d MantisStateClient::g( void ) {
	return g_;
}

Eigen::Vector3d MantisStateClient::bv( void ) {
	return bv_;
}

Eigen::Vector3d MantisStateClient::wv( void ) {
	return g_.linear()*bv_;
}

Eigen::Vector3d MantisStateClient::bw( void ) {
	return bw_;
}

Eigen::Vector3d MantisStateClient::ba( void ) {
	return ba_;
}

Eigen::Vector3d MantisStateClient::bwa( void ) {
	return bwa_;
}

Eigen::VectorXd MantisStateClient::r( void ) {
	return r_;
}

Eigen::VectorXd MantisStateClient::rd( void ) {
	return rd_;
}

Eigen::VectorXd MantisStateClient::rdd( void ) {
	return rdd_;
}

double MantisStateClient::voltage( void ) {
	return voltage_;
}

bool MantisStateClient::flight_ready( void ) {
	return flight_ready_;
}

void MantisStateClient::callback_state(const mantis_msgs::State::ConstPtr &msg_in) {
	timestamp_ = msg_in->header.stamp;

	Eigen::Affine3d g_ = affine_from_msg(msg_in->pose);
	Eigen::Vector3d bv_ = vector_from_msg(msg_in->twist.linear);
	Eigen::Vector3d bw_ = vector_from_msg(msg_in->twist.angular);
	Eigen::Vector3d ba_ = vector_from_msg(msg_in->accel.linear);
	Eigen::Vector3d bwa_ = vector_from_msg(msg_in->accel.angular);

	int num_manip_states = msg_in->r.size();
	r_.resize(num_manip_states);
	rd_.resize(num_manip_states);
	rdd_.resize(num_manip_states);

	for(int i=0; i<num_manip_states; i++) {
		r_[i] = msg_in->r[i];
		rd_[i] = msg_in->rd[i];
		rdd_[i] = msg_in->rdd[i];
	}

	voltage_ = msg_in->battery_voltage;
	flight_ready_ = msg_in->flight_ready;
}

Eigen::Vector3d MantisStateClient::vector_from_msg(const geometry_msgs::Vector3 v) {
		return Eigen::Vector3d(v.x, v.y, v.z);
}

Eigen::Vector3d MantisStateClient::point_from_msg(const geometry_msgs::Point p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond MantisStateClient::quaternion_from_msg(const geometry_msgs::Quaternion q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d MantisStateClient::affine_from_msg(const geometry_msgs::Pose pose) {
		Eigen::Affine3d a;

		a.translation() << point_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}
