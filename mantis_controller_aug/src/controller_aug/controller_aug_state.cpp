#include <ros/ros.h>

#include <controller_aug/controller_aug_state.h>

#include <eigen3/Eigen/Dense>

ControllerAugState::ControllerAugState() :
		new_odom_data_(true),
		new_qd_data_(true),
		num_(0) {

		//Current States
		g_ = Eigen::Affine3d::Identity();
		wv_ = Eigen::Vector3d::Zero();
		bv_ = Eigen::Vector3d::Zero();
		bw_ = Eigen::Vector3d::Zero();

		//Additional state information
		voltage_ = 0.0;
		mav_armed_ = false;

		//High Level Control
		high_level_control_ready_ = false;
		g_sp_ = Eigen::Affine3d::Identity();
		gv_sp_ = Eigen::Vector3d::Zero();
		a_sp_ = Eigen::Vector3d::Zero();
}

ControllerAugState::~ControllerAugState() {
}

void ControllerAugState::init( const int num_manipulator_links ) {
	num_ = 6 + num_manipulator_links;	//XXX: 6 comes from XYZ + Wrpy

	r_ = Eigen::VectorXd::Zero(num_manipulator_links);
	rd_ = Eigen::VectorXd::Zero(num_manipulator_links);
	rdd_ = Eigen::VectorXd::Zero(num_manipulator_links);

	qd_ = Eigen::VectorXd::Zero(num_);
	qd_ << bv_, bw_, rd_;	//Full body-frame velocity state vector
}


void ControllerAugState::update_g( const Eigen::Affine3d &g ) {
	new_odom_data_ = true;

	g_ = g;
}

void ControllerAugState::update_bw( const Eigen::Vector3d &bw ) {
	new_qd_data_ = true;

	bw_ = bw;
}

void ControllerAugState::update_bv( const Eigen::Vector3d &bv ) {
	new_odom_data_ = true;
	new_qd_data_ = true;

	bv_ = bv;
}

void ControllerAugState::update_r( const Eigen::VectorXd &r ) {
	r_ = r;
}

void ControllerAugState::update_rd( const Eigen::VectorXd &rd ) {
	new_qd_data_ = true;

	rd_ = rd;
}

void ControllerAugState::update_rdd( const Eigen::VectorXd &rdd ) {
	rdd_ = rdd;
}

void ControllerAugState::update_voltage( const double voltage ) {
	voltage_ = voltage;
}

void ControllerAugState::update_status_armed( const bool armed ) {
	mav_armed_ = armed;
}

void ControllerAugState::update_status_hl_control( const bool ready ) {
	high_level_control_ready_ = ready;
}


void ControllerAugState::update_g_sp( const Eigen::Affine3d &g_sp ) {
	g_sp_ = g_sp;
}

void ControllerAugState::update_gv_sp( const Eigen::Vector3d &gv_sp ) {
	gv_sp_ = gv_sp;
}

void ControllerAugState::update_a_sp( const Eigen::Vector3d &a_sp ) {
	a_sp_ = a_sp;
}


int ControllerAugState::num( void ) {
	return num_;
}

Eigen::Affine3d ControllerAugState::g( void ) {
	return g_;
}

Eigen::Vector3d ControllerAugState::wv( void ) {
	//Recalculate this if the values has been updated
	if(new_odom_data_) {
		wv_ = g_.linear()*bv_;

		new_odom_data_ = false;
	}

	return wv_;
}

Eigen::Vector3d ControllerAugState::bw( void ) {
	return bw_;
}

Eigen::Vector3d ControllerAugState::bv( void ) {
	return bv_;
}

Eigen::VectorXd ControllerAugState::r( void ) {
	return r_;
}

Eigen::VectorXd ControllerAugState::rd( void ) {
	return rd_;
}

Eigen::VectorXd ControllerAugState::rdd( void ) {
	return rdd_;
}

Eigen::VectorXd ControllerAugState::qd( void ) {
	if(new_qd_data_) {
		qd_ << bv_, bw_, rd_;	//Full body-frame velocity state vector
		new_qd_data_ = false;
	}

	return qd_;
}

double ControllerAugState::voltage( void ) {
	return voltage_;
}

bool ControllerAugState::status_armed( void ) {
	return mav_armed_;
}

bool ControllerAugState::status_hl_control( void ) {
	return high_level_control_ready_;
}


Eigen::Affine3d ControllerAugState::g_sp( void ) {
	return g_sp_;
}

Eigen::Vector3d ControllerAugState::gv_sp( void ) {
	return gv_sp_;
}

Eigen::Vector3d ControllerAugState::a_sp( void ) {
	return a_sp_;
}
