#include <ros/ros.h>

#include <controller_id/controller_id_state.h>

#include <eigen3/Eigen/Dense>

ControllerIDState::ControllerIDState() {
		num_ = 0;

		//Current States
		g_ = Eigen::Affine3d::Identity();
		bw_ = Eigen::Vector3d::Zero();
		bv_ = Eigen::Vector3d::Zero();

		//Additional state information
		voltage_ = 0.0;
		mav_armed_ = false;

		//High Level Control
		high_level_control_ready_ = false;
		g_sp_ = Eigen::Affine3d::Identity();
		gv_sp_ = Eigen::Vector3d::Zero();
		a_sp_ = Eigen::Vector3d::Zero();
}

ControllerIDState::~ControllerIDState() {
}

void ControllerIDState::init( const int num_manipulator_links ) {
	num_ = 6 + num_manipulator_links;	//XXX: 6 comes from XYZ + Wrpy

	r_ = Eigen::VectorXd::Zero(num_manipulator_links);
	rd_ = Eigen::VectorXd::Zero(num_manipulator_links);
	rdd_ = Eigen::VectorXd::Zero(num_manipulator_links);

	qd_ = Eigen::VectorXd::Zero(num_);
	qd_ << bv_, bw_, rd_;	//Full body-frame velocity state vector
}


void ControllerIDState::update_g( const Eigen::Affine3d &g ) {
	g_ = g;
}

void ControllerIDState::update_bw( const Eigen::Vector3d &bw ) {
	bw_ = bw;
	qd_ << bv_, bw_, rd_;	//Full body-frame velocity state vector
}

void ControllerIDState::update_bv( const Eigen::Vector3d &bv ) {
	bv_ = bv;
	qd_ << bv_, bw_, rd_;	//Full body-frame velocity state vector
}

void ControllerIDState::update_r( const Eigen::VectorXd &r ) {
	r_ = r;
}

void ControllerIDState::update_rd( const Eigen::VectorXd &rd ) {
	rd_ = rd;
	qd_ << bv_, bw_, rd_;	//Full body-frame velocity state vector
}

void ControllerIDState::update_rdd( const Eigen::VectorXd &rdd ) {
	rdd_ = rdd;
}

void ControllerIDState::update_voltage( const double voltage ) {
	voltage_ = voltage;
}

void ControllerIDState::update_status_armed( const bool armed ) {
	mav_armed_ = armed;
}

void ControllerIDState::update_status_hl_control( const bool ready ) {
	high_level_control_ready_ = ready;
}


void ControllerIDState::update_g_sp( const Eigen::Affine3d &g_sp ) {
	g_sp_ = g_sp;
}

void ControllerIDState::update_gv_sp( const Eigen::Vector3d &gv_sp ) {
	gv_sp_ = gv_sp;
}

void ControllerIDState::update_a_sp( const Eigen::Vector3d &a_sp ) {
	a_sp_ = a_sp;
}


int ControllerIDState::num( void ) {
	return num_;
}

Eigen::Affine3d ControllerIDState::g( void ) {
	return g_;
}

Eigen::Vector3d ControllerIDState::bw( void ) {
	return bw_;
}

Eigen::Vector3d ControllerIDState::bv( void ) {
	return bv_;
}

Eigen::VectorXd ControllerIDState::r( void ) {
	return r_;
}

Eigen::VectorXd ControllerIDState::rd( void ) {
	return rd_;
}

Eigen::VectorXd ControllerIDState::rdd( void ) {
	return rdd_;
}

Eigen::VectorXd ControllerIDState::qd( void ) {
	return qd_;
}

double ControllerIDState::voltage( void ) {
	return voltage_;
}

bool ControllerIDState::status_armed( void ) {
	return mav_armed_;
}

bool ControllerIDState::status_hl_control( void ) {
	return high_level_control_ready_;
}


Eigen::Affine3d ControllerIDState::g_sp( void ) {
	return g_sp_;
}

Eigen::Vector3d ControllerIDState::gv_sp( void ) {
	return gv_sp_;
}

Eigen::Vector3d ControllerIDState::a_sp( void ) {
	return a_sp_;
}
