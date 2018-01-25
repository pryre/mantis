#include <ros/ros.h>

#include <controller_id/controller_id_joint_state.h>

#include <eigen3/Eigen/Dense>

#include <vector>

ControllerIDJointState::ControllerIDJointState(const int num_joints, const double accel_filter) :
	num_joints_(num_joints),
	beta_(accel_filter) {

		Eigen::VectorXd q_ = Eigen::VectorXd::Zero(num_joints_);
		Eigen::VectorXd qd_ = Eigen::VectorXd::Zero(num_joints_);
		Eigen::VectorXd qdd_ = Eigen::VectorXd::Zero(num_joints_);
}

ControllerIDJointState::~ControllerIDJointState() {
}


double ControllerIDJointState::lpf(const double v, const double vp) {
	return ((1.0 - beta_) * v) + (beta_ * vp);
}

void ControllerIDJointState::update_states(const double dt, std::vector<double> q, std::vector<double> qd) {
	ROS_ASSERT_MSG((q.size() == num_joints) && (qd.size() == num_joints), "Error updating joint states: q and qd sizes do not match number of states!");

	for(int i=0; i<num_joints; i++) {
		//Update the acceleration estiamte first
		a = (qd[i] - qd_(i)) / dt;
		qdd_(i) = lpf(a, qdd_(i));

		//Update velocity and position states
		qd_(i) = qd[i];
		q_(i) = q[i];
	}
}

Eigen::VectorXd ControllerIDJointState::q( void ) {
	return q_;
}

Eigen::VectorXd ControllerIDJointState::qd( void ) {
	return qd_;
}

Eigen::VectorXd ControllerIDJointState::qdd( void ) {
	return qdd_;
}
