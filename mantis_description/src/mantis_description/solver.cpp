#include <eigen3/Eigen/Dense>

#include <ros/ros.h>

#include <mantis_description/solver.h>

#include <mantis_description/se_tools.h>
#include <mantis_description/dynamics/calc_Dq.h>
#include <mantis_description/dynamics/calc_Cqqd.h>
#include <mantis_description/dynamics/calc_Lqd.h>
#include <mantis_description/dynamics/calc_Jj2.h>
#include <mantis_description/dynamics/calc_Je.h>


MantisSolver::MantisSolver( MantisParamClient *p, MantisStateClient *s ) :
	p_(p),
	s_(s),
	param_load_time_(0),
	state_load_time_(0) {

	check_description();
}

MantisSolver::~MantisSolver() {
}

bool MantisSolver::check_description( void ) {
	bool success = true;

	success &= check_parameters();
	success &= check_state();

	if(!success) {
		ROS_ERROR("Unable to validate params and state");
	}

	return success;
}

bool MantisSolver::check_parameters( void ) {
	bool success = true;

	if(p_->time_updated() != param_load_time_) {
		success &= load_parameters();
	}

	return success;
}

bool MantisSolver::check_state( void ) {
	bool success = true;

	if(s_->time_updated() != state_load_time_) {
		success &= load_state();
	}

	return success;
}

//TODO: Could use this to dynamically load in the dynamics solver
bool MantisSolver::load_parameters( void ) {
	bool success = true;

	//Load in the link definitions
	joints_.clear();

	for(int i=0; i<p_->get_joint_num(); i++) {
		DHParameters dh( p_->joint(i) );

		if( dh.is_valid() ) {
			joints_.push_back(dh);
		} else {
			ROS_FATAL("Error loading joint %i", int(i));
			success = false;
			break;
		}
	}

	//Invalidate the current state as the system may have changed
	state_load_time_ = ros::Time(0);

	if(success) {
		param_load_time_ = p_->time_updated();
	}

	return success;
}

bool MantisSolver::load_state( void ) {
	bool success = true;

	Eigen::VectorXd r = s_->r();
	Eigen::VectorXd rd = s_->rd();

	if( ( r.size() == rd.size() ) && (p_->get_dynamic_joint_num() == r.size() ) ) {
		int jc = 0;
		for(int i=0; i<joints_.size(); i++) {
			if(joints_[i].jt() != DHParameters::JointType::Static) {
				joints_[i].update(r[jc], rd[jc]);
				jc++;
			}
		}
	} else {
		ROS_ERROR("Error: MantisSolver::load_state(): Dynamic joint sizes are inconsistent");
		success = false;
	}

	if(success) {
		state_load_time_ = s_->time_updated();
	}

	return success;
}

int MantisSolver::num_states( void ) {
	return 6 + p_->get_dynamic_joint_num();
}

bool MantisSolver::solve_inverse_dynamics( Eigen::VectorXd &tau, const Eigen::VectorXd &ua ) {
	bool success = false;

	if( check_description() ) {
		//Need to copy this data to access it directly
		Eigen::VectorXd r = s_->r();
		Eigen::VectorXd rd = s_->rd();
		Eigen::Vector3d bw = s_->bw();
		Eigen::Vector3d bv = s_->bv();

		Eigen::VectorXd qd(num_states());
		qd << bv, bw, rd;

		if( ( ua.size() == qd.size() ) && ( tau.size() == qd.size() ) && ( num_states() == qd.size() ) ) {
			//Calculate dynamics matricies
			Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_states(), num_states());
			Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_states(), num_states());
			Eigen::MatrixXd L = Eigen::MatrixXd::Zero(num_states(), num_states());

			//XXX: Take care!
			double l0 = joints_[0].d();
			double l1 = joints_[1].r();

			calc_Dq(D,
					p_->body_inertial(0).Ixx, p_->body_inertial(0).Iyy, p_->body_inertial(0).Izz,
					p_->body_inertial(1).Ixx, p_->body_inertial(1).Iyy, p_->body_inertial(1).Izz,
					p_->body_inertial(2).Ixx, p_->body_inertial(2).Iyy, p_->body_inertial(2).Izz,
					l0, l1, p_->body_inertial(1).com, p_->body_inertial(2).com,
					p_->body_inertial(0).mass, p_->body_inertial(1).mass, p_->body_inertial(2).mass,
					r(0), r(1));

			calc_Cqqd(C,
					  p_->body_inertial(1).Ixx, p_->body_inertial(1).Iyy,
					  p_->body_inertial(2).Ixx, p_->body_inertial(2).Iyy,
					  bv(0), bv(1), bv(2), bw(0), bw(1), bw(2),
					  l0, l1, p_->body_inertial(1).com, p_->body_inertial(2).com,
					  p_->body_inertial(1).mass, p_->body_inertial(2).mass,
					  r(0), rd(0), r(1), rd(1));

			tau = D*ua + (C + L)*qd;

			success = true;
		} else {
			ROS_ERROR("Error: MantisSolver::solve_inverse_dynamics(): Vector lengths inconsistent");
		}
	}

	return success;
}

//TODO: Failure conditions
bool MantisSolver::calculate_Je( Eigen::MatrixXd &Je ) {
	bool success = true;
	Je = Eigen::MatrixXd::Zero(6, s_->rd().size());

	if( check_description() ) {
		calc_Je(Je,
			joints_[1].r(),
			joints_[2].r(),
			joints_[2].q());

		success = true;
	}

	return success;
}

bool MantisSolver::calculate_gbe( Eigen::Affine3d &gbe ) {
	bool success = false;

	gbe = Eigen::Affine3d::Identity();

	if( check_description() ) {
		for(int i=0; i<joints_.size(); i++) {
			gbe = gbe * joints_[i].transform();
		}

		success = true;
	}

	return success;
}
