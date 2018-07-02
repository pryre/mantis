#include <eigen3/Eigen/Dense>

#include <ros/ros.h>

#include <mantis_kinematics/solver.h>

#include <mantis_description/se_tools.h>
#include <mantis_kinematics/dynamics/calc_Dq.h>
#include <mantis_kinematics/dynamics/calc_Cqqd.h>
#include <mantis_kinematics/dynamics/calc_Lqd.h>
#include <mantis_kinematics/dynamics/calc_Jj2.h>
#include <mantis_kinematics/dynamics/calc_Je.h>


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

bool MantisSolver::calculate_gxy( Eigen::Affine3d &g, const unsigned int x, const unsigned int y ) {
	bool success = false;

	g = Eigen::Affine3d::Identity();

	if( check_description() ) {
		if( (x <= joints_.size()) && (y <= joints_.size()) && (x !=y ) ) {
			//The inverse calculation was requested
			if(x > y) {
				for(int i=y; i<x; i++) {
					g = g * joints_[i].transform();
				}

				g = g.inverse();
			} else {
				for(int i=x; i<y; i++) {
					g = g * joints_[i].transform();
				}
			}

			success = true;
		} else {
			ROS_ERROR("Error: MantisSolver::calculate_gxy(): frame inputs invalid");
		}
	}

	return success;
}

bool MantisSolver::calculate_thrust_coeffs( double &kT, double &ktx, double &kty, double &ktz) {
	bool success = false;

	//double rpm_max = p_->motor_kv() * s_->voltage();	//Get the theoretical maximum rpm at the current battery voltage
	//double thrust_single = p_->rpm_thrust_m() * rpm_max + p_->rpm_thrust_c();	//Use the RPM to calculate maximum thrust
	double thrust_single = 0.8*9.80665;

	if(p_->airframe_type() == "quad_x4") {
		double arm_ang = M_PI / 4.0; //45Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (4.0 * la * std::sin(arm_ang) * thrust_single);
		kty = ktx; //Airframe is symmetric
		ktz = 1.0 / (p_->motor_num() * p_->motor_drag_max());

		success = true;
	} else if(p_->airframe_type() == "hex_x6") {
		double arm_ang = (M_PI / 6.0); //30Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (2.0 * la * (2.0 * std::sin(arm_ang) + 1.0) * thrust_single);
		kty = 1.0 / (4.0 * la * std::cos(arm_ang) * thrust_single);
		ktz = 1.0 / (p_->motor_num() * p_->motor_drag_max());

		success = true;
	} else {
		ROS_ERROR("Error: MantisSolver::calculate_thrust_coeffs(): unknown mixer type");
	}

	return success;
}
