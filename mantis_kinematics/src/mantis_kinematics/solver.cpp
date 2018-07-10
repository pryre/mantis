#include <eigen3/Eigen/Dense>

#include <ros/ros.h>

#include <mantis_kinematics/solver.h>

#include <mantis_description/se_tools.h>
#include <mantis_kinematics/dynamics/calc_Dq.h>
#include <mantis_kinematics/dynamics/calc_Cqqd.h>
//#include <mantis_kinematics/dynamics/calc_Lqd.h>
//#include <mantis_kinematics/dynamics/calc_Jj2.h>
//#include <mantis_kinematics/dynamics/calc_Je.h>

#include <string>


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
		ROS_ERROR_THROTTLE(2.0, "Unable to validate params and state");
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
	manip_.reset();

	for(int i=0; i<p_->get_joint_num(); i++) {
		if( !manip_.add_joint( p_->joint(i) ) ) {
			ROS_FATAL("Error loading joint %i", int(i));
			success = false;
			break;
		}
	}

	for(int i=0; i<p_->get_body_num(); i++) {
		if( !manip_.add_body( p_->body_inertial(i).com, i-1 ) ) {
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
		for(int i=0; i<manip_.num_joints(); i++) {
			if(manip_.joint(i).jt() != DHParameters::JointType::Static) {
				manip_.joint(i).update(r[jc], rd[jc]);
				jc++;
			}
		}
	} else {
		ROS_ERROR_THROTTLE(2.0, "Error: MantisSolver::load_state(): Dynamic joint sizes are inconsistent");
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

bool MantisSolver::calculate_mass_matrix( Eigen::MatrixXd &Dq ) {
	//XXX: Take care!
	Eigen::MatrixXd Dq_old = Eigen::MatrixXd::Zero(num_states(), num_states());

	Eigen::VectorXd r = s_->r();
	double l0 = manip_.joint(0).d();
	double l1 = manip_.joint(1).r();

	calc_Dq(Dq_old,
			p_->body_inertial(0).Ixx, p_->body_inertial(0).Iyy, p_->body_inertial(0).Izz,
			p_->body_inertial(1).Ixx, p_->body_inertial(1).Iyy, p_->body_inertial(1).Izz,
			p_->body_inertial(2).Ixx, p_->body_inertial(2).Iyy, p_->body_inertial(2).Izz,
			l0, l1, p_->body_inertial(1).com, p_->body_inertial(2).com,
			p_->body_inertial(0).mass, p_->body_inertial(1).mass, p_->body_inertial(2).mass,
			r(0), r(1));
	//XXX: Take care!

	//Prepare required matricies
	const unsigned int nj = p_->get_dynamic_joint_num();
	Dq = Eigen::MatrixXd::Zero(num_states(), num_states());
	Eigen::MatrixXd M_A_A = Eigen::MatrixXd::Zero(6,6);
	Eigen::MatrixXd M_A_J = Eigen::MatrixXd::Zero(6,nj);
	Eigen::MatrixXd M_J_A = Eigen::MatrixXd::Zero(nj,6);
	Eigen::MatrixXd M_J_J = Eigen::MatrixXd::Zero(nj,nj);

	M_A_A = full_inertial(p_->body_inertial(0));

	for(int i=1;i<p_->get_body_num();i++) {
		Eigen::Affine3d gbic;
		//Eigen::Affine3d gt = Eigen::Affine3d::Identity();
		Eigen::MatrixXd Abic;
		manip_.calculate_g_body(gbic,i);
		//gt.linear() = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()).toRotationMatrix();
		//gbic = gt*gbic;
		Abic = adjoint(gbic.inverse());

		Eigen::MatrixXd Jbic = Eigen::MatrixXd::Zero(6,nj);
		Eigen::MatrixXd Jbic_calc;
		manip_.calculate_J_body(Jbic_calc,i);
		std::vector<bool> rmap;
		manip_.get_dynamic_joint_map(rmap);

		ROS_ASSERT_MSG(Jbic_calc.cols()<=rmap.size(), "Jacobian is larger than joint map");

		int colc = 0;
		for(int j=0; j<Jbic_calc.cols(); j++) {
			//If the index represents a dynamic joint
			if(rmap[j]) {
				Jbic.col(colc) = Jbic_calc.col(j);
				colc++;
			}
		}

		Eigen::MatrixXd Mi = full_inertial(p_->body_inertial(i));

		M_A_A += Abic.transpose()*Mi*Abic;
		M_A_J += Abic.transpose()*Mi*Jbic;
		M_J_A += Jbic.transpose()*Mi*Abic;
		M_J_J += Jbic.transpose()*Mi*Jbic;
	}

	Dq << M_A_A, M_A_J,
		  M_J_A, M_J_J;

	//ROS_INFO_STREAM("Dq_old:\n" << Dq_old);
	//ROS_INFO_STREAM("Dq:\n" << Dq);

	//Dq = Dq_old;
}

bool MantisSolver::calculate_coriolis_matrix( Eigen::MatrixXd &Cqqd ) {
	ROS_WARN_ONCE("MantisSolver::calculate_coriolis_matrix() not implemented correctly");

	Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_states(), num_states());
	//Need to copy this data to access it directly
	Eigen::VectorXd r = s_->r();
	Eigen::VectorXd rd = s_->rd();
	Eigen::Vector3d bw = s_->bw();
	Eigen::Vector3d bv = s_->bv();
	double l0 = manip_.joint(0).d();
	double l1 = manip_.joint(1).r();

	calc_Cqqd(C,
			  p_->body_inertial(1).Ixx, p_->body_inertial(1).Iyy,
			  p_->body_inertial(2).Ixx, p_->body_inertial(2).Iyy,
			  bv(0), bv(1), bv(2), bw(0), bw(1), bw(2),
			  l0, l1, p_->body_inertial(1).com, p_->body_inertial(2).com,
			  p_->body_inertial(1).mass, p_->body_inertial(2).mass,
			  r(0), rd(0), r(1), rd(1));

	return true;
}

//XXX: TODO: Not implemented
bool MantisSolver::calculate_loss_matrix( Eigen::MatrixXd &Lqd ) {
	ROS_WARN_ONCE("MantisSolver::calculate_loss_matrix() not implemented correctly");
	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(num_states(), num_states());

	return true;
}

bool MantisSolver::solve_inverse_dynamics( Eigen::VectorXd &tau, const Eigen::VectorXd &ua ) {
	bool success = false;

	if( check_description() ) {

		Eigen::VectorXd qd(num_states());
		qd << s_->bv(), s_->bw(), s_->rd();

		if( ( ua.size() == qd.size() ) && ( tau.size() == qd.size() ) && ( num_states() == qd.size() ) ) {
			//Calculate dynamics matricies
			Eigen::MatrixXd D;
			Eigen::MatrixXd C;
			Eigen::MatrixXd L;

			if( calculate_mass_matrix(D) ) {// && calculate_coriolis_matrix(C) && calculate_loss_matrix(L) ) {
				tau = D*ua;// + (C + L)*qd;

				success = true;
			} else {
				ROS_ERROR_THROTTLE(2.0, "Error: MantisSolver::solve_inverse_dynamics(): Could not calculate dynamics matricies");
			}
		} else {
			ROS_ERROR_THROTTLE(2.0, "Error: MantisSolver::solve_inverse_dynamics(): Vector lengths inconsistent");
		}
	}

	return success;
}

bool MantisSolver::calculate_vbe( Eigen::VectorXd &vbe ) {
	return calculate_vbx(vbe, manip_.num_joints());
}

bool MantisSolver::calculate_vbx( Eigen::VectorXd &vbx, const unsigned int x ) {

	bool success = false;

	if( check_description() ) {
		//Eigen::MatrixXd Je = Eigen::MatrixXd::Zero(6, s_->rd().size());
		/*
		calc_Je(Je,
			manip_.joint(1).r(),
			manip_.joint(2).r(),
			manip_.joint(2).q());


		vbx = Je*s_->rd();//manip_.qd();
		*/
		Eigen::MatrixXd Jbx;
		Eigen::VectorXd qdx;
		manip_.calculate_Jxy( Jbx, 0, x );
		manip_.get_qdxn(qdx, 0, x);
		vbx = Jbx*qdx;
		//ROS_INFO_STREAM("Je_old:\n" << Je);
		//ROS_INFO_STREAM("Je_new:\n" << Je_new);

		success = true;
	}

	return success;
}
/*
bool MantisSolver::calculate_dynamic_Je( Eigen::MatrixXd &Je ) {
	bool success = false;
	Je = Eigen::MatrixXd::Zero(6, s_->rd().size());

	if( check_description() ) {
		calc_Je(Je,
			manip_.joint(1).r(),
			manip_.joint(2).r(),
			manip_.joint(2).q());

		success = true;
	}

	Eigen::MatrixXd Je_new;


	ROS_INFO_STREAM("Je_old:\n" << Je);
	ROS_INFO_STREAM("Je_new:\n" << Je_new);

	return success;
}
*/
bool MantisSolver::calculate_gbe( Eigen::Affine3d &gbe ) {
	/*
	bool success = false;

	gbe = Eigen::Affine3d::Identity();

	if( check_description() ) {
		for(int i=0; i<joints_.size(); i++) {
			gbe = gbe * joints_[i].transform();
		}

		success = true;
	}
	*/

	return calculate_gxy( gbe, 0, manip_.num_joints() );;
}

bool MantisSolver::calculate_gxy( Eigen::Affine3d &g, const unsigned int x, const unsigned int y ) {
	bool success = false;

	if( check_description() ) {
		if( ( x <= manip_.num_joints() ) && ( y <= manip_.num_joints() ) ) {

			manip_.calculate_gxy(g,x,y);

			success = true;
		} else {
			std::string reason;
			if( x > manip_.num_joints() ) {
				reason = "start frame > total frames";
			} else if( y > manip_.num_joints() ) {
				reason = "end frame > total frames";
			} else {
				reason = "undefined error";
			}

			ROS_ERROR_THROTTLE(2.0, "Error: MantisSolver::calculate_gxy(): frame inputs; %s", reason.c_str());
		}
	}

	return success;
}

bool MantisSolver::calculate_thrust_coeffs( double &kT, double &ktx, double &kty, double &ktz) {
	bool success = false;

	//XXX: TODO: Had to disable due to bad values
	double rpm_max = p_->motor_kv() * s_->voltage();	//Get the theoretical maximum rpm at the current battery voltage
	double thrust_single = p_->rpm_thrust_m() * rpm_max + p_->rpm_thrust_c();	//Use the RPM to calculate maximum thrust
	//double thrust_single = 0.8*9.80665;

	if(p_->airframe_type() == "quad_x4") {
		double arm_ang = M_PI / 4.0; //45Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (4.0 * la * std::sin(arm_ang) * thrust_single);
		kty = ktx; //Airframe is symmetric
		ktz = 1.0 / (p_->motor_num() * p_->motor_drag_max());

		success = true;
	} else if(p_->airframe_type() == "quad_p4") {
		//XXX: UNTESTED!
		//double arm_ang = M_PI / 4.0; //45Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (2.0 * la * thrust_single);
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
	} else if(p_->airframe_type() == "hex_p6") {
		//XXX: UNTESTED!
		double arm_ang = (M_PI / 3.0); //30Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (4.0 * la * std::sin(arm_ang) * thrust_single);
		kty = 1.0 / (2.0 * la * (2.0 * std::cos(arm_ang) + 1.0) * thrust_single);
		ktz = 1.0 / (p_->motor_num() * p_->motor_drag_max());

		success = true;
	} else if(p_->airframe_type() == "octo_x8") {
		//XXX: UNTESTED!
		double arm_ang = (M_PI / 8.0); //22.5Deg from forward to arm rotation
		double arm_ang_2 = (M_PI / 2.0) - arm_ang; //77.5Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (4.0 * la * (std::sin(arm_ang) + std::sin(arm_ang_2)) * thrust_single);
		kty = ktx; //Airframe is symmetric
		ktz = 1.0 / (p_->motor_num() * p_->motor_drag_max());

		success = true;
	} else if(p_->airframe_type() == "octo_p8") {
		//XXX: UNTESTED!
		double arm_ang = (M_PI / 4.0); //45Deg from forward to arm rotation
		double la = p_->base_arm_length();
		kT = 1.0 / (p_->motor_num() * thrust_single);
		ktx = 1.0 / (2.0 * la * (2.0 * std::sin(arm_ang) + 1.0) * thrust_single);
		kty = ktx; //Airframe is symmetric
		ktz = 1.0 / (p_->motor_num() * p_->motor_drag_max());

		success = true;
	} else {
		ROS_ERROR_THROTTLE(2.0, "Error: MantisSolver::calculate_thrust_coeffs(): unknown mixer type");
	}

	return success;
}
