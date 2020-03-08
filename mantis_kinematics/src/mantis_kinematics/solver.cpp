/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>

#include <mantis_kinematics/solver.h>

#include <mantis_description/se_tools.h>
//#include <mantis_kinematics/dynamics/calc_Dq.h>
//#include <mantis_kinematics/dynamics/calc_Cqqd.h>
//#include <mantis_kinematics/dynamics/calc_Lqd.h>
//#include <mantis_kinematics/dynamics/calc_Jj2.h>
//#include <mantis_kinematics/dynamics/calc_Je.h>

#include <string>

MantisSolver::MantisSolver( MantisParams::Client& p, MantisState::Client& s )
	: p_( p )
	, s_( s )
	, param_load_time_( 0 )
	, state_load_time_( 0 ) {
	// check_description();
}

MantisSolver::~MantisSolver() {
}

bool MantisSolver::check_description( void ) {
	bool success = true;

	success &= check_parameters();
	success &= check_state();

	if ( !success ) {
		ROS_ERROR_THROTTLE( 2.0, "MantisSolver::check_description(): Unable to "
								 "validate params and state" );
	}

	return success;
}

bool MantisSolver::check_parameters( void ) {
	bool success = true;

	// Ensures the parameters are ready
	if ( p_.ok() ) {
		// Make sure our local variables are up to date
		if ( p_.get(MantisParams::PARAM_TIME_UPDATED) != param_load_time_ ) {
			success &= load_parameters();
		}
	} else {
		success = false;
	}

	return success;
}

bool MantisSolver::check_state( void ) {
	bool success = true;

	// Ensure the state are is ready, and that it matches the parameter
	// configuration
	if ( s_.ok() ) {
		// Make sure our local variables are up to date
		if ( s_.time_updated() != state_load_time_ ) {
			success &= load_state();
		}
	} else {
		success = false;
	}

	return success;
}

// TODO: Could use this to dynamically load in the dynamics solver
bool MantisSolver::load_parameters( void ) {
	bool success = true;

	// Load in the link definitions
	manip_.reset();
	manip_ref_.reset();

	for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM); i++ ) {
		if ( !manip_.add_joint( p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i) ) ) {
			ROS_FATAL( "Error loading joint %i", int( i ) );
			success = false;
			break;
		}

		if ( !manip_ref_.add_joint( p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i) ) ) {
			ROS_FATAL( "Error loading ref joint %i", int( i ) );
			success = false;
			break;
		}
	}

	for ( int i = 0; i < p_.get(MantisParams::PARAM_BODY_NUM); i++ ) {
		if ( !manip_.add_body( p_.get(MantisParams::PARAM_BODY_INERTIAL, i).com, i - 1 ) ) {
			ROS_FATAL( "Error loading joint %i", int( i ) );
			success = false;
			break;
		}

		if ( !manip_ref_.add_body( p_.get(MantisParams::PARAM_BODY_INERTIAL, i).com, i - 1 ) ) {
			ROS_FATAL( "Error loading ref joint %i", int( i ) );
			success = false;
			break;
		}
	}

	// Invalidate the current state as the system may have changed
	state_load_time_ = ros::Time( 0 );

	if ( success ) {
		param_load_time_ = p_.get(MantisParams::PARAM_TIME_UPDATED);
	}

	return success;
}

bool MantisSolver::load_state( void ) {
	bool success = true;

	Eigen::VectorXd r = s_.r();
	Eigen::VectorXd rd = s_.rd();

	int jc = 0;
	for ( int i = 0; i < manip_.num_joints(); i++ ) {
		if ( manip_.joint( i ).jt() != DHParameters::JointType::Static ) {
			manip_.joint( i ).update( r[jc], rd[jc] );
			jc++;
		}
	}

	state_load_time_ = s_.time_updated();

	return success;
}

bool MantisSolver::load_ref( const Eigen::VectorXd& r,
	const Eigen::VectorXd& rd ) {
	bool success = false;

	if ( ( r.size() == rd.size() ) && ( r.size() <= manip_ref_.num_joints() ) ) {
		int jc = 0;
		for ( int i = 0; i < manip_ref_.num_joints(); i++ ) {
			if ( manip_ref_.joint( i ).jt() != DHParameters::JointType::Static ) {
				manip_ref_.joint( i ).update( r[jc], rd[jc] );
				jc++;
			}
		}

		success = true;
	}

	return success;
}

int MantisSolver::num_states( void ) {

	if ( !check_parameters() )
		ROS_ERROR_THROTTLE( 2.0, "MantisSolver::num_states(): parameters no ok" );

	return 6 + p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC);
}

bool MantisSolver::calculate_mass_matrix( Eigen::MatrixXd& Dq ) {
	bool success = false;

	if ( check_description() ) {
		// Prepare required matricies
		const unsigned int nj = p_.get(MantisParams::PARAM_JOINT_NUM_DYNAMIC);
		Dq = Eigen::MatrixXd::Zero( num_states(), num_states() );
		// Eigen::MatrixXd M_A_A = Eigen::MatrixXd::Zero(6,6);
		// M_A_A = MDTools::full_inertial(p_.body_inertial(0));
		Eigen::MatrixXd M_A_A = MDTools::full_inertial( p_.get(MantisParams::PARAM_BODY_INERTIAL, 0) );
		Eigen::MatrixXd M_A_J = Eigen::MatrixXd::Zero( 6, nj );
		//Eigen::MatrixXd M_J_A = Eigen::MatrixXd::Zero( nj, 6 );
		Eigen::MatrixXd M_J_J = Eigen::MatrixXd::Zero( nj, nj );

		for ( int i = 1; i < p_.get(MantisParams::PARAM_BODY_NUM); i++ ) {
			Eigen::Affine3d gbic;
			Eigen::MatrixXd Abic;
			manip_.calculate_g_body( gbic, i );
			Abic = MDTools::adjoint( gbic.inverse() );

			Eigen::MatrixXd Jbic = Eigen::MatrixXd::Zero( 6, nj );
			Eigen::MatrixXd Jbic_calc;
			manip_.calculate_J_body( Jbic_calc, i );
			std::vector<bool> rmap;
			manip_.get_dynamic_joint_map( rmap );

			ROS_ASSERT_MSG( Jbic_calc.cols() <= rmap.size(),
				"Jacobian is larger than joint map" );

			int colc = 0;
			for ( int j = 0; j < Jbic_calc.cols(); j++ ) {
				// If the index represents a dynamic joint
				if ( rmap[j] ) {
					Jbic.col( colc ) = Jbic_calc.col( j );
					colc++;
				}
			}

			// ROS_INFO_STREAM("Jbic:\n" << Jbic);

			Eigen::MatrixXd Mi = MDTools::full_inertial( p_.get(MantisParams::PARAM_BODY_INERTIAL, i) );

			M_A_A += Abic.transpose() * Mi * Abic;
			M_A_J += Abic.transpose() * Mi * Jbic;
			//M_J_A += Jbic.transpose() * Mi * Abic;
			M_J_J += Jbic.transpose() * Mi * Jbic;
		}

		Dq << M_A_A, M_A_J, M_A_J.transpose(), M_J_J;

		success = true;
	}

	return success;
}

bool MantisSolver::calculate_coriolis_matrix( Eigen::MatrixXd& Cqqd ) {
	ROS_WARN_ONCE(
		"MantisSolver::calculate_coriolis_matrix() not implemented correctly" );
	Cqqd = Eigen::MatrixXd::Zero( num_states(), num_states() );

	/* XXX: Old method
  //Need to copy this data to access it directly
  Eigen::VectorXd r = s_.r();
  Eigen::VectorXd rd = s_.rd();
  Eigen::Vector3d bw = s_.bw();
  Eigen::Vector3d bv = s_.bv();
  double l0 = manip_.joint(0).d();
  double l1 = manip_.joint(1).r();

  calc_Cqqd(Cqqd,
                    p_.body_inertial(1).Ixx, p_.body_inertial(1).Iyy,
                    p_.body_inertial(2).Ixx, p_.body_inertial(2).Iyy,
                    bv(0), bv(1), bv(2), bw(0), bw(1), bw(2),
                    l0, l1, p_.body_inertial(1).com, p_.body_inertial(2).com,
                    p_.body_inertial(1).mass, p_.body_inertial(2).mass,
                    r(0), rd(0), r(1), rd(1));
  */

	return true;
}

// XXX: TODO: Not implemented
bool MantisSolver::calculate_loss_matrix( Eigen::MatrixXd& Lqd ) {
	ROS_WARN_ONCE(
		"MantisSolver::calculate_loss_matrix() not implemented correctly" );
	Lqd = Eigen::MatrixXd::Zero( num_states(), num_states() );

	return true;
}

bool MantisSolver::solve_inverse_dynamics( Eigen::VectorXd& tau,
	const Eigen::VectorXd& ua ) {
	bool success = false;

	if ( check_description() ) {

		Eigen::VectorXd qd( num_states() );
		qd << s_.bv(), s_.bw(), s_.rd();

		if ( ( ua.size() == qd.size() ) && ( tau.size() == qd.size() ) && ( num_states() == qd.size() ) ) {
			// Calculate dynamics matricies
			Eigen::MatrixXd D;
			Eigen::MatrixXd C;
			Eigen::MatrixXd L;

			if ( calculate_mass_matrix( D ) && calculate_coriolis_matrix( C ) && calculate_loss_matrix( L ) ) {
				tau = D * ua + ( C + L ) * qd;

				success = true;
			} else {
				ROS_ERROR_THROTTLE( 2.0, "MantisSolver::solve_inverse_dynamics(): Could "
										 "not calculate dynamics matricies" );
			}
		} else {
			ROS_ERROR_THROTTLE( 2.0, "MantisSolver::solve_inverse_dynamics(): Vector "
									 "lengths inconsistent" );
		}
	}

	return success;
}

bool MantisSolver::calculate_thrust_coeffs( double& kT, double& ktx, double& kty,
	double& ktz ) {
	bool success = false;

	if ( check_parameters() ) {
		// XXX: TODO: Had to disable due to bad values
		// double rpm_max = p_.motor_kv() * s_.voltage();	//Get the theoretical
		// maximum rpm at the current battery voltage
		// double thrust_single = p_.rpm_thrust_m() * rpm_max + p_.rpm_thrust_c();
		// //Use the RPM to calculate maximum thrust
		// double thrust_single = 0.8*9.80665;
		double thrust_single = p_.get(MantisParams::PARAM_MOTOR_MAX_THRUST);

		switch( p_.get(MantisParams::PARAM_AIRFRAME_TYPE) ) {
			case MantisParams:: PARAM_AIRFRAME_TYPE_QUAD_X4: {
				double arm_ang = M_PI / 4.0; // 45Deg from forward to arm rotation
				double la = p_.get(MantisParams::PARAM_BASE_ARM_LENGTH);
				kT = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * thrust_single );
				ktx = 1.0 / ( 4.0 * la * std::sin( arm_ang ) * thrust_single );
				kty = ktx; // Airframe is symmetric
				ktz = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * p_.get(MantisParams::PARAM_MOTOR_MAX_DRAG) );

				success = true;
				break;
			}
			case MantisParams:: PARAM_AIRFRAME_TYPE_QUAD_P4: {
				// XXX: UNTESTED!
				// double arm_ang = M_PI / 4.0; //45Deg from forward to arm rotation
				double la = p_.get(MantisParams::PARAM_BASE_ARM_LENGTH);
				kT = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * thrust_single );
				ktx = 1.0 / ( 2.0 * la * thrust_single );
				kty = ktx; // Airframe is symmetric
				ktz = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * p_.get(MantisParams::PARAM_MOTOR_MAX_DRAG) );

				success = true;
				break;
			}
			case MantisParams:: PARAM_AIRFRAME_TYPE_HEX_X6: {
				double arm_ang = ( M_PI / 6.0 ); // 30Deg from forward to arm rotation
				double la = p_.get(MantisParams::PARAM_BASE_ARM_LENGTH);
				kT = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * thrust_single );
				ktx = 1.0 / ( 2.0 * la * ( 2.0 * std::sin( arm_ang ) + 1.0 ) * thrust_single );
				kty = 1.0 / ( 4.0 * la * std::cos( arm_ang ) * thrust_single );
				ktz = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * p_.get(MantisParams::PARAM_MOTOR_MAX_DRAG) );

				success = true;
				break;
			}
			case MantisParams:: PARAM_AIRFRAME_TYPE_HEX_P6: {
				// XXX: UNTESTED!
				double arm_ang = ( M_PI / 3.0 ); // 30Deg from forward to arm rotation
				double la = p_.get(MantisParams::PARAM_BASE_ARM_LENGTH);
				kT = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * thrust_single );
				ktx = 1.0 / ( 4.0 * la * std::sin( arm_ang ) * thrust_single );
				kty = 1.0 / ( 2.0 * la * ( 2.0 * std::cos( arm_ang ) + 1.0 ) * thrust_single );
				ktz = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * p_.get(MantisParams::PARAM_MOTOR_MAX_DRAG) );

				success = true;
				break;
			}
			case MantisParams:: PARAM_AIRFRAME_TYPE_OCTO_X8: {
				// XXX: UNTESTED!
				double arm_ang = ( M_PI / 8.0 ); // 22.5Deg from forward to arm rotation
				double arm_ang_2 = ( M_PI / 2.0 ) - arm_ang; // 77.5Deg from forward to arm rotation
				double la = p_.get(MantisParams::PARAM_BASE_ARM_LENGTH);
				kT = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * thrust_single );
				ktx = 1.0 / ( 4.0 * la * ( std::sin( arm_ang ) + std::sin( arm_ang_2 ) ) * thrust_single );
				kty = ktx; // Airframe is symmetric
				ktz = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * p_.get(MantisParams::PARAM_MOTOR_MAX_DRAG) );

				success = true;
				break;
			}
			case MantisParams:: PARAM_AIRFRAME_TYPE_OCTO_P8: {
				// XXX: UNTESTED!
				double arm_ang = ( M_PI / 4.0 ); // 45Deg from forward to arm rotation
				double la = p_.get(MantisParams::PARAM_BASE_ARM_LENGTH);
				kT = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * thrust_single );
				ktx = 1.0 / ( 2.0 * la * ( 2.0 * std::sin( arm_ang ) + 1.0 ) * thrust_single );
				kty = ktx; // Airframe is symmetric
				ktz = 1.0 / ( p_.get(MantisParams::PARAM_MOTOR_NUM) * p_.get(MantisParams::PARAM_MOTOR_MAX_DRAG) );

				success = true;
				break;
			}
			default: {
				ROS_ERROR_THROTTLE( 2.0,
									"MantisSolver::calculate_thrust_coeffs(): unknown airframe type (%s)",
									p_.get(MantisParams::PARAM_AIRFRAME_NAME).c_str() );
				break;
			}
		}
	}

	return success;
}

bool MantisSolver::calculate_vbx_int( Eigen::VectorXd& vbx, SerialManipulator& m,
	const unsigned int x ) {

	bool success = false;

	if ( check_description() ) {
		Eigen::MatrixXd Jbx;
		Eigen::VectorXd qdx;
		m.calculate_Jxy( Jbx, 0, x );
		m.get_qdxn( qdx, 0, x );
		vbx = Jbx * qdx;

		success = true;
	}

	return success;
}

bool MantisSolver::calculate_gxy_int( Eigen::Affine3d& g, SerialManipulator& m,
	const unsigned int x,
	const unsigned int y ) {
	bool success = false;

	if ( check_description() ) {
		if ( ( x <= m.num_joints() ) && ( y <= m.num_joints() ) ) {

			m.calculate_gxy( g, x, y );

			success = true;
		} else {
			std::string reason;
			if ( x > m.num_joints() ) {
				reason = "start frame > total frames";
			} else if ( y > m.num_joints() ) {
				reason = "end frame > total frames";
			} else {
				reason = "undefined error";
			}

			ROS_ERROR_THROTTLE( 2.0, "MantisSolver::calculate_gxy(): frame inputs; %s",
				reason.c_str() );
		}
	}

	return success;
}

bool MantisSolver::calculate_gbe( Eigen::Affine3d& gbe ) {
	return calculate_gxy_int( gbe, manip_, 0, manip_.num_joints() );
}

bool MantisSolver::calculate_vbx( Eigen::VectorXd& vbx, const unsigned int n ) {
	return calculate_vbx_int( vbx, manip_, n );
}

bool MantisSolver::calculate_vbe( Eigen::VectorXd& vbe ) {
	return calculate_vbx_int( vbe, manip_, manip_.num_joints() );
}

bool MantisSolver::calculate_gxy( Eigen::Affine3d& g, const unsigned int x,
	const unsigned int y ) {
	return calculate_gxy_int( g, manip_, x, y );
}

bool MantisSolver::calculate_gbe_ref( Eigen::Affine3d& gbe ) {
	return calculate_gxy_int( gbe, manip_ref_, 0, manip_ref_.num_joints() );
}

bool MantisSolver::calculate_vbx_ref( Eigen::VectorXd& vbx,
	const unsigned int n ) {
	return calculate_vbx_int( vbx, manip_ref_, n );
}

bool MantisSolver::calculate_vbe_ref( Eigen::VectorXd& vbe ) {
	return calculate_vbx_int( vbe, manip_ref_, manip_ref_.num_joints() );
}

bool MantisSolver::calculate_gxy_ref( Eigen::Affine3d& g, const unsigned int x,
	const unsigned int y ) {
	return calculate_gxy_int( g, manip_ref_, x, y );
}
