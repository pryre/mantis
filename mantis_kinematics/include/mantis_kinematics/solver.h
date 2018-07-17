#pragma once

#include <ros/ros.h>
#include <mantis_description/param_client.h>
#include <mantis_state/state_client.h>

#include <dh_parameters/serial_manipulator.h>

#include <eigen3/Eigen/Dense>

#include <string>

class MantisSolver {
	private:
		MantisParamClient& p_;
		MantisStateClient& s_;

		ros::Time param_load_time_;	//XXX: Could use the config/param stamps, but everything needs to be updated regardless
		ros::Time state_load_time_;

		SerialManipulator manip_;

	public:
		MantisSolver( MantisParamClient& p, MantisStateClient& s );

		~MantisSolver( void );

		int num_states( void );

		bool calculate_mass_matrix( Eigen::MatrixXd &Dq );
		bool calculate_coriolis_matrix( Eigen::MatrixXd &Cqqd ); //XXX: TODO: Not implemented properly
		bool calculate_loss_matrix( Eigen::MatrixXd &Lqd ); //XXX: TODO: Not implemented
		bool solve_inverse_dynamics( Eigen::VectorXd &tau, const Eigen::VectorXd &ua );

		bool calculate_vbe( Eigen::VectorXd &vbe );
		bool calculate_vbx( Eigen::VectorXd &vbx, const unsigned int n );

		bool calculate_gbe( Eigen::Affine3d &gbe );
		bool calculate_gxy( Eigen::Affine3d &g, const unsigned int x, const unsigned int y );

		bool calculate_thrust_coeffs( double &kT, double &ktx, double &kty, double &ktz);

	private:
		bool load_parameters( void );
		bool load_state( void );

		bool check_description( void );
		bool check_parameters( void );
		bool check_state( void );
};
