#pragma once

#include <ros/ros.h>
#include <mantis_description/param_client.h>
#include <mantis_state/state_client.h>

#include <dh_parameters/dh_parameters.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <string>

class MantisSolver {
	private:
		MantisParamClient *p_;
		MantisStateClient *s_;

		ros::Time param_load_time_;
		ros::Time state_load_time_;

		std::vector<DHParameters,Eigen::aligned_allocator<DHParameters> > joints_;
		Eigen::MatrixXd mixer_;
		std::string mixer_name_;

	public:
		MantisSolver( MantisParamClient *p, MantisStateClient *s );

		~MantisSolver( void );

		int num_states( void );

		bool solve_inverse_dynamics( Eigen::VectorXd &tau, const Eigen::VectorXd &ua );
		bool calculate_Je( Eigen::MatrixXd &Je );
		bool calculate_gbe( Eigen::Affine3d &gbe );
		bool calculate_gxy( Eigen::Affine3d &g, const unsigned int x, const unsigned int y );

		const Eigen::MatrixXd& get_mixer( void );
		bool calculate_thrust_coeffs( double &kT, double &ktx, double &kty, double &ktz);

	private:
		bool load_parameters( void );
		bool load_state( void );

		bool check_description( void );
		bool check_parameters( void );
		bool check_state( void );
};
