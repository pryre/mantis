#pragma once

#include <ros/ros.h>

#include <dh_parameters/dh_parameters.h>

#include <eigen3/Eigen/Dense>

class ControllerIDState {
	public:
		int num_;
		Eigen::VectorXd qd_;

		Eigen::Affine3d g_;
		Eigen::Vector3d bv_;
		Eigen::Vector3d bw_;

		Eigen::VectorXd r_;
		Eigen::VectorXd rd_;
		Eigen::VectorXd rdd_;

		double voltage_;

		bool mav_armed_;

		// High Level control status and setpoints
		bool high_level_control_ready_;
		Eigen::Affine3d g_sp_;
		Eigen::Vector3d gv_sp_;
		Eigen::Vector3d a_sp_;

	public:
		ControllerIDState( void );

		~ControllerIDState( void );

		void init( const int num_manipulator_links );

		void update_g( const Eigen::Affine3d &g );
		void update_bw( const Eigen::Vector3d &bw );
		void update_bv( const Eigen::Vector3d &bv );
		void update_r( const Eigen::VectorXd &r );
		void update_rd( const Eigen::VectorXd &rd );
		void update_rdd( const Eigen::VectorXd &rdd );
		void update_voltage( const double voltage );
		void update_status_armed( const bool armed );
		void update_status_hl_control( const bool ready );

		void update_g_sp( const Eigen::Affine3d &g_sp );
		void update_gv_sp( const Eigen::Vector3d &gv_sp );
		void update_a_sp( const Eigen::Vector3d &a_sp );

		int num( void );
		Eigen::Affine3d g( void );
		Eigen::Vector3d bw( void );
		Eigen::Vector3d bv( void );
		Eigen::VectorXd r( void );
		Eigen::VectorXd rd( void );
		Eigen::VectorXd rdd( void );
		Eigen::VectorXd qd( void );
		double voltage( void );
		bool status_armed( void );
		bool status_hl_control( void );

		Eigen::Affine3d g_sp( void );
		Eigen::Vector3d gv_sp( void );
		Eigen::Vector3d a_sp( void );

	//private:
};
