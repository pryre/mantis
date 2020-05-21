/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include <mantis_description/mixer_maps.h>

//==-- The Mantis Description Tools namespace
namespace MDTools {
int mixer_generate_quad_x4( Eigen::MatrixXd& m ) {
	int num_motors = 4;

	m = Eigen::MatrixXd( num_motors, 4 );
	m << 1.0, -1.0, -1.0, -1.0, // Motor 1
		1.0, 1.0, 1.0, -1.0, // Motor 2
		1.0, 1.0, -1.0, 1.0, // Motor 3
		1.0, -1.0, 1.0, 1.0; // Motor 4

	return num_motors;
}

int mixer_generate_quad_p4( Eigen::MatrixXd& m ) {
	int num_motors = 4;

	m = Eigen::MatrixXd( num_motors, 4 );
	m << 1.0, -1.0, 0.0, -1.0, // Motor 1
		1.0, 1.0, 0.0, -1.0, // Motor 2
		1.0, 0.0, -1.0, 1.0, // Motor 3
		1.0, 0.0, 1.0, 1.0; // Motor 4

	return num_motors;
}

int mixer_generate_hex_x6( Eigen::MatrixXd& m ) {
	int num_motors = 6;

	m = Eigen::MatrixXd( num_motors, 4 );
	m << 1.0, -1.0, 0.0, 1.0, // Motor 1
		1.0, 1.0, 0.0, -1.0, // Motor 2
		1.0, 0.5, -1.0, 1.0, // Motor 3
		1.0, -0.5, 1.0, -1.0, // Motor 4
		1.0, -0.5, -1.0, -1.0, // Motor 5
		1.0, 0.5, 1.0, 1.0; // Motor 6

	return num_motors;
}

int mixer_generate_hex_p6( Eigen::MatrixXd& m ) {
	int num_motors = 6;

	m = Eigen::MatrixXd( num_motors, 4 );
	m << 1.0, 0.0, -1.0, 1.0, // Motor 1
		1.0, 0.0, 1.0, -1.0, // Motor 2
		1.0, 1.0, 0.5, 1.0, // Motor 3
		1.0, -1.0, -0.5, -1.0, // Motor 4
		1.0, 1.0, -0.5, -1.0, // Motor 5
		1.0, -1.0, 0.5, 1.0; // Motor 6

	return num_motors;
}

inline double deg2rad (double degrees) {
    static const double pi_on_180 = 4.0 * atan (1.0) / 180.0;
    return degrees * pi_on_180;
}

Eigen::Matrix3d rx( const double theta ) {
	const double c = cos(theta);
	const double s = sin(theta);

	Eigen::Matrix3d E;
	E << 1,  0,  0,
		 0,  c,  s,
		 0, -s,  c;
	return E;
}

Eigen::Matrix3d rz( const double theta ) {
	const double c = cos(theta);
	const double s = sin(theta);

	Eigen::Matrix3d E;
	E << c,  s,  0,
		-s,  c,  0,
		 0,  0,  1;
	return E;
}

Eigen::Matrix3d skew( const Eigen::Vector3d& in ) {
	Eigen::Matrix3d out;
	out <<      0, -in(2),  in(1),
	        in(2),      0, -in(0),
	       -in(1),  in(0),      0;
	return out;
}

//E,r --> X
Eigen::MatrixXd plux( const Eigen::Matrix3d& E, const Eigen::Vector3d& r ) {
	Eigen::MatrixXd X( 6, 6 );

	X <<  E, Eigen::Matrix3d::Zero(),
		 -E*skew(r), E;

	return X;
}

int mixer_generate_hex_fa( Eigen::MatrixXd& m, const double la, const double alpha ) {
	int num_motors = 6;

	//each col is the map for one motor
	Eigen::MatrixXd Mbi = Eigen::MatrixXd::Zero( num_motors, 6 );

	double d = 1; // Rotor Spin Direction

	// Motor map is given as angular position to rotor
	Eigen::MatrixXd motor_map( num_motors, 3 );

	motor_map << deg2rad(270), alpha, d,	//1
				  deg2rad(90),-alpha,-d,	//2
			      deg2rad(30), alpha, d,	//3
				 deg2rad(210),-alpha,-d,	//4
				 deg2rad(330),-alpha,-d,	//5
				 deg2rad(150), alpha, d;	//6

    // Torquing Forces
    for(int i=0; i<6; i++) {
        double mt = motor_map(i,0);	//theta
        double ma = motor_map(i,1);	//phi
        double md = motor_map(i,2);	//delta

        // Constructed from A->B
        Eigen::Matrix3d REr = rx(ma)*rz(mt);

        // Use a 'displacement' rotation to calculate the rotor position
        // in frame A (i.e. R = E')
		Eigen::Matrix3d Rr = rz(mt);
		Rr.transposeInPlace();
		Eigen::Vector3d r = Rr*Eigen::Vector3d(la, 0.0, 0.0);
		r << la, 0, 0;

        Eigen::MatrixXd X = plux(REr, r);

		//T_max = 1.0; D_max = 1.0;
		Eigen::VectorXd v(6);
		v << 0, 0, md, 0, 0, 1.0;

		X.transposeInPlace();
        Mbi.block(0,i,6,1) << X*v;
	}

	Eigen::FullPivLU<Eigen::MatrixXd> lu(Mbi);

	ROS_ASSERT_MSG(lu.rank() == 6, "Configuration is not fully-actuated and motor-map cannot be calculated!");

	Eigen::MatrixXd M = lu.inverse();

	//Have to switch the blocks to have [T,tau] not [tau,T]
	m = Eigen::MatrixXd(6,6);
    m.block(0,0,6,3) << M.block(0,3,6,3);
    m.block(0,3,6,3) << M.block(0,0,6,3);

	return num_motors;
}

int mixer_generate_octo_x8( Eigen::MatrixXd& m ) {
	int num_motors = 8;

	m = Eigen::MatrixXd( num_motors, 4 );
	m << 1.0, -1.0, -1.0, 1.0, // Motor 1
		1.0, 1.0, 1.0, 1.0, // Motor 2
		1.0, -1.0, -1.0, -1.0, // Motor 3
		1.0, -1.0, 1.0, -1.0, // Motor 4
		1.0, 1.0, -1.0, -1.0, // Motor 5
		1.0, 1.0, 1.0, -1.0; // Motor 6
	1.0, 1.0, -1.0, 1.0; // Motor 7
	1.0, -1.0, 1.0, 1.0; // Motor 8

	return num_motors;
}

int mixer_generate_octo_p8( Eigen::MatrixXd& m ) {
	int num_motors = 8;

	m = Eigen::MatrixXd( num_motors, 4 );
	m << 1.0, 0.0, -1.0, 1.0, // Motor 1
		1.0, 0.0, 1.0, 1.0, // Motor 2
		1.0, -1.0, -1.0, -1.0, // Motor 3
		1.0, -1.0, 1.0, -1.0, // Motor 4
		1.0, 1.0, -1.0, -1.0, // Motor 5
		1.0, 1.0, 1.0, -1.0; // Motor 6
	1.0, 1.0, 0.0, 1.0; // Motor 7
	1.0, -1.0, 0.0, 1.0; // Motor 8

	return num_motors;
}
}
