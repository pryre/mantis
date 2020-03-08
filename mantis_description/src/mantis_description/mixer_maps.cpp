/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

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
