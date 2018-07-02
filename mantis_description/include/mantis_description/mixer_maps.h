#include <eigen3/Eigen/Dense>

/*
All mixers return an m*4 map where:
	m is number of motors
	4 is the mapping of [Thrust, Roll, Pitch, Yaw] scalings

These maps can be multiplied against a attitude command vector [Thrust, Roll, Pitch, Yaw]'
to get the correct output:
	Eigen::Vector4d ua = Eigen::Vector4d::Zero();
	Eigen::MatrixXd = mixer;
	mixer_generate_quad_x(mixer);
	Eigen::VectorXd u = mixer*ua; //Where u is the normailized motor inputs for 'm' motors
*/

inline void mixer_generate_quad_x4(Eigen::MatrixXd &m) {
	m = Eigen::MatrixXd(4,4);
	m << 1.0, -1.0,	-1.0,	-1.0, // Motor 1
		 1.0,  1.0,	 1.0,	-1.0, // Motor 2
		 1.0,  1.0,	-1.0,	 1.0, // Motor 3
		 1.0, -1.0,	 1.0,	 1.0; // Motor 4
}

inline void mixer_generate_hex_x6(Eigen::MatrixXd &m) {
	m = Eigen::MatrixXd(6,4);
	m << 1.0, -1.0,	 0.0,	 1.0, // Motor 1
		 1.0,  1.0,	 0.0,	-1.0, // Motor 2
		 1.0,  0.5,	-1.0,	 1.0, // Motor 3
		 1.0, -0.5,	 1.0,	-1.0, // Motor 4
		 1.0, -0.5,	-1.0,	-1.0, // Motor 5
		 1.0,  0.5,	 1.0,	 1.0; // Motor 6
}
