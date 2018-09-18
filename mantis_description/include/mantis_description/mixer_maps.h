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

//==-- The Mantis Description Tools namespace
namespace MDTools {
	int mixer_generate_quad_x4(Eigen::MatrixXd &m);
	int mixer_generate_quad_p4(Eigen::MatrixXd &m);	//XXX: UNTESTED!

	int mixer_generate_hex_x6(Eigen::MatrixXd &m);
	int mixer_generate_hex_p6(Eigen::MatrixXd &m);	//XXX: UNTESTED!

	int mixer_generate_octo_x8(Eigen::MatrixXd &m);	//XXX: UNTESTED!
	int mixer_generate_octo_p8(Eigen::MatrixXd &m);	//XXX: UNTESTED!
}
