#include <eigen3/Eigen/Dense>

inline void calc_G0(Eigen::MatrixXd& m, double g0_1_1, double g0_1_2, double g0_1_3, double g0_1_4, double g0_2_1, double g0_2_2, double g0_2_3, double g0_2_4, double g0_3_1, double g0_3_2, double g0_3_3, double g0_3_4, double g0_4_1, double g0_4_2, double g0_4_3, double g0_4_4) {
	m(0,0) = g0_1_1;
	m(0,1) = g0_1_2;
	m(0,2) = g0_1_3;
	m(0,3) = g0_1_4;
	m(1,0) = g0_2_1;
	m(1,1) = g0_2_2;
	m(1,2) = g0_2_3;
	m(1,3) = g0_2_4;
	m(2,0) = g0_3_1;
	m(2,1) = g0_3_2;
	m(2,2) = g0_3_3;
	m(2,3) = g0_3_4;
	m(3,0) = g0_4_1;
	m(3,1) = g0_4_2;
	m(3,2) = g0_4_3;
	m(3,3) = g0_4_4;
}
