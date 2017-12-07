#include <Eigen3/Dense>

inline void calc_G0(Eigen::Vector3d& m, double g0_1_1, double g0_1_2, double g0_1_3, double g0_1_4, double g0_2_1, double g0_2_2, double g0_2_3, double g0_2_4, double g0_3_1, double g0_3_2, double g0_3_3, double g0_3_4, double g0_4_1, double g0_4_2, double g0_4_3, double g0_4_4) {
	G0(1,1) = g0_1_1;
	G0(2,1) = g0_2_1;
	G0(3,1) = g0_3_1;
	G0(4,1) = g0_4_1;
	G0(1,2) = g0_1_2;
	G0(2,2) = g0_2_2;
	G0(3,2) = g0_3_2;
	G0(4,2) = g0_4_2;
	G0(1,3) = g0_1_3;
	G0(2,3) = g0_2_3;
	G0(3,3) = g0_3_3;
	G0(4,3) = g0_4_3;
	G0(1,4) = g0_1_4;
	G0(2,4) = g0_2_4;
	G0(3,4) = g0_3_4;
	G0(4,4) = g0_4_4;
}
