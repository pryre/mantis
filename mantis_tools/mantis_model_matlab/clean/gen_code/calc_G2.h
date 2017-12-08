#include <Eigen3/Dense>

inline void calc_G2(Eigen::Vector3d& G2, double g0_1_1, double g0_1_2, double g0_1_3, double g0_1_4, double g0_2_1, double g0_2_2, double g0_2_3, double g0_2_4, double g0_3_1, double g0_3_2, double g0_3_3, double g0_3_4, double g0_4_1, double g0_4_2, double g0_4_3, double g0_4_4, double l1, double l2, double r1, double r2) {
	G2(0,0) = g0_1_1*cos(r1+r2)-g0_1_3*sin(r1+r2);
	G2(1,0) = g0_2_1*cos(r1+r2)-g0_2_3*sin(r1+r2);
	G2(2,0) = g0_3_1*cos(r1+r2)-g0_3_3*sin(r1+r2);
	G2(3,0) = g0_4_1*cos(r1+r2)-g0_4_3*sin(r1+r2);
	G2(0,1) = g0_1_2;
	G2(1,1) = g0_2_2;
	G2(2,1) = g0_3_2;
	G2(3,1) = g0_4_2;
	G2(0,2) = g0_1_3*cos(r1+r2)+g0_1_1*sin(r1+r2);
	G2(1,2) = g0_2_3*cos(r1+r2)+g0_2_1*sin(r1+r2);
	G2(2,2) = g0_3_3*cos(r1+r2)+g0_3_1*sin(r1+r2);
	G2(3,2) = g0_4_3*cos(r1+r2)+g0_4_1*sin(r1+r2);
	G2(0,3) = g0_1_4+g0_1_1*l1*cos(r1)-g0_1_3*l1*sin(r1)+l2*cos(r2)*(g0_1_1*cos(r1)-g0_1_3*sin(r1))-l2*sin(r2)*(g0_1_3*cos(r1)+g0_1_1*sin(r1));
	G2(1,3) = g0_2_4+g0_2_1*l1*cos(r1)-g0_2_3*l1*sin(r1)+l2*cos(r2)*(g0_2_1*cos(r1)-g0_2_3*sin(r1))-l2*sin(r2)*(g0_2_3*cos(r1)+g0_2_1*sin(r1));
	G2(2,3) = g0_3_4+g0_3_1*l1*cos(r1)-g0_3_3*l1*sin(r1)+l2*cos(r2)*(g0_3_1*cos(r1)-g0_3_3*sin(r1))-l2*sin(r2)*(g0_3_3*cos(r1)+g0_3_1*sin(r1));
	G2(3,3) = g0_4_4+g0_4_1*l1*cos(r1)-g0_4_3*l1*sin(r1)+l2*cos(r2)*(g0_4_1*cos(r1)-g0_4_3*sin(r1))-l2*sin(r2)*(g0_4_3*cos(r1)+g0_4_1*sin(r1));
}
