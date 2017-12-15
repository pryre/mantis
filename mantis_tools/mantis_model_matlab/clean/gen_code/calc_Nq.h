#include <eigen3/Eigen/Dense>

inline void calc_Nq(Eigen::MatrixXd& Nq, double g, double gb_3_3, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	Nq(2,0) = (g*(m0+m1+m2))/gb_3_3;
	Nq(4,0) = -(g*(l1*m1*sin(r1)*(1.0/2.0)+l1*m2*sin(r1)+l2*m2*sin(r1+r2)*(1.0/2.0)))/gb_3_3;
	Nq(6,0) = (g*(l1*m1*sin(r1)*(1.0/2.0)+l1*m2*sin(r1)+l2*m2*sin(r1+r2)*(1.0/2.0)))/gb_3_3;
	Nq(7,0) = (g*l2*m2*sin(r1+r2)*(1.0/2.0))/gb_3_3;
}
