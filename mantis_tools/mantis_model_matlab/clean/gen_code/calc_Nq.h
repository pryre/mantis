#include <eigen3/Eigen/Dense>

inline void calc_Nq(Eigen::MatrixXd& Nq, double IJ1z, double IJ2z, double g, double gb_3_3, double l0, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	Nq(1,0) = (g*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1z*l1*cos(r1)*(1.0/2.0)))/gb_3_3;
	Nq(3,0) = -(g*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1z*l1*cos(r1)*(l0+l1*sin(r1)*(1.0/2.0))*(1.0/2.0)))/gb_3_3;
	Nq(5,0) = (g*(m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2z*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1z*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0)))/gb_3_3;
	Nq(6,0) = (g*(l1*m1*cos(r1)*(1.0/2.0)+l1*m2*cos(r1)-IJ2z*l2*cos(r1+r2)*(1.0/2.0)+l2*m2*cos(r1+r2)*(1.0/2.0)-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2z*l1*cos(r1)))/gb_3_3;
	Nq(7,0) = -(g*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))-l2*m2*cos(r1+r2)*(1.0/2.0)))/gb_3_3;
}
