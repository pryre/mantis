#include <eigen3/Eigen/Dense>

inline void calc_Nq(Eigen::MatrixXd& Nq, double IJ1x, double IJ1y, double IJ1z, double IJ2x, double IJ2y, double IJ2z, double g, double gb_3_1, double gb_3_2, double gb_3_3, double l0, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	Nq(0,0) = g*gb_3_2*(IJ1x*l0*pow(cos(r1),2.0)+IJ1y*sin(r1)*(l1*(1.0/2.0)+l0*sin(r1))+IJ2y*sin(r1+r2)*(l2*(1.0/2.0)+l0*sin(r1+r2)+l1*cos(r2))+IJ2x*cos(r1+r2)*(l0*cos(r1+r2)-l1*sin(r2)));
	Nq(1,0) = -g*gb_3_1*(IJ1z*(l0+l1*sin(r1)*(1.0/2.0))+IJ2z*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1)))+g*gb_3_3*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1z*l1*cos(r1)*(1.0/2.0));
	Nq(2,0) = -g*gb_3_2*(IJ1y*cos(r1)*(l1*(1.0/2.0)+l0*sin(r1))+IJ2y*cos(r1+r2)*(l2*(1.0/2.0)+l0*sin(r1+r2)+l1*cos(r2))-IJ2x*sin(r1+r2)*(l0*cos(r1+r2)-l1*sin(r2))-IJ1x*l0*cos(r1)*sin(r1));
	Nq(3,0) = -g*gb_3_3*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1z*l1*cos(r1)*(l0+l1*sin(r1)*(1.0/2.0))*(1.0/2.0))+g*gb_3_1*(m0+m1*pow(cos(r1),2.0)+IJ2z*pow(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1),2.0)+m1*pow(sin(r1),2.0)+IJ1z*pow(l0+l1*sin(r1)*(1.0/2.0),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0));
	Nq(4,0) = g*gb_3_2*(m0+m1+m2+IJ1y*pow(l1+l0*sin(r1)*2.0,2.0)*(1.0/4.0)+IJ2x*pow(l0*cos(r1+r2)-l1*sin(r2),2.0)+IJ2y*pow(l2+l0*sin(r1+r2)*2.0+l1*cos(r2)*2.0,2.0)*(1.0/4.0)+IJ1x*(l0*l0)*pow(cos(r1),2.0));
	Nq(5,0) = -g*gb_3_1*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1z*l1*cos(r1)*(l0+l1*sin(r1)*(1.0/2.0))*(1.0/2.0))+g*gb_3_3*(m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2z*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1z*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0));
	Nq(6,0) = g*gb_3_3*(l1*m1*cos(r1)*(1.0/2.0)+l1*m2*cos(r1)-IJ2z*l2*cos(r1+r2)*(1.0/2.0)+l2*m2*cos(r1+r2)*(1.0/2.0)-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2z*l1*cos(r1))+g*gb_3_1*(IJ1z*l0+IJ2z*l0-l1*m1*sin(r1)*(1.0/2.0)-l1*m2*sin(r1)+IJ2z*l2*sin(r1+r2)*(1.0/2.0)-l2*m2*sin(r1+r2)*(1.0/2.0)+IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2z*l1*sin(r1));
	Nq(7,0) = g*gb_3_1*(IJ2z*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))-l2*m2*sin(r1+r2)*(1.0/2.0))-g*gb_3_3*(IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))-l2*m2*cos(r1+r2)*(1.0/2.0));
}
