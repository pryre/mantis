#include <Eigen3/Dense>

inline void calc_Nq(Eigen::Vector3d& m, double IJ1y, double IJ1z, double IJ2x, double IJ2y, double IJ2z, double g, double g0_3_1, double g0_3_2, double g0_3_3, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	Nq(1,1) = -g*g0_3_2*(IJ2z*sin(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))+IJ1z*l1*sin(r1)*(1.0/2.0)-IJ2x*l1*cos(r1+r2)*sin(r2));
	Nq(2,1) = g*g0_3_3*(IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0))+g*g0_3_1*(IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0));
	Nq(3,1) = -g*g0_3_2*(IJ2z*cos(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))+IJ1z*l1*cos(r1)*(1.0/2.0)+IJ2x*l1*sin(r1+r2)*sin(r2));
	Nq(4,1) = g*g0_3_1*(m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(sin(r1),2.0)*(1.0/4.0))+g*g0_3_3*(IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0));
	Nq(5,1) = g*g0_3_2*(m0+m1+m2+IJ1z*(l1*l1)*(1.0/4.0)+IJ2z*pow(l2+l1*cos(r2)*2.0,2.0)*(1.0/4.0)+IJ2x*(l1*l1)*pow(sin(r2),2.0));
	Nq(6,1) = g*g0_3_1*(IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0))+g*g0_3_3*(m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0));
	Nq(7,1) = g*g0_3_3*(IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0))+g*g0_3_1*(IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0));
	Nq(8,1) = g*g0_3_3*(IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*2.0+IJ1y*l1*cos(r1))+g*g0_3_1*(IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))*2.0+IJ1y*l1*sin(r1));
}
