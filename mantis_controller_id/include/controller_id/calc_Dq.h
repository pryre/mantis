#include <eigen3/Eigen/Dense>

inline void calc_Dq(Eigen::MatrixXd& m, double IJ1x, double IJ1y, double IJ1z, double IJ2x, double IJ2y, double IJ2z, double IT0x, double IT0y, double IT0z, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	m(0,0) = IT0x+IJ2x*pow(cos(r1+r2),2.0)+IJ2z*pow(sin(r1+r2),2.0)+IJ1x*pow(cos(r1),2.0)+IJ1z*pow(sin(r1),2.0);
	m(0,2) = IJ2x*sin(r1*2.0+r2*2.0)*(-1.0/2.0)+IJ2z*sin(r1*2.0+r2*2.0)*(1.0/2.0)-IJ1x*sin(r1*2.0)*(1.0/2.0)+IJ1z*sin(r1*2.0)*(1.0/2.0);
	m(0,4) = -IJ2z*sin(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2x*l1*cos(r1+r2)*sin(r2);
	m(1,1) = IJ1y+IJ2y+IT0y;
	m(1,3) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	m(1,5) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	m(1,6) = IJ1y+IJ2y;
	m(1,7) = IJ1y*2.0+IJ2y*2.0;
	m(2,0) = IJ2x*sin(r1*2.0+r2*2.0)*(-1.0/2.0)+IJ2z*sin(r1*2.0+r2*2.0)*(1.0/2.0)-IJ1x*sin(r1*2.0)*(1.0/2.0)+IJ1z*sin(r1*2.0)*(1.0/2.0);
	m(2,2) = IT0z+IJ2z*pow(cos(r1+r2),2.0)+IJ2x*pow(sin(r1+r2),2.0)+IJ1z*pow(cos(r1),2.0)+IJ1x*pow(sin(r1),2.0);
	m(2,4) = -IJ2z*cos(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2x*l1*sin(r1+r2)*sin(r2);
	m(3,1) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	m(3,3) = m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(sin(r1),2.0)*(1.0/4.0);
	m(3,5) = IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0);
	m(3,6) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	m(3,7) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))*2.0+IJ1y*l1*sin(r1);
	m(4,0) = -IJ2z*sin(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2x*l1*cos(r1+r2)*sin(r2);
	m(4,2) = -IJ2z*cos(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2x*l1*sin(r1+r2)*sin(r2);
	m(4,4) = m0+m1+m2+IJ1z*(l1*l1)*(1.0/4.0)+IJ2z*pow(l2+l1*cos(r2)*2.0,2.0)*(1.0/4.0)+IJ2x*(l1*l1)*pow(sin(r2),2.0);
	m(5,1) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	m(5,3) = IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0);
	m(5,5) = m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0);
	m(5,6) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	m(5,7) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*2.0+IJ1y*l1*cos(r1);
	m(6,1) = IJ1y+IJ2y;
	m(6,3) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	m(6,5) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	m(6,6) = IJ1y+IJ2y;
	m(6,7) = IJ1y*2.0+IJ2y*2.0;
	m(7,1) = IJ1y*2.0+IJ2y*2.0;
	m(7,3) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))*2.0+IJ1y*l1*sin(r1);
	m(7,5) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*2.0+IJ1y*l1*cos(r1);
	m(7,6) = IJ1y*2.0+IJ2y*2.0;
	m(7,7) = IJ1y*4.0+IJ2y*4.0;
}
