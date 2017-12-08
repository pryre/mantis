#include <Eigen3/Dense>

inline void calc_Dq(Eigen::Vector3d& Dq, double IJ0x, double IJ0y, double IJ0z, double IJ1x, double IJ1y, double IJ1z, double IJ2x, double IJ2y, double IJ2z, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	Dq(0,0) = IJ0x+IJ2x*pow(cos(r1+r2),2.0)+IJ2z*pow(sin(r1+r2),2.0)+IJ1x*pow(cos(r1),2.0)+IJ1z*pow(sin(r1),2.0);
	Dq(2,0) = IJ2x*sin(r1*2.0+r2*2.0)*(-1.0/2.0)+IJ2z*sin(r1*2.0+r2*2.0)*(1.0/2.0)-IJ1x*sin(r1*2.0)*(1.0/2.0)+IJ1z*sin(r1*2.0)*(1.0/2.0);
	Dq(4,0) = -IJ2z*sin(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2x*l1*cos(r1+r2)*sin(r2);
	Dq(1,1) = IJ0y+IJ1y+IJ2y;
	Dq(3,1) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	Dq(5,1) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	Dq(6,1) = IJ1y+IJ2y;
	Dq(7,1) = IJ1y*2.0+IJ2y*2.0;
	Dq(0,2) = IJ2x*sin(r1*2.0+r2*2.0)*(-1.0/2.0)+IJ2z*sin(r1*2.0+r2*2.0)*(1.0/2.0)-IJ1x*sin(r1*2.0)*(1.0/2.0)+IJ1z*sin(r1*2.0)*(1.0/2.0);
	Dq(2,2) = IJ0z+IJ2z*pow(cos(r1+r2),2.0)+IJ2x*pow(sin(r1+r2),2.0)+IJ1z*pow(cos(r1),2.0)+IJ1x*pow(sin(r1),2.0);
	Dq(4,2) = -IJ2z*cos(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2x*l1*sin(r1+r2)*sin(r2);
	Dq(1,3) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	Dq(3,3) = m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(sin(r1),2.0)*(1.0/4.0);
	Dq(5,3) = IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0);
	Dq(6,3) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	Dq(7,3) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))*2.0+IJ1y*l1*sin(r1);
	Dq(0,4) = -IJ2z*sin(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2x*l1*cos(r1+r2)*sin(r2);
	Dq(2,4) = -IJ2z*cos(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2x*l1*sin(r1+r2)*sin(r2);
	Dq(4,4) = m0+m1+m2+IJ1z*(l1*l1)*(1.0/4.0)+IJ2z*pow(l2+l1*cos(r2)*2.0,2.0)*(1.0/4.0)+IJ2x*(l1*l1)*pow(sin(r2),2.0);
	Dq(1,5) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	Dq(3,5) = IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0);
	Dq(5,5) = m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0);
	Dq(6,5) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	Dq(7,5) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*2.0+IJ1y*l1*cos(r1);
	Dq(1,6) = IJ1y+IJ2y;
	Dq(3,6) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0);
	Dq(5,6) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0);
	Dq(6,6) = IJ1y+IJ2y;
	Dq(7,6) = IJ1y*2.0+IJ2y*2.0;
	Dq(1,7) = IJ1y*2.0+IJ2y*2.0;
	Dq(3,7) = IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))*2.0+IJ1y*l1*sin(r1);
	Dq(5,7) = IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*2.0+IJ1y*l1*cos(r1);
	Dq(6,7) = IJ1y*2.0+IJ2y*2.0;
	Dq(7,7) = IJ1y*4.0+IJ2y*4.0;
}
