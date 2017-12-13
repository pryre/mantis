#include <eigen3/Eigen/Dense>

inline void calc_Dq(Eigen::MatrixXd& Dq, double IJ0x, double IJ0y, double IJ0z, double IJ1x, double IJ1y, double IJ1z, double IJ2x, double IJ2y, double IJ2z, double l0, double l1, double l2, double m0, double m1, double m2, double r1, double r2) {
	Dq(0,0) = IJ0x+IJ2x*pow(cos(r1+r2),2.0)+IJ2y*pow(sin(r1+r2),2.0)+IJ1x*pow(cos(r1),2.0)+IJ1y*pow(sin(r1),2.0);
	Dq(2,0) = IJ2x*sin(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*sin(r1*2.0+r2*2.0)*(1.0/2.0)+IJ1x*sin(r1*2.0)*(1.0/2.0)-IJ1y*sin(r1*2.0)*(1.0/2.0);
	Dq(4,0) = IJ1x*l0*pow(cos(r1),2.0)+IJ1y*sin(r1)*(l1*(1.0/2.0)+l0*sin(r1))+IJ2y*sin(r1+r2)*(l2*(1.0/2.0)+l0*sin(r1+r2)+l1*cos(r2))+IJ2x*cos(r1+r2)*(l0*cos(r1+r2)-l1*sin(r2));
	Dq(1,1) = IJ0y+IJ1z+IJ2z;
	Dq(3,1) = -IJ1z*(l0+l1*sin(r1)*(1.0/2.0))-IJ2z*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1));
	Dq(5,1) = IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1z*l1*cos(r1)*(1.0/2.0);
	Dq(6,1) = -IJ1z-IJ2z;
	Dq(7,1) = -IJ2z;
	Dq(0,2) = IJ2x*sin(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*sin(r1*2.0+r2*2.0)*(1.0/2.0)+IJ1x*sin(r1*2.0)*(1.0/2.0)-IJ1y*sin(r1*2.0)*(1.0/2.0);
	Dq(2,2) = IJ0z+IJ2y*pow(cos(r1+r2),2.0)+IJ2x*pow(sin(r1+r2),2.0)+IJ1y*pow(cos(r1),2.0)+IJ1x*pow(sin(r1),2.0);
	Dq(4,2) = -IJ1y*cos(r1)*(l1*(1.0/2.0)+l0*sin(r1))-IJ2y*cos(r1+r2)*(l2*(1.0/2.0)+l0*sin(r1+r2)+l1*cos(r2))+IJ2x*sin(r1+r2)*(l0*cos(r1+r2)-l1*sin(r2))+IJ1x*l0*cos(r1)*sin(r1);
	Dq(1,3) = -IJ1z*(l0+l1*sin(r1)*(1.0/2.0))-IJ2z*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1));
	Dq(3,3) = m0+m1*pow(cos(r1),2.0)+IJ2z*pow(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1),2.0)+m1*pow(sin(r1),2.0)+IJ1z*pow(l0+l1*sin(r1)*(1.0/2.0),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0);
	Dq(5,3) = -IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))-IJ1z*l1*cos(r1)*(l0+l1*sin(r1)*(1.0/2.0))*(1.0/2.0);
	Dq(6,3) = IJ1z*l0+IJ2z*l0-l1*m1*sin(r1)*(1.0/2.0)-l1*m2*sin(r1)+IJ2z*l2*sin(r1+r2)*(1.0/2.0)-l2*m2*sin(r1+r2)*(1.0/2.0)+IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2z*l1*sin(r1);
	Dq(7,3) = IJ2z*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))-l2*m2*sin(r1+r2)*(1.0/2.0);
	Dq(0,4) = IJ1x*l0*pow(cos(r1),2.0)+IJ1y*sin(r1)*(l1*(1.0/2.0)+l0*sin(r1))+IJ2y*sin(r1+r2)*(l2*(1.0/2.0)+l0*sin(r1+r2)+l1*cos(r2))+IJ2x*cos(r1+r2)*(l0*cos(r1+r2)-l1*sin(r2));
	Dq(2,4) = -IJ1y*cos(r1)*(l1*(1.0/2.0)+l0*sin(r1))-IJ2y*cos(r1+r2)*(l2*(1.0/2.0)+l0*sin(r1+r2)+l1*cos(r2))+IJ2x*sin(r1+r2)*(l0*cos(r1+r2)-l1*sin(r2))+IJ1x*l0*cos(r1)*sin(r1);
	Dq(4,4) = m0+m1+m2+IJ1y*pow(l1+l0*sin(r1)*2.0,2.0)*(1.0/4.0)+IJ2x*pow(l0*cos(r1+r2)-l1*sin(r2),2.0)+IJ2y*pow(l2+l0*sin(r1+r2)*2.0+l1*cos(r2)*2.0,2.0)*(1.0/4.0)+IJ1x*(l0*l0)*pow(cos(r1),2.0);
	Dq(1,5) = IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1z*l1*cos(r1)*(1.0/2.0);
	Dq(3,5) = -IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))-IJ1z*l1*cos(r1)*(l0+l1*sin(r1)*(1.0/2.0))*(1.0/2.0);
	Dq(5,5) = m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2z*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1z*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0);
	Dq(6,5) = l1*m1*cos(r1)*(1.0/2.0)+l1*m2*cos(r1)-IJ2z*l2*cos(r1+r2)*(1.0/2.0)+l2*m2*cos(r1+r2)*(1.0/2.0)-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2z*l1*cos(r1);
	Dq(7,5) = -IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+l2*m2*cos(r1+r2)*(1.0/2.0);
	Dq(1,6) = -IJ1z-IJ2z;
	Dq(3,6) = IJ1z*l0+IJ2z*l0-l1*m1*sin(r1)*(1.0/2.0)-l1*m2*sin(r1)+IJ2z*l2*sin(r1+r2)*(1.0/2.0)-l2*m2*sin(r1+r2)*(1.0/2.0)+IJ1z*l1*sin(r1)*(1.0/2.0)+IJ2z*l1*sin(r1);
	Dq(5,6) = l1*m1*cos(r1)*(1.0/2.0)+l1*m2*cos(r1)-IJ2z*l2*cos(r1+r2)*(1.0/2.0)+l2*m2*cos(r1+r2)*(1.0/2.0)-IJ1z*l1*cos(r1)*(1.0/2.0)-IJ2z*l1*cos(r1);
	Dq(6,6) = IJ1z+IJ2z+(l1*l1)*m1*(1.0/4.0)+(l1*l1)*m2+(l2*l2)*m2*(1.0/4.0)+l1*l2*m2*cos(r2);
	Dq(7,6) = IJ2z+l2*m2*(l2*(1.0/2.0)+l1*cos(r2))*(1.0/2.0);
	Dq(1,7) = -IJ2z;
	Dq(3,7) = IJ2z*(l0+l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))-l2*m2*sin(r1+r2)*(1.0/2.0);
	Dq(5,7) = -IJ2z*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+l2*m2*cos(r1+r2)*(1.0/2.0);
	Dq(6,7) = IJ2z+l2*m2*(l2*(1.0/2.0)+l1*cos(r2))*(1.0/2.0);
	Dq(7,7) = IJ2z+(l2*l2)*m2*(1.0/4.0);
}
