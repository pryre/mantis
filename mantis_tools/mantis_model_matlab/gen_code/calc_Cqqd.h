/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

inline void calc_Cqqd(Eigen::MatrixXd& Cqqd, double IJ0x, double IJ0y, double IJ0z, double IJ1x, double IJ1y, double IJ2x, double IJ2y, double bvx, double bvy, double bvz, double bw1, double bw2, double bw3, double l0, double l1, double lc1, double lc2, double m0, double m1, double m2, double r1, double r1d, double r2, double r2d) {
	Cqqd(1,0) = bw3*m0;
	Cqqd(2,0) = -bw2*m0;
	Cqqd(4,0) = r1d*(l1*m2*sin(r1)*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0)+lc2*m2*sin(r1+r2)*(1.0/2.0))+lc2*m2*r2d*sin(r1+r2)*(1.0/2.0);
	Cqqd(6,0) = -bw2*(l1*m2*sin(r1)*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0)+lc2*m2*sin(r1+r2)*(1.0/2.0));
	Cqqd(7,0) = bw2*lc2*m2*sin(r1+r2)*(-1.0/2.0);
	Cqqd(0,1) = -bw3*m0;
	Cqqd(2,1) = bw1*m0;
	Cqqd(3,1) = -r1d*(m2*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0))-lc2*m2*r2d*sin(r1+r2)*(1.0/2.0);
	Cqqd(5,1) = r1d*(m2*(lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0))+lc2*m2*r2d*cos(r1+r2)*(1.0/2.0);
	Cqqd(6,1) = -bw3*(m2*(lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0))+bw1*(m2*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0));
	Cqqd(7,1) = bw3*lc2*m2*cos(r1+r2)*(-1.0/2.0)+bw1*lc2*m2*sin(r1+r2)*(1.0/2.0);
	Cqqd(0,2) = bw2*m0;
	Cqqd(1,2) = -bw1*m0;
	Cqqd(4,2) = -r1d*(l1*m2*cos(r1)*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0)+lc2*m2*cos(r1+r2)*(1.0/2.0))-lc2*m2*r2d*cos(r1+r2)*(1.0/2.0);
	Cqqd(6,2) = bw2*(l1*m2*cos(r1)*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0)+lc2*m2*cos(r1+r2)*(1.0/2.0));
	Cqqd(7,2) = bw2*lc2*m2*cos(r1+r2)*(1.0/2.0);
	Cqqd(1,3) = -r1d*(m2*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0))-lc2*m2*r2d*sin(r1+r2)*(1.0/2.0);
	Cqqd(3,3) = r1d*(IJ2x*cos(r1+r2)*sin(r1+r2)-IJ2y*cos(r1+r2)*sin(r1+r2)+IJ1x*cos(r1)*sin(r1)-IJ1y*cos(r1)*sin(r1)-m2*(lc2*sin(r1+r2)+l1*sin(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))+lc1*m1*sin(r1)*(l0-lc1*cos(r1)))-r2d*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)+lc2*m2*sin(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1)));
	Cqqd(4,3) = IJ0x*bw3;
	Cqqd(5,3) = -IJ0x*bw2-r1d*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)+m2*pow(lc2*sin(r1+r2)+l1*sin(r1),2.0)*(1.0/2.0)+IJ1x*cos(r1*2.0)*(1.0/2.0)-IJ1y*cos(r1*2.0)*(1.0/2.0)-m2*(lc2*cos(r1+r2)+l1*cos(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+(lc1*lc1)*m1*pow(sin(r1),2.0)*(1.0/2.0)+lc1*m1*cos(r1)*(l0-lc1*cos(r1))*(1.0/2.0))-r2d*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)-lc2*m2*cos(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc2*m2*sin(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0));
	Cqqd(6,3) = -bw1*(IJ2x*cos(r1+r2)*sin(r1+r2)-IJ2y*cos(r1+r2)*sin(r1+r2)+IJ1x*cos(r1)*sin(r1)-IJ1y*cos(r1)*sin(r1)-m2*(lc2*sin(r1+r2)+l1*sin(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))+lc1*m1*sin(r1)*(l0-lc1*cos(r1)))+bw3*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)+m2*pow(lc2*sin(r1+r2)+l1*sin(r1),2.0)*(1.0/2.0)+IJ1x*cos(r1*2.0)*(1.0/2.0)-IJ1y*cos(r1*2.0)*(1.0/2.0)-m2*(lc2*cos(r1+r2)+l1*cos(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+(lc1*lc1)*m1*pow(sin(r1),2.0)*(1.0/2.0)+lc1*m1*cos(r1)*(l0-lc1*cos(r1))*(1.0/2.0))+bvy*(m2*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0));
	Cqqd(7,3) = bw1*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)+lc2*m2*sin(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1)))+bw3*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)-lc2*m2*cos(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc2*m2*sin(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0))+bvy*lc2*m2*sin(r1+r2)*(1.0/2.0);
	Cqqd(0,4) = r1d*(l1*m2*sin(r1)*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0)+lc2*m2*sin(r1+r2)*(1.0/2.0))+lc2*m2*r2d*sin(r1+r2)*(1.0/2.0);
	Cqqd(2,4) = -r1d*(l1*m2*cos(r1)*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0)+lc2*m2*cos(r1+r2)*(1.0/2.0))-lc2*m2*r2d*cos(r1+r2)*(1.0/2.0);
	Cqqd(3,4) = -IJ0y*bw3;
	Cqqd(4,4) = r2d*(l0*lc2*m2*sin(r1+r2)-l1*lc2*m2*sin(r2))+r1d*(l0*lc2*m2*sin(r1+r2)+l0*l1*m2*sin(r1)+l0*lc1*m1*sin(r1));
	Cqqd(5,4) = IJ0y*bw1;
	Cqqd(6,4) = bvz*(l1*m2*cos(r1)*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0)+lc2*m2*cos(r1+r2)*(1.0/2.0))-bvx*(l1*m2*sin(r1)*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0)+lc2*m2*sin(r1+r2)*(1.0/2.0))-bw2*(l0*lc2*m2*sin(r1+r2)+l0*l1*m2*sin(r1)+l0*lc1*m1*sin(r1))+l1*lc2*m2*r2d*sin(r2);
	Cqqd(7,4) = -bw2*(l0*lc2*m2*sin(r1+r2)-l1*lc2*m2*sin(r2))+bvz*lc2*m2*cos(r1+r2)*(1.0/2.0)-bvx*lc2*m2*sin(r1+r2)*(1.0/2.0)-l1*lc2*m2*r1d*sin(r2);
	Cqqd(1,5) = r1d*(m2*(lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0))+lc2*m2*r2d*cos(r1+r2)*(1.0/2.0);
	Cqqd(3,5) = IJ0z*bw2-r1d*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)+m2*pow(lc2*sin(r1+r2)+l1*sin(r1),2.0)*(1.0/2.0)+IJ1x*cos(r1*2.0)*(1.0/2.0)-IJ1y*cos(r1*2.0)*(1.0/2.0)-m2*(lc2*cos(r1+r2)+l1*cos(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+(lc1*lc1)*m1*pow(sin(r1),2.0)*(1.0/2.0)+lc1*m1*cos(r1)*(l0-lc1*cos(r1))*(1.0/2.0))-r2d*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)-lc2*m2*cos(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc2*m2*sin(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0));
	Cqqd(4,5) = -IJ0z*bw1;
	Cqqd(5,5) = r1d*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)-IJ1x*cos(r1)*sin(r1)+IJ1y*cos(r1)*sin(r1)+m2*(lc2*cos(r1+r2)+l1*cos(r1))*(lc2*sin(r1+r2)+l1*sin(r1))+(lc1*lc1)*m1*cos(r1)*sin(r1))+r2d*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)+lc2*m2*cos(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1)));
	Cqqd(6,5) = -bw3*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)-IJ1x*cos(r1)*sin(r1)+IJ1y*cos(r1)*sin(r1)+m2*(lc2*cos(r1+r2)+l1*cos(r1))*(lc2*sin(r1+r2)+l1*sin(r1))+(lc1*lc1)*m1*cos(r1)*sin(r1))-bvy*(m2*(lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0))+bw1*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)+m2*pow(lc2*sin(r1+r2)+l1*sin(r1),2.0)*(1.0/2.0)+IJ1x*cos(r1*2.0)*(1.0/2.0)-IJ1y*cos(r1*2.0)*(1.0/2.0)-m2*(lc2*cos(r1+r2)+l1*cos(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+(lc1*lc1)*m1*pow(sin(r1),2.0)*(1.0/2.0)+lc1*m1*cos(r1)*(l0-lc1*cos(r1))*(1.0/2.0));
	Cqqd(7,5) = -bw3*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)+lc2*m2*cos(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1)))+bw1*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)-lc2*m2*cos(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc2*m2*sin(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0))-bvy*lc2*m2*cos(r1+r2)*(1.0/2.0);
	Cqqd(0,6) = bw2*(l1*m2*sin(r1)*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0)+lc2*m2*sin(r1+r2)*(1.0/2.0))-r1d*(l1*m2*sin(r1)+lc1*m1*sin(r1)+lc2*m2*sin(r1+r2))-lc2*m2*r2d*sin(r1+r2);
	Cqqd(1,6) = bw3*(m2*(lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0))-bw1*(m2*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0));
	Cqqd(2,6) = -bw2*(l1*m2*cos(r1)*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0)+lc2*m2*cos(r1+r2)*(1.0/2.0))+r1d*(l1*m2*cos(r1)+lc1*m1*cos(r1)+lc2*m2*cos(r1+r2))+lc2*m2*r2d*cos(r1+r2);
	Cqqd(3,6) = bw1*(IJ2x*cos(r1+r2)*sin(r1+r2)-IJ2y*cos(r1+r2)*sin(r1+r2)+IJ1x*cos(r1)*sin(r1)-IJ1y*cos(r1)*sin(r1)-m2*(lc2*sin(r1+r2)+l1*sin(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))+lc1*m1*sin(r1)*(l0-lc1*cos(r1)))-bw3*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)+m2*pow(lc2*sin(r1+r2)+l1*sin(r1),2.0)*(1.0/2.0)+IJ1x*cos(r1*2.0)*(1.0/2.0)-IJ1y*cos(r1*2.0)*(1.0/2.0)-m2*(lc2*cos(r1+r2)+l1*cos(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+(lc1*lc1)*m1*pow(sin(r1),2.0)*(1.0/2.0)+lc1*m1*cos(r1)*(l0-lc1*cos(r1))*(1.0/2.0))-bvy*(m2*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0));
	Cqqd(4,6) = -r2d*(l0*lc2*m2*sin(r1+r2)-l1*lc2*m2*sin(r2))-bvz*(l1*m2*cos(r1)*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0)+lc2*m2*cos(r1+r2)*(1.0/2.0))+bvx*(l1*m2*sin(r1)*(1.0/2.0)+lc1*m1*sin(r1)*(1.0/2.0)+lc2*m2*sin(r1+r2)*(1.0/2.0))+bw2*(l0*lc2*m2*sin(r1+r2)+l0*l1*m2*sin(r1)+l0*lc1*m1*sin(r1))-r1d*(l0*lc2*m2*sin(r1+r2)+l0*l1*m2*sin(r1)+l0*lc1*m1*sin(r1));
	Cqqd(5,6) = bw3*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)-IJ1x*cos(r1)*sin(r1)+IJ1y*cos(r1)*sin(r1)+m2*(lc2*cos(r1+r2)+l1*cos(r1))*(lc2*sin(r1+r2)+l1*sin(r1))+(lc1*lc1)*m1*cos(r1)*sin(r1))+bvy*(m2*(lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc1*m1*cos(r1)*(1.0/2.0))-bw1*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)+m2*pow(lc2*sin(r1+r2)+l1*sin(r1),2.0)*(1.0/2.0)+IJ1x*cos(r1*2.0)*(1.0/2.0)-IJ1y*cos(r1*2.0)*(1.0/2.0)-m2*(lc2*cos(r1+r2)+l1*cos(r1))*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+(lc1*lc1)*m1*pow(sin(r1),2.0)*(1.0/2.0)+lc1*m1*cos(r1)*(l0-lc1*cos(r1))*(1.0/2.0));
	Cqqd(6,6) = -l1*lc2*m2*r2d*sin(r2);
	Cqqd(7,6) = -bw2*l1*lc2*m2*sin(r2)+l1*lc2*m2*r1d*sin(r2);
	Cqqd(0,7) = bw2*lc2*m2*sin(r1+r2)*(1.0/2.0)-lc2*m2*r1d*sin(r1+r2)-lc2*m2*r2d*sin(r1+r2);
	Cqqd(1,7) = bw3*lc2*m2*cos(r1+r2)*(1.0/2.0)-bw1*lc2*m2*sin(r1+r2)*(1.0/2.0);
	Cqqd(2,7) = bw2*lc2*m2*cos(r1+r2)*(-1.0/2.0)+lc2*m2*r1d*cos(r1+r2)+lc2*m2*r2d*cos(r1+r2);
	Cqqd(3,7) = -bw1*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)+lc2*m2*sin(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1)))-bw3*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)-lc2*m2*cos(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc2*m2*sin(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0))-bvy*lc2*m2*sin(r1+r2)*(1.0/2.0);
	Cqqd(4,7) = bw2*(l0*lc2*m2*sin(r1+r2)-l1*lc2*m2*sin(r2))-r1d*(l0*lc2*m2*sin(r1+r2)-l1*lc2*m2*sin(r2))-lc2*m2*r2d*(l0*sin(r1+r2)-l1*sin(r2))-bvz*lc2*m2*cos(r1+r2)*(1.0/2.0)+bvx*lc2*m2*sin(r1+r2)*(1.0/2.0);
	Cqqd(5,7) = bw3*(-IJ2x*cos(r1+r2)*sin(r1+r2)+IJ2y*cos(r1+r2)*sin(r1+r2)+lc2*m2*cos(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1)))-bw1*(IJ2x*cos(r1*2.0+r2*2.0)*(1.0/2.0)-IJ2y*cos(r1*2.0+r2*2.0)*(1.0/2.0)-lc2*m2*cos(r1+r2)*(-l0+lc2*cos(r1+r2)+l1*cos(r1))*(1.0/2.0)+lc2*m2*sin(r1+r2)*(lc2*sin(r1+r2)+l1*sin(r1))*(1.0/2.0))+bvy*lc2*m2*cos(r1+r2)*(1.0/2.0);
	Cqqd(6,7) = bw2*l1*lc2*m2*sin(r2)-l1*lc2*m2*r1d*sin(r2)-l1*lc2*m2*r2d*sin(r2);
}
