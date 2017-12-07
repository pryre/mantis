void calc_Nq(Eigen::Vector3d& m, IJ1y, IJ1z, IJ2x, IJ2y, IJ2z, g, g0_3_1, g0_3_2, g0_3_3, l1, l2, m0, m1, m2, r1, r2) {
	m[0][0] = -g*g0_3_2*(IJ2z*sin(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))+IJ1z*l1*sin(r1)*(1.0/2.0)-IJ2x*l1*cos(r1+r2)*sin(r2));
	m[1][0] = g*g0_3_3*(IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0))+g*g0_3_1*(IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0));
	m[2][0] = -g*g0_3_2*(IJ2z*cos(r1+r2)*(l2*(1.0/2.0)+l1*cos(r2))+IJ1z*l1*cos(r1)*(1.0/2.0)+IJ2x*l1*sin(r1+r2)*sin(r2));
	m[3][0] = g*g0_3_1*(m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(sin(r1),2.0)*(1.0/4.0))+g*g0_3_3*(IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0));
	m[4][0] = g*g0_3_2*(m0+m1+m2+IJ1z*(l1*l1)*(1.0/4.0)+IJ2z*pow(l2+l1*cos(r2)*2.0,2.0)*(1.0/4.0)+IJ2x*(l1*l1)*pow(sin(r2),2.0));
	m[5][0] = g*g0_3_1*(IJ2y*((l1*l1)*sin(r1*2.0)*(1.0/2.0)+(l2*l2)*sin(r1*2.0+r2*2.0)*(1.0/8.0)+l1*l2*sin(r1*2.0+r2)*(1.0/2.0))+IJ1y*(l1*l1)*sin(r1*2.0)*(1.0/8.0))+g*g0_3_3*(m0+m1*pow(cos(r1),2.0)+m1*pow(sin(r1),2.0)+IJ2y*pow(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1),2.0)+m2*pow(cos(r1+r2),2.0)+m2*pow(sin(r1+r2),2.0)+IJ1y*(l1*l1)*pow(cos(r1),2.0)*(1.0/4.0));
	m[6][0] = g*g0_3_3*(IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))+IJ1y*l1*cos(r1)*(1.0/2.0))+g*g0_3_1*(IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))+IJ1y*l1*sin(r1)*(1.0/2.0));
	m[7][0] = g*g0_3_3*(IJ2y*(l2*cos(r1+r2)*(1.0/2.0)+l1*cos(r1))*2.0+IJ1y*l1*cos(r1))+g*g0_3_1*(IJ2y*(l2*sin(r1+r2)*(1.0/2.0)+l1*sin(r1))*2.0+IJ1y*l1*sin(r1));
}