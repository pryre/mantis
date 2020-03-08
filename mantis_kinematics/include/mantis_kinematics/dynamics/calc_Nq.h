/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

inline void calc_Nq(Eigen::MatrixXd& Nq, double g, double gb_3_3, double l1, double lc1, double lc2, double m0, double m1, double m2, double r1, double r2) {
	Nq(2,0) = (g*(m0+m1+m2))/gb_3_3;
	Nq(4,0) = -(g*(l1*m2*sin(r1)+lc1*m1*sin(r1)+lc2*m2*sin(r1+r2)))/gb_3_3;
	Nq(6,0) = (g*(l1*m2*sin(r1)+lc1*m1*sin(r1)+lc2*m2*sin(r1+r2)))/gb_3_3;
	Nq(7,0) = (g*lc2*m2*sin(r1+r2))/gb_3_3;
}
