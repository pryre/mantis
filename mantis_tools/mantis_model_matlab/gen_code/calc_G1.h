/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

inline void calc_G1(Eigen::MatrixXd& G1, double gb_1_1, double gb_1_2, double gb_1_3, double gb_1_4, double gb_2_1, double gb_2_2, double gb_2_3, double gb_2_4, double gb_3_1, double gb_3_2, double gb_3_3, double gb_3_4, double l0) {
	G1(0,0) = gb_1_1;
	G1(1,0) = gb_2_1;
	G1(2,0) = gb_3_1;
	G1(0,1) = gb_1_3;
	G1(1,1) = gb_2_3;
	G1(2,1) = gb_3_3;
	G1(0,2) = -gb_1_2;
	G1(1,2) = -gb_2_2;
	G1(2,2) = -gb_3_2;
	G1(0,3) = gb_1_4+gb_1_3*l0;
	G1(1,3) = gb_2_4+gb_2_3*l0;
	G1(2,3) = gb_3_4+gb_3_3*l0;
	G1(3,3) = 1.0;
}
