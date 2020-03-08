/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

inline void calc_Jj2(Eigen::MatrixXd& Jj2, double l1) {
	Jj2(1,0) = l1;
	Jj2(5,0) = 1.0;
}
