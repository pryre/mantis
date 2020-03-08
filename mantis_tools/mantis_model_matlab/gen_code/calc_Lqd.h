/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <eigen3/Eigen/Dense>

inline void calc_Lqd(Eigen::MatrixXd& Lqd) {
	Lqd(6,6) = 1.0/2.0E1;
	Lqd(7,7) = 1.0/2.0E1;
}
