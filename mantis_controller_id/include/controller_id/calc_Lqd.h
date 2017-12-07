#include <Eigen3/Dense>

inline void calc_Lqd(Eigen3::MatrixXd& m) {
	m[6][6] = 1.0/2.0E1;
	m[7][7] = 1.0/2.0E1;
}
