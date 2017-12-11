#include <eigen3/Eigen/Dense>

inline void calc_Lqd(Eigen::MatrixXd& Lqd) {
	Lqd(6,6) = 0.01;
	Lqd(7,7) = 0.01;
}
