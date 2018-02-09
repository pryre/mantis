#include <eigen3/Eigen/Dense>

inline void calc_Jj2(Eigen::MatrixXd& Jj2, double l1) {
	Jj2(1,0) = l1;
	Jj2(5,0) = 1.0;
}
