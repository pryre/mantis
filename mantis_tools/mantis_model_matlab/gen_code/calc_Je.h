#include <eigen3/Eigen/Dense>

inline void calc_Je(Eigen::MatrixXd& Je, double l1, double l2, double r2) {
	Je(0,0) = l1*sin(r2);
	Je(1,0) = l2+l1*cos(r2);
	Je(5,0) = 1.0;
	Je(1,1) = l2;
	Je(5,1) = 1.0;
}
