#include <Eigen3/Dense>

inline void calc_Lqd(Eigen::Vector3d& m) {
	Lqd(7,7) = 1.0/2.0E1;
	Lqd(8,8) = 1.0/2.0E1;
}
