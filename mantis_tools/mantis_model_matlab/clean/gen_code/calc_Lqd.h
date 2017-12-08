#include <Eigen3/Dense>

inline void calc_Lqd(Eigen::Vector3d& Lqd) {
	Lqd(6,6) = 1.0/2.0E1;
	Lqd(7,7) = 1.0/2.0E1;
}
