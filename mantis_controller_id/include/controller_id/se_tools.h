#include <eigen3/Eigen/Dense>

Eigen::Matrix3d vee_up(const Eigen::Vector3d& w) {
	Eigen::Matrix3d W;

	W <<  0.0, -w(2),  w(1),
		 w(2),   0.0, -w(0),
		-w(1),  w(0),   0.0;

	return W;
}

Eigen::Vector3d vee_down(const Eigen::Matrix3d& W) {
	return Eigen::Vector3d(W(2,1), W(0,2), W(1,0));
}
