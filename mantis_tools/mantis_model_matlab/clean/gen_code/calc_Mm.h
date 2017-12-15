#include <eigen3/Eigen/Dense>

inline void calc_Mm(Eigen::MatrixXd& Mm) {
	Mm(0,2) = 2.427773731488225E-2;
	Mm(1,2) = 2.427773731488225E-2;
	Mm(2,2) = 2.427773731488225E-2;
	Mm(3,2) = 2.427773731488225E-2;
	Mm(4,2) = 2.427773731488225E-2;
	Mm(5,2) = 2.427773731488225E-2;
	Mm(0,3) = -1.324240217175396E-1;
	Mm(1,3) = 1.324240217175396E-1;
	Mm(2,3) = 1.324240217175396E-1;
	Mm(3,3) = -1.324240217175396E-1;
	Mm(4,3) = -1.324240217175396E-1;
	Mm(5,3) = 1.324240217175396E-1;
	Mm(2,4) = 2.648480434350791E-1;
	Mm(3,4) = -2.648480434350791E-1;
	Mm(4,4) = 2.648480434350791E-1;
	Mm(5,4) = -2.648480434350791E-1;
	Mm(0,5) = -1.0E1/3.0;
	Mm(1,5) = 1.0E1/3.0;
	Mm(2,5) = -1.0E1/3.0;
	Mm(3,5) = 1.0E1/3.0;
	Mm(4,5) = 1.0E1/3.0;
	Mm(5,5) = -1.0E1/3.0;
	Mm(6,6) = 1.0;
	Mm(7,7) = 1.0;
}
