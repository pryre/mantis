#include <eigen3/Eigen/Dense>
#include <math.h>

void dh_gen(Eigen::Affine3d &g, const double d, const double t, const double r, const double a) {
	Eigen::Affine3d Td = Eigen::Affine3d::Identity();
	Eigen::Affine3d Rt = Eigen::Affine3d::Identity();
	Eigen::Affine3d Tr = Eigen::Affine3d::Identity();
	Eigen::Affine3d Ra = Eigen::Affine3d::Identity();

    //Translation of d
	//Td = [1, 0, 0, 0;
    //		0, 1, 0, 0;
	//		0, 0, 1, d;
    //		0, 0, 0, 1];
	Td.translation() << Eigen::Vector3d(0.0, 0.0, d);

	//Rotation of theta
	//Rt = [cos(theta), -sin(theta), 0, 0;
	//		sin(theta),  cos(theta), 0, 0;
	//				 0,			  0, 1, 0;
	//				 0,			  0, 0, 1];
	Rt.linear().block(0,0,2,2) << std::cos(t), -std::sin(t),
								  std::sin(t),  std::cos(t);

	//Translation of r
	//Tr = [1, 0, 0, r;
    //		0, 1, 0, 0;
	//		0, 0, 1, 0;
    //		0, 0, 0, 1];
	Tr.translation() << Eigen::Vector3d(r, 0.0, 0.0);

    //Rotation of alpha
    //Ra = [1,			0,			 0, 0; ...
    //		0, cos(alpha), -sin(alpha), 0; ...
    //		0, sin(alpha),  cos(alpha), 0; ...
    //		0,			0,			 0, 1];
	Ra.linear().block(1,1,2,2) << std::cos(a), -std::sin(a),
								  std::sin(a),  std::cos(a);

    //Formulate transformation
    g = ((Rt*Td)*Tr)*Ra;
}
