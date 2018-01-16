#include <ros/ros.h>

#include <mantis_paths/paths.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>

Paths::Paths() :
	nh_("~"),
	param_frame_id_("map"),
	param_arc_res_(0),
	running_seq_(0) {

	//Setup publisher and get known parameters
	pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1, true);

	nh_.param("frame_id", param_frame_id_, param_frame_id_);
	nh_.param("arc_resolution", param_arc_res_, param_arc_res_);

	//Give the node a moment to recieve a clock message (to allow it to work with simulated time)
	ros::Duration(0.2).sleep();
	ros::spinOnce();

	bool error = false;

	ROS_INFO("Loading path starting point");

	//Check that params were loaded correctly
	if( load_start_params() ) {
		add_pose(ros::Duration(0), pose_from_eigen(param_start_position_, param_start_orientation_));

		ROS_INFO("Generating path...");

		error = !generate_path();

		if(!error) {
			msg_out_.header.frame_id = param_frame_id_;
			msg_out_.header.stamp = ros::Time::now();

			pub_path_.publish(msg_out_);
		}
	} else {
		error = true;
		ROS_ERROR("Invalid starting parameters (position or direction)");
	}

	if(error)
		ros::shutdown();
}

Paths::~Paths() {
}

bool Paths::load_start_params(void) {
	bool success = false;

	std::vector<double> start_p;
	std::vector<double> start_dq;
	std::vector<double> start_oq;

	if( nh_.getParam("start/position", start_p) &&
		nh_.getParam("start/direction", start_dq) ) {

		param_start_position_ = vector_from_doubles(start_p);
		param_start_direction_ = quaternion_from_doubles(start_dq);

		if( nh_.getParam("start/orientation", start_oq) ) {
			param_start_direction_ = quaternion_from_doubles(start_oq);
		} else {
			ROS_WARN("Using starting direction as starting orientation!");
			param_start_orientation_ = param_start_direction_;
		}

		success = true;
	}

	return success;
}

bool Paths::generate_path() {
	bool error = false;

	int i = 0;
	std::string seg_type;

	Eigen::Vector3d pc = param_start_position_;	//Current position
	Eigen::Quaterniond qc = param_start_direction_;	//Current direction

	//Loop through every segment available
	while( (!error) && nh_.getParam("path/s" + std::to_string(i) + "/type", seg_type) ) {
		double height = 0.0;
		double velocity = 0.0;
		if( nh_.getParam("path/s" + std::to_string(i) + "/velocity", velocity) &&
			nh_.getParam("path/s" + std::to_string(i) + "/height", height) ) {

			if(seg_type == "line") {
				double length = 0.0;

				if( nh_.getParam("path/s" + std::to_string(i) + "/length", length) ) {
					pc += qc.toRotationMatrix()*Eigen::Vector3d(length, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, height);
				} else {
					ROS_ERROR("Could not load length for line segment (%i)", i);
					error = true;
				}

				Eigen::Quaterniond qo = qc;
				std::vector<double> orientation;
				if( nh_.getParam("path/s" + std::to_string(i) + "/orientation", orientation) ) {
					qo = quaternion_from_doubles(orientation);
				}

				if(!error)
					add_pose(travel_time(Eigen::Vector3d(length, 0.0, height).norm(), velocity), pose_from_eigen(pc, qo));
			} else if(seg_type == "arc") {
				double radius = 0.0;
				double theta = 0.0;

				if( nh_.getParam("path/s" + std::to_string(i) + "/radius", radius) &&
					nh_.getParam("path/s" + std::to_string(i) + "/theta", theta) ) {

					double dtheta = theta / param_arc_res_;
					double dx = std::fabs(radius*dtheta);
					double dz = height / param_arc_res_;
					double length = Eigen::Vector3d(dx,0.0,dz).norm();	//Integrate over the arc

					Eigen::Quaterniond qo = qc;
					Eigen::Quaterniond qcs = qc;
					bool use_orientation = false;
					std::vector<double> orientation;
					if( nh_.getParam("path/s" + std::to_string(i) + "/orientation", orientation) ) {
						qo = quaternion_from_doubles(orientation);
						use_orientation = true;
					}

					for(int j=1; j<=param_arc_res_; j++) {
						double alpha = (double)j / (double)param_arc_res_;
						pc += qc.toRotationMatrix()*Eigen::Vector3d(dx, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, dz);
						qc *= Eigen::Quaterniond(Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitZ()));

						Eigen::Quaterniond qs = use_orientation ? qcs.slerp(alpha,qo) : qc;

						//Always calculate the slerp, even if it's that not efficient
						add_pose(travel_time(length, velocity), pose_from_eigen(pc, qs));
					}
				} else {
					ROS_ERROR("Could not load radius and theta for arc segment (%i)", i);
					error = true;
				}
			} else {
				ROS_ERROR("Unkown type (%s) for segment %i", seg_type.c_str(), i);
				error = true;
			}
		} else {
			ROS_ERROR("Could not load height or velocity info for segment %i", i);
			error = true;
		}

		i++;
	}

	if(!error) {
		ROS_INFO("Path generated with %i segments!", i);
	}

	if(i == 0)
		ROS_ERROR("Could not load any segments!");

	return (i > 0) & !error;
}

void Paths::add_pose(const ros::Duration dt, const geometry_msgs::Pose pose) {
	geometry_msgs::PoseStamped pose_out;

	running_stamp_ += dt;

	pose_out.header.frame_id = param_frame_id_;
	pose_out.header.seq = running_seq_;
	pose_out.header.stamp = running_stamp_;

	pose_out.pose = pose;

	msg_out_.poses.push_back(pose_out);
	running_seq_++;
}

geometry_msgs::Pose Paths::pose_from_eigen(const Eigen::Vector3d p, const Eigen::Quaterniond q) {
	geometry_msgs::Pose pose;
	Eigen::Quaterniond qc = q.normalized();

	pose.position.x = p.x();
	pose.position.y = p.y();
	pose.position.z = p.z();
	pose.orientation.w = qc.w();
	pose.orientation.x = qc.x();
	pose.orientation.y = qc.y();
	pose.orientation.z = qc.z();

	return pose;
}


Eigen::Vector3d Paths::vector_from_doubles(std::vector<double> &a) {
	ROS_ASSERT_MSG( (a.size() == 3), "Vector3 size (%li) must be 3", a.size());

	return Eigen::Vector3d(a[0], a[1], a[2]);
}

Eigen::Quaterniond Paths::quaternion_from_doubles(std::vector<double> &a) {
	ROS_ASSERT_MSG( (a.size() == 4), "Quaternion size (%li) must be 4", a.size());

	return Eigen::Quaterniond(a[0], a[1], a[2], a[3]).normalized();
}

ros::Duration Paths::travel_time(const double len, const double vel) {
	return ros::Duration(len / vel);
}







/*
bool success = false;
nav_msgs::Path msg_out;
double time_step = param_duration_ / param_resolution_;

//Find the path to generate
if(param_path_type_ == "line ") {

	success = true;
} else if (param_path_type_ == "points") {

	success = true;
} else if (param_path_type_ == "square") {

	success = true;
} else if(param_path_type_ == "circle") {
	geometry_msgs::PoseStamped p;
	p.header.frame_id = param_frame_id_;

	p.pose.position.z = param_path_height_;
	p.pose.orientation.w = 1.0;
	p.pose.orientation.x = 0.0;
	p.pose.orientation.y = 0.0;
	p.pose.orientation.z = 0.0;

	double rot_step = 2*M_PI/(param_resolution_);
	double r = 0.0;

	//Fill in the rest of the path
	if(nh_.getParam("path/radius", r)) {
		for(int i=0; i<=(param_resolution_); i++) {
			p.header.seq = i;
			p.header.stamp = ros::Time(0) + ros::Duration((i)*time_step);

			p.pose.position.x = r*std::cos(rot_step*i);
			p.pose.position.y = r*std::sin(rot_step*i);

			msg_out.poses.push_back(p);
		}

		success = true;
	} else {
		ROS_ERROR("Type 'circle' needs parameter 'path/radius'");
	}
} else if (param_path_type_ == "figure_8") {


	success = true;
} else {
	error = true;
	ROS_ERROR("Path type (%s) is not supported.", param_path_type_.c_str());
}

if(success) {
	msg_out.header.frame_id = param_frame_id_;
	msg_out.header.stamp = ros::Time::now();

	pub_path_.publish(msg_out);

	ROS_INFO("Path generated!");
} else {
	ROS_ERROR("A path was unable to be generated.");
	error = true;
		}
*/