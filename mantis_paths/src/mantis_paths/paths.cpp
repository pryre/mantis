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

			bool use_qo = false;
			Eigen::Quaterniond qo = Eigen::Quaterniond::Identity();
			std::vector<double> orientation;
			if( nh_.getParam("path/s" + std::to_string(i) + "/orientation", orientation) ) {
				qo = quaternion_from_doubles(orientation);
				use_qo = true;
			}

			if(seg_type == "line") {
				double length = 0.0;

				if( nh_.getParam("path/s" + std::to_string(i) + "/length", length) ) {
					pc += qc.toRotationMatrix()*Eigen::Vector3d(length, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, height);

					double dist = Eigen::Vector3d(length, 0.0, height).norm();	//Segment distance
					add_pose(travel_time(dist, velocity), pose_from_eigen(pc, (use_qo ? qo : qc) ) );
				} else {
					ROS_ERROR("Could not load length for line segment (%i)", i);
					error = true;
				}
			} else if(seg_type == "arc") {
				double radius = 0.0;
				double theta = 0.0;

				if( nh_.getParam("path/s" + std::to_string(i) + "/radius", radius) &&
					nh_.getParam("path/s" + std::to_string(i) + "/theta", theta) ) {

					double dtheta = theta / param_arc_res_;
					double dx = std::fabs(radius*dtheta);
					double dz = height / param_arc_res_;

					//Go through each subsegment
					for(int j=1; j<=param_arc_res_; j++) {
						double alpha = (double)j / (double)param_arc_res_;
						pc += qc.toRotationMatrix()*Eigen::Vector3d(dx, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, dz);
						qc *= Eigen::Quaterniond(Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitZ()));

						double dist = Eigen::Vector3d(dx, 0.0, dz).norm();	//Segment distance
						add_pose(travel_time(dist, velocity), pose_from_eigen(pc, (use_qo ? qo : qc) ) );
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
