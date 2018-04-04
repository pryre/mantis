#include <ros/ros.h>

#include <mantis_paths/path_extract.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>

PathExtract::PathExtract( ros::NodeHandle *nh, Eigen::Affine3d init_pose ) :
	nh_(nh),
	have_path_(false),
	path_hint_(1),
	latest_g_(init_pose) {

	sub_path_ = nh_->subscribe<nav_msgs::Path>( "reference/path", 10, &PathExtract::callback_path, this );
}

PathExtract::~PathExtract( void ) {
}

bool PathExtract::received_valid_path( void ) {
	return have_path_;
}

void PathExtract::set_latest(const Eigen::Vector3d &p, const Eigen::Quaterniond &q) {
	latest_g_.translation() << p;
	latest_g_.linear() << q.normalized().toRotationMatrix();
}

bool PathExtract::get_ref_state(Eigen::Affine3d &g_c, Eigen::Vector3d &v_c, const ros::Time tc) {
	bool success = false;

	//Create temporary storage
	Eigen::Affine3d g_t = Eigen::Affine3d::Identity();
	Eigen::Vector3d v_t = Eigen::Vector3d::Zero();

	try {
		//If we have recieved a path message
		if(p_c_.header.stamp > ros::Time(0)) {
			ros::Duration duration_start = p_c_.poses.front().header.stamp - ros::Time(0);
			ros::Duration duration_end = p_c_.poses.back().header.stamp - ros::Time(0);

			ros::Time ts = p_c_.header.stamp + duration_start;
			ros::Time tf = p_c_.header.stamp + duration_end;
			ros::Duration td = duration_end - duration_start;

			//If the current time is within the path time, follow the path
			if((tc >= ts) && (tc < tf)) {
				//Check if we should use the path hint
				ros::Duration dth = p_c_.poses[path_hint_ - 1].header.stamp - ros::Time(0);
				int p = (tc > (p_c_.header.stamp + dth) ) ? path_hint_ : 1;

				//Find the next point in the path
				bool found_seg = false;

				while( ( !found_seg ) && ( p < p_c_.poses.size() ) ) {
					ros::Duration d_l = p_c_.poses[p - 1].header.stamp - ros::Time(0);
					ros::Duration d_n = p_c_.poses[p].header.stamp - ros::Time(0);

					if( (tc >= p_c_.header.stamp + d_l ) &&
						(tc < p_c_.header.stamp + d_n ) ) {
						//We have the right index
						found_seg = true;
					} else {
						//Keep looping
						p++;
					}
				}

				//Calculate the reference state as long as we found the segment
				if(found_seg) {
					//Get the last and next points
					Eigen::Affine3d g_l = affine_from_msg(p_c_.poses[p - 1].pose);
					Eigen::Affine3d g_n = affine_from_msg(p_c_.poses[p].pose);
					ros::Duration t_l = p_c_.poses[p - 1].header.stamp - ros::Time(0);
					ros::Duration t_n = p_c_.poses[p].header.stamp - ros::Time(0);
					ros::Time t_lt = p_c_.header.stamp + t_l;
					double dts = (t_n - t_l).toSec();	//Time to complete this segment
					double da = (tc - t_lt).toSec();	//Time to alpha
					double alpha = da / dts;

					//Position goal
					g_t.translation() << vector_lerp(g_l.translation(), g_n.translation(), alpha);
					Eigen::Quaterniond ql_sp(g_l.linear());
					Eigen::Quaterniond qc_sp = ql_sp.slerp(alpha, Eigen::Quaterniond(g_n.linear()));
					g_t.linear() << qc_sp.toRotationMatrix();

					//Velocity goal
					v_t = (g_n.translation() -  g_l.translation()) / dts;

					//Record the index we ues so we can start the time checks there next loop
					path_hint_ = p;

					success = true;
				}
			}
		}
	} catch( std::runtime_error &e) {
		//May get errors if timestamp is malformed
		ROS_ERROR("Exception: [%s]", e.what());
		ROS_WARN("Invalidating current path!");

		p_c_.header.stamp = ros::Time(0);
	}

	if(success) {
		//Update latest setpoint in case we need to hold lastest position
		latest_g_ = g_t;

		g_c = g_t;
		v_c = v_t;
	} else {
		//Something went wrong, fall back to the last know safe values
		g_c = latest_g_;
		v_c = Eigen::Vector3d::Zero();

		reset_hinting();
	}

	return success;
}

void PathExtract::callback_path(const nav_msgs::Path::ConstPtr& msg_in) {
	//If there is at least 2 poses in the path
	if(msg_in->poses.size() > 1) {
		//If at least the very last timestamp is in the future, accept path
		if( ( msg_in->header.stamp + ( msg_in->poses.back().header.stamp - ros::Time(0) ) ) > ros::Time::now() ) {
			ROS_INFO("Recieved new path!");
			p_c_ = *msg_in;
			have_path_ = true;
			reset_hinting();
		} else {
			ROS_WARN("Rejecting path, timestamps are too old.");
		}
	} else {
		ROS_WARN("Rejecting path, must be at least 2 poses.");
	}
}

void PathExtract::reset_hinting(void) {
	path_hint_ = 1;	//Reset path hinting
}

Eigen::Vector3d PathExtract::position_from_msg(const geometry_msgs::Point p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond PathExtract::quaternion_from_msg(const geometry_msgs::Quaternion q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d PathExtract::affine_from_msg(const geometry_msgs::Pose pose) {
		Eigen::Affine3d a;

		a.translation() << position_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}

Eigen::Vector3d PathExtract::vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha) {
  return ((1.0 - alpha) * a) + (alpha * b);
}
