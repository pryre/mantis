#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <mantis_joints/JointGoalsConfig.h>
#include <mantis_joints/mantis_joints.h>

#include <mantis_msgs/JointTrajectoryGoal.h>
#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/PathProgress.h>

MantisJoints::MantisJoints() :
	nh_(),
	nhp_("~"),
	p_(nh_),
	tracked_input_(MantisJoints::LastJointReference::Undefined),
	param_frame_id_("mantis_uav"),
	param_update_rate_(50.0),
	param_configuration_stamp_(0),
	current_discrete_point_(0),
	spline_start_(0),
	spline_duration_(0),
	spline_joint_stamp_(0),
	spline_configuration_stamp_(0),
	dyncfg_joint_goals_(ros::NodeHandle(nhp_, "direct_position_control")) {

	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	dyncfg_joint_goals_.setCallback(boost::bind(&MantisJoints::callback_cfg_joint_goals, this, _1, _2));

	if( p_.wait_for_params() ) {
		sub_discrete_progress_ = nhp_.subscribe<contrail_msgs::PathProgress>( "reference/discrete_progress", 10, &MantisJoints::callback_discrete_progress, this );
		sub_cubic_spline_ = nhp_.subscribe<contrail_msgs::CubicSpline>( "reference/spline", 10, &MantisJoints::callback_cubic_spline, this );
		sub_joint_list_ = nhp_.subscribe<mantis_msgs::JointTrajectoryList>( "reference/joint_setpoints", 10, &MantisJoints::callback_joint_list, this );

		configure_publishers();

		timer_joints_ = nhp_.createTimer(ros::Duration(1.0/param_update_rate_), &MantisJoints::callback_joints, this );

		ROS_INFO_ONCE("Mantis joint planner loaded!");
	} else {
		ROS_WARN("Mantis joint planner shutting down.");
		ros::shutdown();
	}
}

MantisJoints::~MantisJoints( void ) {
}

void MantisJoints::configure_publishers( void ) {
	if(tracked_input_ != MantisJoints::LastJointReference::Undefined ) {
		param_configuration_stamp_ = p_.time_configuration_change();

		pub_joints_.clear();
		for(int i=0; i<p_.get_dynamic_joint_num(); i++) {
			pub_joints_.push_back(ros::Publisher());
			pub_joints_.back() = nhp_.advertise<mantis_msgs::JointTrajectoryGoal>("command/joint" + std::to_string(i), 10);
		}
	}
}

void MantisJoints::callback_joints(const ros::TimerEvent& e) {
	//Double check the configuration hasn't changed
	if( param_configuration_stamp_ != p_.time_configuration_change() )
		configure_publishers();

	//Loop through each dynamic joint
	for(int i=0; i<p_.get_dynamic_joint_num(); i++) {
		bool success = false;

		mantis_msgs::JointTrajectoryGoal msg_out;
		msg_out.header.frame_id = param_frame_id_;
		msg_out.header.stamp = e.current_real;

		if(tracked_input_ == MantisJoints::LastJointReference::DirectInput) {
			success = get_direct_input_reference(msg_out.position, msg_out.velocity, i);
		} else if(tracked_input_ == MantisJoints::LastJointReference::DiscreteProgress) {
			success = get_discrete_progress_reference(msg_out.position, msg_out.velocity, i);
		} else if(tracked_input_ == MantisJoints::LastJointReference::CubicSpline) {
			success = get_cubic_spline_reference(msg_out.position, msg_out.velocity, i, e.current_real);
		} //Undefined is already covered

		if(success)
			pub_joints_[i].publish(msg_out);
	}
}

//XXX:	This is more of a hacky way to give the direct input.
//		A better way would be to have a subscriber and a proper rqt app
//		that is capable of dynamically displaying the number of joints
//		using the parameter description
void MantisJoints::callback_cfg_joint_goals(mantis_joints::JointGoalsConfig &config, uint32_t level) {
	joint_list_.header.frame_id = param_frame_id_;
	joint_list_.header.stamp = ros::Time::now();

	joint_list_.num_joints = 8;
	joint_list_.num_points = 1;

	joint_list_.joint_interp_points.clear();
	joint_list_.joint_interp_points.push_back(config.joint1);
	joint_list_.joint_interp_points.push_back(config.joint2);
	joint_list_.joint_interp_points.push_back(config.joint3);
	joint_list_.joint_interp_points.push_back(config.joint4);
	joint_list_.joint_interp_points.push_back(config.joint5);
	joint_list_.joint_interp_points.push_back(config.joint6);
	joint_list_.joint_interp_points.push_back(config.joint7);
	joint_list_.joint_interp_points.push_back(config.joint8);

	tracked_input_ = MantisJoints::LastJointReference::DirectInput;
}

bool MantisJoints::get_direct_input_reference( double &pos_ref, double &vel_ref, const unsigned int index ) {
	bool success = false;

	if( index < joint_list_.num_joints ) {
		//We only use a n*1 list for direct input, so directly extract reference
		pos_ref = joint_list_.joint_interp_points[index];
		vel_ref = 0.0;

		success = true;
	}

	return success;
}

bool MantisJoints::get_discrete_progress_reference( double &pos_ref, double &vel_ref, const unsigned int index ) {
	bool success = false;

	if( ( index < joint_list_.num_joints ) &&
		( current_discrete_point_ < joint_list_.num_points ) ) {

		pos_ref = joint_list_.joint_interp_points[index + current_discrete_point_*joint_list_.num_joints];
		vel_ref = 0.0;

		success = true;
	}

	return success;
}

bool MantisJoints::get_cubic_spline_reference( double &pos_ref, double &vel_ref, const unsigned int index, const ros::Time& tc ) {
	bool success = false;

	if( (spline_configuration_stamp_ != p_.time_configuration_change()) ||
		(spline_joint_stamp_ != joint_list_.header.stamp) ) {
		generate_splines();
	}

	if( index < spline_list_.size() ) {
		if( tc < spline_start_ ) {
			//If the spline hasn't started, just prepare starting point
			pos_ref = joint_list_.joint_interp_points[index];
			vel_ref = 0.0;
		} else if( tc > (spline_start_ + spline_duration_) ) {
			//The spline has ended, hold last point
			pos_ref = joint_list_.joint_interp_points[index + (joint_list_.num_points-1)*joint_list_.num_joints];
			vel_ref = 0.0;
		} else {
			//Track spline
			double u = normalize((tc - spline_start_).toSec(), 0.0, spline_duration_.toSec());	//Get normalized time
			ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");

			std::vector<tinyspline::real> q = spline_list_[index](u).result();
			pos_ref = q[0];

			//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
			double dt = 0.02;
			//Shorten time to ensure that 0.0<=u<=1.0 is preserved
			double ul = u - dt;
			double uh = u + dt;
			ul = (ul >= 0.0) ? ul : 0.0;
			uh = (uh <= 1.0) ? uh : 1.0;

			std::vector<tinyspline::real> qdl = spline_list_[index](ul).result();
			std::vector<tinyspline::real> qdh = spline_list_[index](uh).result();

			vel_ref = ( (qdh[0] - qdl[0]) / (2*dt) ) / spline_duration_.toSec();
		}

		success = true;
	}
	return success;
}

void MantisJoints::callback_discrete_progress( const contrail_msgs::PathProgress::ConstPtr& msg_in ) {
	current_discrete_point_ = msg_in->current;
	ROS_INFO("[JointPlanner] Setting joint configuration to %i", current_discrete_point_);

	tracked_input_ = MantisJoints::LastJointReference::DiscreteProgress;
}

void MantisJoints::callback_cubic_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in ) {
	ros::Time tc = ros::Time::now();

	if( check_msg_spline(*msg_in, tc) ) {
		//Input is valid, no do a check of the actual values
		ros::Time start_time = (msg_in->start_time == ros::Time(0)) ? tc : msg_in->start_time;

		if( ( start_time + msg_in->duration ) > tc ) {
			ROS_INFO("[JointPlanner] Syncronizing joint movement with spline reference");
			spline_start_ = start_time;
			spline_duration_ = msg_in->duration;

			tracked_input_ = MantisJoints::LastJointReference::CubicSpline;
		} else {
			ROS_WARN("[JointPlanner] Spline has already finished");
		}
	}
}

void MantisJoints::callback_joint_list( const mantis_msgs::JointTrajectoryList::ConstPtr& msg_in ) {
	if( ( msg_in->header.stamp > ros::Time(0) ) &&
		(msg_in->num_joints > 0) &&
		(msg_in->num_points > 0) &&
		(msg_in->joint_interp_points.size() == (msg_in->num_joints*msg_in->num_points) ) ) {

		joint_list_ = *msg_in;
	} else {
		ROS_WARN("Ignoring joint list");
	}
}

void MantisJoints::generate_splines( void ) {
	if( ( joint_list_.header.stamp > ros::Time(0) ) &&
		( joint_list_.num_points > 1) ) {

		ROS_INFO("[JointPlanner] Recalculating joint splines");

		spline_list_.clear();
		spline_configuration_stamp_ = p_.time_configuration_change();
		spline_joint_stamp_ = joint_list_.header.stamp;

		for(int i=0; i<p_.get_dynamic_joint_num(); i++) {
			std::vector<double> jl;
			for(int j=0; j<joint_list_.num_points; j++)
				jl.push_back(joint_list_.joint_interp_points[i + j*joint_list_.num_joints]);

			spline_list_.push_back(tinyspline::Utils::interpolateCubic(&jl, 1));
		}
	}
}

bool MantisJoints::check_msg_spline(const contrail_msgs::CubicSpline& spline, const ros::Time t ) {
	bool t_check = (spline.header.stamp != ros::Time(0)) && (spline.duration > ros::Duration(0));
	bool xvec_check = (spline.x.size() > 0);
	bool yvec_check = (spline.y.size() == spline.x.size());
	bool zvec_check = (spline.z.size() == spline.x.size());
	bool rvec_check = (spline.yaw.size() == spline.x.size());

	return (t_check && xvec_check && yvec_check && zvec_check && rvec_check);
}
