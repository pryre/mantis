#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <mantis_router_joints/joint.h>
#include <mantis_router_joints/JointMovementAction.h>

#include <mantis_msgs/JointTrajectoryGoal.h>

namespace MantisRouterJoints {

Joint::Joint( const ros::NodeHandle& nh, std::string joint_name ) :
	nh_(ros::NodeHandle(nh, joint_name)),
	spline_start_(0),
	spline_duration_(0),
	spline_in_progress_(false),
	as_(nh, joint_name, false) {
    //as_(nh, joint_name, boost::bind(&Joint::action_goal_cb, this, _1), false) {

	name_ = joint_name;

	output_.header.frame_id = name_;
	output_.header.stamp = ros::Time(0);
	output_.position = 0;
	output_.velocity = 0;

	pub_output_ = nh_.advertise<mantis_msgs::JointTrajectoryGoal>( "command", 10);

	as_.start();
}

Joint::~Joint() {
}

//void Joint::action_goal_cb( const mantis_router_joints::JointMovementGoalConstPtr &goal ) {
void Joint::set_action_goal( void ) {
	mantis_router_joints::JointMovementGoal goal = *(as_.acceptNewGoal());

	if( (goal.duration > ros::Duration(0) ) &&
		(goal.positions.size() >= 2) ) {

		spline_in_progress_ = true;

		spline_start_ = ( goal.start == ros::Time(0) ) ? ros::Time::now() : goal.start;
		spline_duration_ = goal.duration;

		spline_ = tinyspline::Utils::interpolateCubic(&goal.positions, 1);

		//Smooth out control points to give a nicer fit
		std::vector<tinyspline::real> ctrlp = spline_.controlPoints();
		ROS_ASSERT_MSG( ctrlp.size() >= 4, "Number of control points is <4 (%i)", (int)ctrlp.size());
		ctrlp.at(1) = ctrlp.front();
		ctrlp.at(ctrlp.size() - 2) = ctrlp.back();
		spline_.setControlPoints(ctrlp);
		for(int i=0; i<ctrlp.size(); i++) {
			ROS_INFO("%s(i): %0.4f", name_.c_str(), ctrlp[i]);
		}
		splined_ = spline_.derive();

		spline_pos_start_ = goal.positions.front();
		spline_pos_end_ = goal.positions.back();

		ROS_INFO( "Router %s: creating spline connecting %i points", name_.c_str(), (int)goal.positions.size() );
	} else {
		spline_in_progress_ = false;
		as_.setAborted();

		ROS_ERROR( "Router %s: at least 2 positions must be specified (%i), and duration must be >0 (%0.4f)", name_.c_str(), (int)goal.positions.size(), goal.duration.toSec() );
	}
}

void Joint::get_spline_reference(double& pos, double& vel, const double u) const {
	// x values need to be scaled down in extraction as well.
	ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");

	std::vector<tinyspline::real> vp = spline_(u).result();
	std::vector<tinyspline::real> vv = splined_(u).result();

	pos = vp[0];
	vel = vv[0];
	/*
	//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
	double dt = 0.02;
	//Shorten time to ensure that 0.0<=u<=1.0 is preserved
	double ul = u - dt;
	double uh = u + dt;
	ul = (ul >= 0.0) ? ul : 0.0;
	uh = (uh <= 1.0) ? uh : 1.0;

	std::vector<tinyspline::real> vdl = spline_(ul).result();
	std::vector<tinyspline::real> vdh = spline_(uh).result();

	vel = (vdh[0] - vdl[0]) / (2*dt);
	*/
}

void Joint::update( ros::Time tc ) {
	//Check for a new goal
	if( as_.isNewGoalAvailable() ) {
		set_action_goal();
	}

	// Check that preempt has not been requested by the client
	if( as_.isPreemptRequested() ) {
		ROS_INFO("Router %s: Preempted", name_.c_str());
		as_.setPreempted();
		spline_in_progress_ = false;
	}

	//Don't output unless some reference has been set
	if( spline_start_ > ros::Time(0) ) {
		//If in progress, calculate the lastest reference
		if( spline_in_progress_ ) {
			if( tc < spline_start_ ) {
				//Have no begun, stay at start position
				output_.position = spline_pos_start_;
				output_.velocity = 0;

				mantis_router_joints::JointMovementFeedback feedback;
				feedback.progress = -1.0;
				feedback.position = output_.position;
				feedback.velocity = output_.velocity;

				as_.publishFeedback(feedback);
			} else if( tc <= (spline_start_ + spline_duration_) ) {
				double t_norm = normalize((tc - spline_start_).toSec(), 0.0, spline_duration_.toSec());
				double npos = 0.0;
				double nvel = 0.0;

				get_spline_reference(npos, nvel, t_norm);

				output_.position = npos;
				output_.velocity = nvel / spline_duration_.toSec();

				mantis_router_joints::JointMovementFeedback feedback;
				feedback.progress = t_norm;
				feedback.position = output_.position;
				feedback.velocity = output_.velocity;

				as_.publishFeedback(feedback);
			} else {
				//We just finished, so send a result
				output_.position = spline_pos_end_;
				output_.velocity = 0;

				mantis_router_joints::JointMovementResult result;
				result.position_final = output_.position;
				as_.setSucceeded(result);

				spline_in_progress_ = false;
				ROS_INFO( "Router %s: action finished", name_.c_str() );
			}
		}

		output_.header.stamp = tc;
		pub_output_.publish(output_);
	}
}

}
