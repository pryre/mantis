#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <math.h>

class PoseTest {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;

		ros::Publisher pub_state_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_joints_;

		ros::Subscriber sub_hover_goal_;

		mavros_msgs::State msg_state_;
		geometry_msgs::PoseStamped msg_pose_;
		sensor_msgs::JointState msg_joints_;

		std::string param_frame_id_;
		double param_hover_height_;

	public:
		PoseTest( void );

		~PoseTest( void );

		void callback(const ros::TimerEvent& e);
		void callback_hover_goal(const geometry_msgs::PoseStamped::ConstPtr& msg_in);
};



PoseTest::PoseTest() :
	nh_("~"),
	param_frame_id_("map"),
	param_hover_height_(1.5) {

	pub_state_ = nh_.advertise<mavros_msgs::State>("state", 10);
	pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
	pub_joints_ = nh_.advertise<sensor_msgs::JointState>("joints", 10);

	sub_hover_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>( "hover_goal", 10, &PoseTest::callback_hover_goal, this );

	msg_state_.header.frame_id = param_frame_id_;
	msg_state_.connected = true;
	msg_state_.armed = true;
	msg_state_.guided = true;
	msg_state_.mode = "OFFBOARD";

	msg_pose_.header.frame_id = param_frame_id_;
	msg_pose_.pose.position.x = 0.0;
	msg_pose_.pose.position.y = 0.0;
	msg_pose_.pose.position.z = param_hover_height_;
	msg_pose_.pose.orientation.x = 0.0;
	msg_pose_.pose.orientation.y = 0.0;
	msg_pose_.pose.orientation.z = 0.0;
	msg_pose_.pose.orientation.w = 1.0;

	msg_joints_.header.frame_id = param_frame_id_;
	msg_joints_.name.push_back("shoulder");
	msg_joints_.name.push_back("elbow");
	msg_joints_.position.push_back(0.0);	//M_PI/4.0);
	msg_joints_.position.push_back(0.0);	//M_PI/4.0);

	timer_ = nh_.createTimer(ros::Duration(0.01), &PoseTest::callback, this );

	ROS_INFO("Publishing pose goals");
}

PoseTest::~PoseTest() {
}

void PoseTest::callback(const ros::TimerEvent& e) {
	msg_state_.header.stamp = e.current_real;
	msg_pose_.header.stamp = e.current_real;
	msg_joints_.header.stamp = e.current_real;

	pub_state_.publish(msg_state_);
	pub_pose_.publish(msg_pose_);
	pub_joints_.publish(msg_joints_);
}

void PoseTest::callback_hover_goal(const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
	if(msg_in->header.frame_id == param_frame_id_) {
		msg_pose_.pose = msg_in->pose;
		msg_pose_.pose.position.z = param_hover_height_;

		ROS_INFO("Goal pose set: [%0.2f, %0.2f, %0.2f]; [%0.2f, %0.2f, %0.2f, %0.2f]",
				  msg_pose_.pose.position.x,
				  msg_pose_.pose.position.y,
				  msg_pose_.pose.position.z,
				  msg_pose_.pose.orientation.w,
				  msg_pose_.pose.orientation.x,
				  msg_pose_.pose.orientation.y,
				  msg_pose_.pose.orientation.z);
	} else {
		ROS_ERROR("Goal frame_id (%s) does not match frame_id parameter (%s)", msg_in->header.frame_id.c_str(), param_frame_id_.c_str());
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_test");
	PoseTest pt;

	ros::spin();

	return 0;
}
