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

		mavros_msgs::State msg_state_;
		geometry_msgs::PoseStamped msg_pose_;
		sensor_msgs::JointState msg_joints_;

	public:
		PoseTest( void );

		~PoseTest( void );

		void callback(const ros::TimerEvent& e);
};



PoseTest::PoseTest() :
	nh_("~") {

	pub_state_ = nh_.advertise<mavros_msgs::State>("state", 10);
	pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
	pub_joints_ = nh_.advertise<sensor_msgs::JointState>("joints", 10);

	msg_state_.header.frame_id = "map";
	msg_state_.connected = true;
	msg_state_.armed = true;
	msg_state_.guided = true;
	msg_state_.mode = "OFFBOARD";

	msg_pose_.header.frame_id = "map";
	msg_pose_.pose.position.x = 0.0;
	msg_pose_.pose.position.y = 0.0;
	msg_pose_.pose.position.z = 2.0;
	msg_pose_.pose.orientation.x = 0.0;
	msg_pose_.pose.orientation.y = 0.0;
	msg_pose_.pose.orientation.z = 0.0;
	msg_pose_.pose.orientation.w = 1.0;

	msg_joints_.header.frame_id = "map";
	msg_joints_.name.push_back("shoulder");
	msg_joints_.name.push_back("elbow");
	msg_joints_.position.push_back(M_PI/4.0);
	msg_joints_.position.push_back(M_PI/4.0);

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

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_test");
	PoseTest pt;

	ros::spin();

	return 0;
}
