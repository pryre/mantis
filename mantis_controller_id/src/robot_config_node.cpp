#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <mantis_controller_id/JointGoalsConfig.h>

#include <math.h>

class RobotConfig {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;

		ros::Publisher pub_path_;
		ros::Publisher pub_joints_;

		ros::Subscriber sub_hover_goal_;
		dynamic_reconfigure::Server<mantis_controller_id::JointGoalsConfig> dyncfg_joints_;

		std::string param_frame_id_;

	public:
		RobotConfig( void );

		~RobotConfig( void );

		void callback_hover_goal(const geometry_msgs::PoseStamped::ConstPtr& msg_in);
		void callback_cfg_joints(mantis_controller_id::JointGoalsConfig &config, uint32_t level);
};



RobotConfig::RobotConfig() :
	nh_("~"),
	param_frame_id_("map"),
	dyncfg_joints_(ros::NodeHandle(nh_, "joints")) {

	pub_path_ = nh_.advertise<nav_msgs::Path>("goal/path", 10);
	pub_joints_ = nh_.advertise<std_msgs::Float64MultiArray>("goal/joints", 10);

	sub_hover_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>( "pose", 10, &RobotConfig::callback_hover_goal, this );
	dyncfg_joints_.setCallback(boost::bind(&RobotConfig::callback_cfg_joints, this, _1, _2));

	ROS_INFO("Robot pose configured");
}

RobotConfig::~RobotConfig() {
}


void RobotConfig::callback_cfg_joints(mantis_controller_id::JointGoalsConfig &config, uint32_t level) {
	std_msgs::Float64MultiArray msg_out;

	msg_out.layout.dim.reserve(1);
	msg_out.layout.dim[0].size = 2;
	msg_out.layout.dim[0].stride = 1;

	msg_out.data.push_back(config.joint1);
	msg_out.data.push_back(config.joint2);

	pub_joints_.publish(msg_out);
}


void RobotConfig::callback_hover_goal(const geometry_msgs::PoseStamped::ConstPtr &msg_in) {
	if(msg_in->header.frame_id == param_frame_id_) {
		nav_msgs::Path msg_out;
		ros::Time stamp = ros::Time::now();

		msg_out.header.stamp = stamp;
		msg_out.header.frame_id = param_frame_id_;

		msg_out.poses.push_back(*msg_in);
		msg_out.poses.push_back(*msg_in);
		msg_out.poses[0].header.stamp = ros::Time(0);
		msg_out.poses[1].header.stamp = ros::Time(0) + ros::Duration(1);

		pub_path_.publish(msg_out);

		ROS_INFO("Goal pose sent: [%0.2f, %0.2f, %0.2f]; [%0.2f, %0.2f, %0.2f, %0.2f]",
				  msg_in->pose.position.x,
				  msg_in->pose.position.y,
				  msg_in->pose.position.z,
				  msg_in->pose.orientation.w,
				  msg_in->pose.orientation.x,
				  msg_in->pose.orientation.y,
				  msg_in->pose.orientation.z);
	} else {
		ROS_ERROR("Goal frame_id (%s) does not match frame_id parameter (%s)", msg_in->header.frame_id.c_str(), param_frame_id_.c_str());
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_config");
	RobotConfig pt;

	ros::spin();

	return 0;
}
