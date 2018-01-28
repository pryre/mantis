#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <mantis_controller_id/JointGoalsConfig.h>

#include <math.h>

class RobotConfig {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;

		ros::Publisher pub_path_;
		ros::Publisher pub_joint1_;
		ros::Publisher pub_joint2_;

		ros::Subscriber sub_hover_goal_;
		dynamic_reconfigure::Server<mantis_controller_id::JointGoalsConfig> dyncfg_joints_;

		std::string param_frame_id_;
		double param_hover_height_;

	public:
		RobotConfig( void );

		~RobotConfig( void );

		void callback_hover_goal(const geometry_msgs::PoseStamped::ConstPtr& msg_in);
		void callback_cfg_joints(mantis_controller_id::JointGoalsConfig &config, uint32_t level);
};



RobotConfig::RobotConfig() :
	nh_("~"),
	param_frame_id_("map"),
	param_hover_height_(0.0),
	dyncfg_joints_(ros::NodeHandle(nh_, "joints")) {

	nh_.param("frame_id", param_frame_id_, param_frame_id_);
	nh_.param("hover_height", param_hover_height_, param_hover_height_);

	pub_path_ = nh_.advertise<nav_msgs::Path>("goal/path", 10);
	pub_joint1_ = nh_.advertise<std_msgs::Float64>("goal/joint1", 10);
	pub_joint2_ = nh_.advertise<std_msgs::Float64>("goal/joint2", 10);

	sub_hover_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>( "pose", 10, &RobotConfig::callback_hover_goal, this );
	dyncfg_joints_.setCallback(boost::bind(&RobotConfig::callback_cfg_joints, this, _1, _2));

	ROS_INFO("Robot pose configured");
}

RobotConfig::~RobotConfig() {
}


void RobotConfig::callback_cfg_joints(mantis_controller_id::JointGoalsConfig &config, uint32_t level) {
	std_msgs::Float64 msg_out_j1;
	std_msgs::Float64 msg_out_j2;

	msg_out_j1.data = config.joint1;
	msg_out_j2.data = config.joint2;

	pub_joint1_.publish(msg_out_j1);
	pub_joint2_.publish(msg_out_j2);
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
		msg_out.poses[0].pose.position.z = param_hover_height_;
		msg_out.poses[1].pose.position.z = param_hover_height_;

		pub_path_.publish(msg_out);

		ROS_INFO("Goal pose sent: [%0.2f, %0.2f, %0.2f]; [%0.2f, %0.2f, %0.2f, %0.2f]",
				  msg_in->pose.position.x,
				  msg_in->pose.position.y,
				  param_hover_height_,
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
