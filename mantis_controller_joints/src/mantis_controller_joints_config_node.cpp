#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <dynamic_reconfigure/server.h>
#include <mantis_controller_joints/JointGoalsConfig.h>

#include <math.h>
#include <string>

class JointConfig {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;

		ros::Publisher pub_joint_state_;
		ros::Publisher pub_joint1_;
		ros::Publisher pub_joint2_;

		double sp_j1_;
		double sp_j2_;

		dynamic_reconfigure::Server<mantis_controller_joints::JointGoalsConfig> dyncfg_joints_;

		std::string param_frame_id_;
		double param_rate_;

	public:
		JointConfig( void );

		~JointConfig( void );

		void callback_cfg_joints(mantis_controller_joints::JointGoalsConfig &config, uint32_t level);
		void callback_timer(const ros::TimerEvent& e);
};

JointConfig::JointConfig() :
	nh_(),
	nhp_("~"),
	param_frame_id_("map"),
	param_rate_(20.0),
	dyncfg_joints_(ros::NodeHandle(nh_, "joint_positions")) {

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("update_rate", param_rate_, param_rate_);

	sp_j1_ = 0.0;
	sp_j2_ = 0.0;

	pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("command/joints", 10);
	pub_joint1_ = nh_.advertise<std_msgs::Float64>("command/joint/1", 10);
	pub_joint2_ = nh_.advertise<std_msgs::Float64>("command/joint/2", 10);

	dyncfg_joints_.setCallback(boost::bind(&JointConfig::callback_cfg_joints, this, _1, _2));
	timer_ = nhp_.createTimer(ros::Duration(1.0/param_rate_), &JointConfig::callback_timer, this );

	ROS_INFO("Robot joint config loaded");
}

JointConfig::~JointConfig() {
}


void JointConfig::callback_cfg_joints(mantis_controller_joints::JointGoalsConfig &config, uint32_t level) {
	sp_j1_ = config.joint1;
	sp_j2_ = config.joint2;
}


void JointConfig::callback_timer(const ros::TimerEvent& e) {
	std_msgs::Float64 msg_out_j1;
	std_msgs::Float64 msg_out_j2;
	sensor_msgs::JointState msg_out_js;
	msg_out_js.header.stamp = e.current_real;
	msg_out_js.header.frame_id = param_frame_id_;

	msg_out_j1.data = sp_j1_;
	msg_out_j2.data = sp_j2_;

	msg_out_js.name.push_back("joint_shoulder");
	msg_out_js.name.push_back("joint_elbow");
	msg_out_js.position.push_back(sp_j1_);
	msg_out_js.position.push_back(sp_j2_);

	pub_joint1_.publish(msg_out_j1);
	pub_joint2_.publish(msg_out_j2);
	pub_joint_state_.publish(msg_out_js);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_config");
	JointConfig jc;

	ros::spin();

	return 0;
}
