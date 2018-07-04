#include <ros/ros.h>

#include <dh_parameters/JointDescription.h>
#include <mantis_msgs/BodyInertial.h>
#include <mantis_msgs/Params.h>

#include <mantis_description/param_server.h>

MantisParamServer::MantisParamServer( void ) :
	nh_(),
	pwm_min_(1000),
	pwm_max_(2000),
	motor_kv_(0.0),
	rpm_thrust_m_(0.0),
	rpm_thrust_c_(0.0),
	motor_drag_max_(0.0),
	la_(0.0),
	body_num_(1) {

	bool ready = false;

	//Wait for ros time before initializing the servers
	while(ros::ok() && (!ready) ) {
		ready = ros::Time::now() != ros::Time(0);
		ros::spinOnce();
		ros::Rate(5).sleep();
	}

	if(ready) {
		pub_params_ = nh_.advertise<mantis_msgs::Params>("params", 1, true);
		srv_reload_ = nh_.advertiseService("reload_params", &MantisParamServer::reload, this);

		update();

		ROS_INFO("Mantis param server started!");
	}
}

MantisParamServer::~MantisParamServer() {
}


bool MantisParamServer::ok( void ) {
	return ( load_time_ != ros::Time(0) );
}

void MantisParamServer::update( void ) {
	load();

	pub_params_.publish(get_params());
}

mantis_msgs::Params MantisParamServer::get_params( void ) {
	mantis_msgs::Params p;

	p.header.stamp = load_time_;
	p.header.frame_id = "mantis_uav";

	p.airframe_type = airframe_type_;
	p.pwm_min = pwm_min_;
	p.pwm_max = pwm_max_;
	p.base_arm_length = la_;
	p.motor_kv = motor_kv_;
	p.rpm_thrust_m = rpm_thrust_m_;
	p.rpm_thrust_c = rpm_thrust_c_;
	p.motor_drag_max = motor_drag_max_;

	p.bodies = bodies_;
	p.joints = joints_;

	return p;
};

bool MantisParamServer::reload(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
	update();

	return true;
}

void MantisParamServer::load( void ) {
	load_time_ = ros::Time::now();

	ROS_INFO("--== Loading ControllerAug Parameters ==--");

	nh_.param("motor/arm_len", la_, la_);
	nh_.param("motor/kv", motor_kv_, motor_kv_);
	nh_.param("motor/rpm_thrust_curve/m", rpm_thrust_m_, rpm_thrust_m_);
	nh_.param("motor/rpm_thrust_curve/c", rpm_thrust_c_, rpm_thrust_c_);
	nh_.param("motor/drag_max", motor_drag_max_, motor_drag_max_);

	nh_.param("airframe", airframe_type_, airframe_type_);

	nh_.param("pwm/min", pwm_min_, pwm_min_);
	nh_.param("pwm/max", pwm_max_, pwm_max_);

	//TODO: Should be loaded dynamically
	nh_.param("body/num", body_num_, body_num_);
	bodies_.clear();
	for(int i=0; i<body_num_; i++) {
		mantis_msgs::BodyInertial bi;

		if( nh_.getParam( "body/b" + std::to_string(i) + "/mass/m", bi.mass) ) {
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/Ixx", bi.Ixx);
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/Ixy", bi.Ixx);
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/Ixz", bi.Ixx);
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/Iyy", bi.Iyy);
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/Iyz", bi.Iyy);
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/Izz", bi.Izz);
			nh_.getParam( "body/b" + std::to_string(i) + "/mass/com", bi.com);

			bodies_.push_back(bi);
		} else {
			//Could not find any more valid links defined, give up
			break;
		}
	}

	//Load in the link definitions
	joints_.clear();
	for(int i=0; i<body_num_; i++) {
		dh_parameters::JointDescription jd;

		if( nh_.getParam( "body/b" + std::to_string(i) + "/link/type", jd.type) ) {
			nh_.getParam( "body/b" + std::to_string(i) + "/link/name", jd.name);

			nh_.getParam( "body/b" + std::to_string(i) + "/link/d", jd.d);
			nh_.getParam( "body/b" + std::to_string(i) + "/link/t", jd.t);
			nh_.getParam( "body/b" + std::to_string(i) + "/link/r", jd.r);
			nh_.getParam( "body/b" + std::to_string(i) + "/link/a", jd.a);

			nh_.getParam( "body/b" + std::to_string(i) + "/link/q", jd.q);
			nh_.getParam( "body/b" + std::to_string(i) + "/link/beta", jd.beta);

			joints_.push_back(jd);
		} else {
			//Could not find any more valid links defined, give up
			break;
		}
	}

	ROS_INFO("motor:");
	ROS_INFO("  len: %0.4f", la_);
	ROS_INFO("  kv: %0.4f", motor_kv_);
	ROS_INFO("  T = %0.4fxRPM + %0.4f", rpm_thrust_m_, rpm_thrust_c_);
	ROS_INFO("  Dmax = %0.4f", motor_drag_max_);

	ROS_INFO("airframe_type: %s", airframe_type_.c_str());
	if( (airframe_type_ != "quad_x4") &&
		(airframe_type_ != "quad_p4") &&
		(airframe_type_ != "hex_x6") &&
		(airframe_type_ != "hex_x6") &&
		(airframe_type_ != "octo_x8") &&
		(airframe_type_ != "octo_p8") ) {

		ROS_WARN("Specified airframe type may not be supported!");
	}

	ROS_INFO("pwm: [%i, %i]", pwm_min_, pwm_max_);

	ROS_INFO_STREAM("bodies: " << body_num_);
	ROS_INFO_STREAM("  inertials: " << bodies_.size());
	ROS_INFO_STREAM("  joints: " << joints_.size());

	if( (bodies_.size() != body_num_) || (joints_.size() != body_num_) )
		ROS_WARN("Number of loaded joints and bodies do not match, some params may be invalid");

	ROS_INFO("--== ControllerAug Parameters Loaded ==--");
}

