#pragma once

#include <ros/ros.h>
#include <dh_parameters/JointDescription.h>
#include <mantis_msgs/BodyInertial.h>
#include <mantis_msgs/Params.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>

class MantisParamServer {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_params_;

		ros::ServiceServer srv_reload_;

		ros::Time load_time_;

		std::string airframe_type_;
		int pwm_min_;
		int pwm_max_;

		double la_;
		//int motor_num_;
		//double motor_thrust_max;
		double motor_kv_;
		double rpm_thrust_m_;
		double rpm_thrust_c_;
		double motor_drag_max_;

		int body_num_;
		std::vector<mantis_msgs::BodyInertial> bodies_;
		std::vector<dh_parameters::JointDescription> joints_;

	public:
		MantisParamServer( void );

		~MantisParamServer( void );

		void update( void );
		mantis_msgs::Params get_params( void );

		bool ok( void );

	private:
		void load( void );
		bool reload(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
};
