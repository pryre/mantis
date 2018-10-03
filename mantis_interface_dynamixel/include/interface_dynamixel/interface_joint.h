#pragma once

#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <string>

class InterfaceJoint {
	public:
		enum CurrentReference { Unset,
								Position,
								Velocity,
								Effort };

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_reference_pos_;
		ros::Subscriber sub_reference_vel_;
		ros::Subscriber sub_reference_eff_;

		std::string name_;

		InterfaceJoint::CurrentReference current_ref_;
		double ref_pos_;
		double ref_vel_;
		double ref_eff_;

		ros::Time ref_update_t_;
		ros::Duration ref_timeout_;

	public:
		InterfaceJoint( const ros::NodeHandle& nh, std::string joint_name, double ref_timeout );

		~InterfaceJoint( void );

		void callback_reference_pos( const std_msgs::Float64::ConstPtr& msg_in );
		void callback_reference_vel( const std_msgs::Float64::ConstPtr& msg_in );
		void callback_reference_eff( const std_msgs::Float64::ConstPtr& msg_in );

		bool get_reference( double& ref );
		CurrentReference get_reference_type( void );
		//double ref_type( void );
		//double ref_pos( void );
		//double ref_vel( void );
		//double ref_vel( void );
		const std::string& name( void );
};
