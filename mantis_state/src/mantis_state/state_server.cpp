#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <dh_parameters/JointDescription.h>
#include <mantis_msgs/State.h>
#include <mantis_description/se_tools.h>
#include <mantis_description/param_client.h>
#include <mantis_state/state_server.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>

MantisStateServer::MantisStateServer( void ) :
	nh_(),
	nhp_("~"),
	p_(nh_),
	param_use_odom_avel_(false),
	param_rate_(0.0),
	time_last_est_(0) {

	nhp_.param("use_odom_angular_velocity", param_use_odom_avel_, param_use_odom_avel_);
	nhp_.param("update_rate", param_rate_, param_rate_);

	if( p_.wait_for_params() ) {
		if(param_rate_ > 0.0) {
			//Current States
			g_ = Eigen::Affine3d::Identity();
			bv_ = Eigen::Vector3d::Zero();
			bw_ = Eigen::Vector3d::Zero();
			ba_ = Eigen::Vector3d::Zero();
			bwa_ = Eigen::Vector3d::Zero();

			//Additional state information
			voltage_ = 0.0;
			mav_ready_ = false;

			//Parameter-reliant states
			r_ = Eigen::VectorXd::Zero(p_.get_dynamic_joint_num());
			rd_ = Eigen::VectorXd::Zero(p_.get_dynamic_joint_num());
			rdd_ = Eigen::VectorXd::Zero(p_.get_dynamic_joint_num());

			sub_state_odom_ = nh_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &MantisStateServer::callback_state_odom, this );
			sub_state_battery_ = nh_.subscribe<sensor_msgs::BatteryState>( "state/battery", 10, &MantisStateServer::callback_state_battery, this );
			sub_state_joints_ = nh_.subscribe<sensor_msgs::JointState>( "state/joints", 10, &MantisStateServer::callback_state_joints, this );
			sub_state_imu_ = nh_.subscribe<sensor_msgs::Imu>( "state/imu_data", 10, &MantisStateServer::callback_state_imu, this );
			sub_state_mav_ = nh_.subscribe<mavros_msgs::State>( "state/mav_state", 10, &MantisStateServer::callback_state_mav, this );

			pub_state_ = nh_.advertise<mantis_msgs::State>("state", 10);
			timer_estimator_ = nhp_.createTimer(ros::Duration(1.0/param_rate_), &MantisStateServer::callback_estimator, this );

			ROS_INFO("Mantis state server running!");
		} else {
			ROS_FATAL("State server not started, no update rate given");
		}
	}
}

MantisStateServer::~MantisStateServer() {
}

void MantisStateServer::callback_estimator(const ros::TimerEvent& e) {
	if( ( msg_odom_tr_ != ros::Time(0) ) &&
		( msg_battery_tr_ != ros::Time(0) ) &&
		( msg_joints_tr_ != ros::Time(0) ) &&
		( msg_imu_tr_ != ros::Time(0) ) &&
		( msg_mav_state_tr_ != ros::Time(0) ) &&
		( p_.get_dynamic_joint_num() == r_.size() ) ) {

		//If we can get a dt
		if( time_last_est_ > ros::Time(0) ) {

			//XXX: TODO: Shold have some form of state estimation here!

			mantis_msgs::State state;
			state.header.stamp = e.current_real;
			state.header.frame_id = "map";
			state.child_frame_id = "mantis_uav";
			state.configuration_stamp = p_.time_configuration_change();

			state.pose = MDTools::pose_from_eig(g_);
			state.twist.linear = MDTools::vector_from_eig(bv_);
			state.twist.angular = MDTools::vector_from_eig(bw_);
			state.accel.linear = MDTools::vector_from_eig(ba_);
			state.accel.angular = MDTools::vector_from_eig(bwa_);

			//Some safety in case out joint configuration changes
			//This should make things wait until r_ is updated by the joiint callback
			state.r.resize(p_.get_dynamic_joint_num());
			state.rd.resize(p_.get_dynamic_joint_num());
			state.rdd.resize(p_.get_dynamic_joint_num());

			for(int i=0; i<p_.get_dynamic_joint_num(); i++) {
				state.r[i] = r_[i];
				state.rd[i] = rd_[i];
				state.rdd[i] = rdd_[i];
			}


			state.battery_voltage = voltage_;
			state.flight_ready = mav_ready_;

			//Publish the final state estimate
			pub_state_.publish(state);
		}

		time_last_est_ = e.current_real;
	}
}

void MantisStateServer::update_g( const Eigen::Affine3d &g ) {
	g_ = g;
}

void MantisStateServer::update_bw( const Eigen::Vector3d &bw ) {
	bw_ = bw;
}

void MantisStateServer::update_bw( const Eigen::Vector3d &bw, const double dt ) {
	bwa_ = (bw - bw_) / dt;

	update_bw(bw);
}

void MantisStateServer::update_bv( const Eigen::Vector3d &bv ) {
	bv_ = bv;
}

void MantisStateServer::update_ba( const Eigen::Vector3d &ba ) {
	ba_ = ba;
}

void MantisStateServer::update_r( const Eigen::VectorXd &r ) {
	r_ = r;
}

void MantisStateServer::update_rd( const Eigen::VectorXd &rd ) {
	rd_ = rd;
}

void MantisStateServer::update_rdd( const Eigen::VectorXd &rdd ) {
	rdd_ = rdd;
}

void MantisStateServer::update_voltage( const double voltage ) {
	voltage_ = voltage;
}

void MantisStateServer::update_mav_ready( const bool ready ) {
	mav_ready_ = ready;
}

void MantisStateServer::callback_state_odom(const nav_msgs::Odometry::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_odom_tr_).toSec();
	msg_odom_tr_ = msg_in->header.stamp;

	//Update pose
	Eigen::Affine3d g = MDTools::affine_from_msg(msg_in->pose.pose);
	update_g( g );

	//Update body velocities
	Eigen::Vector3d bv = MDTools::vector_from_msg(msg_in->twist.twist.linear);
	update_bv( bv );

	if( param_use_odom_avel_ ) {
		Eigen::Vector3d bw = MDTools::vector_from_msg(msg_in->twist.twist.angular);
		update_bw( bw, dt );
	}
}

void MantisStateServer::callback_state_battery(const sensor_msgs::BatteryState::ConstPtr& msg_in) {
	msg_battery_tr_ = msg_in->header.stamp;

	//XXX: Need to do this as the straight voltage reading is too slow
	double voltage = 0.0;
	for(int i=0; i<msg_in->cell_voltage.size(); i++) {
		voltage += msg_in->cell_voltage[i];
	}

	update_voltage( voltage );
}

void MantisStateServer::callback_state_imu(const sensor_msgs::Imu::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_imu_tr_).toSec();
	msg_imu_tr_ = msg_in->header.stamp;

	Eigen::Vector3d ba = MDTools::vector_from_msg(msg_in->linear_acceleration);
	update_ba( ba );

	Eigen::Vector3d bw = MDTools::vector_from_msg(msg_in->angular_velocity);
	update_bw( bw, dt );
}

void MantisStateServer::callback_state_mav(const mavros_msgs::State::ConstPtr& msg_in) {
	msg_mav_state_tr_ = msg_in->header.stamp;

	bool ready = msg_in->armed && (msg_in->mode == "OFFBOARD");

	update_mav_ready( ready );
}

void MantisStateServer::callback_state_joints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	double dt = (msg_in->header.stamp - msg_joints_tr_).toSec();

	Eigen::VectorXd r = Eigen::VectorXd::Zero(p_.get_dynamic_joint_num());
	Eigen::VectorXd rd = Eigen::VectorXd::Zero(p_.get_dynamic_joint_num());
	Eigen::VectorXd rdd = Eigen::VectorXd::Zero(p_.get_dynamic_joint_num());

	int jc = 0;	//Seperate counter is used to increment only for dynamic joints
	for(int i=0; i<p_.get_joint_num(); i++) {
		dh_parameters::JointDescription joint = p_.joint(i);

		if(joint.type != "static") {
			for(int j=0; j<msg_in->name.size(); j++) {
				//If we have the right joint
				if(joint.name == msg_in->name[j]) {
					r(jc) = msg_in->position[j];
					rd(jc) = msg_in->velocity[j];

					//Only update rdd if dt is acceptable
					if( (dt > 0.0) && (dt < 1.0) )
						rdd(jc) = (rd(jc) - rd_(jc)) / dt;	//TODO: FILTER?
				}
			}

			jc++;
		}
	}

	update_r(r);
	update_rd(rd);
	//Only update rdd if dt is acceptable
	if( (dt > 0.0) && (dt < 1.0) )
		update_rdd(rdd);

	msg_joints_tr_ = msg_in->header.stamp;
}
