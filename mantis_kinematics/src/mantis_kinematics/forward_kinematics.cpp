/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#include <ros/ros.h>

#include <mantis_description/se_tools.h>
#include <mantis_kinematics/forward_kinematics.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>

#include <stdio.h>

// TODO: Add goal joints

ForwardKinematics::ForwardKinematics()
	: nh_()
	, nhp_( "~" )
	, p_( nh_ )
	, s_( nh_, p_ )
	, solver_( p_, s_ )
	, param_rate_( 30.0 )
	, param_do_end_effector_pose_( true )
	, param_do_prop_viz_( false )
	, prop_rate_( 0.8 )
	, tfBuffer_( ros::Duration( 20.0 ) ) {

	// Load parameters
	nhp_.param( "update_rate", param_rate_, param_rate_ );
	nhp_.param( "prop_viz", param_do_prop_viz_, param_do_prop_viz_ );
	nhp_.param( "armed_prop_velocity", prop_rate_, prop_rate_ );
	nhp_.param( "end_effector_pose", param_do_end_effector_pose_,
		param_do_end_effector_pose_ );

	if ( s_.wait_for_state() ) {
		// Configure publishers and subscribers
		pub_end_ = nh_.advertise<geometry_msgs::PoseStamped>( "end_effector", 10 );
		pub_viz_ = nh_.advertise<visualization_msgs::MarkerArray>( "visualization",
			10, true );

		ROS_INFO( "Configuring static mounts..." );
		// geometry_msgs::TransformStamped tf;
		// tf.header.stamp = ros::Time::now();
		// tf.transform.rotation.w = 1.0;

		// World Frame
		// tf.header.frame_id = "world";
		// tf.child_frame_id = "map";
		// tfsbr_.sendTransform(tf);

		do_reset();

		timer_ = nhp_.createTimer( ros::Duration( 1.0 / param_rate_ ),
			&ForwardKinematics::callback_timer, this );

		// Send static transforms for rviz model
		// TODO: Make more dynamic with params

		// Base Link
		// tf.header.frame_id = param_tf_prefix_;
		// tf.child_frame_id = param_tf_prefix_ + "/" + p_.body_inertial(0).name;
		// tfsbr_.sendTransform(tf);

		// Additional bodies
		/*
		for(int i=0; i<p_.get_joint_num(); i++) {
				DHParameters j(p_.joint(i));
				if(j.jt() != DHParameters::JointType::Static) {
						transform.header.frame_id = param_tf_prefix_ + "/" +
		p_.body_inertial(i).name;
						if( (i +i) < p_.get_joint_num() ) {
								transform.child_frame_id = param_tf_prefix_ + "/" +
		p_.body_inertial(i+1).name;
						} else {
								transform.child_frame_id = param_tf_prefix_ +
		"/end_effector";
						}

						tf.transform.translation =
		MDTools::vector_from_eig(-(j.transform().linear().transpose()*j.transform().translation()));
						tfsbr_.sendTransform(tf);
				}
		}
		*/

		// Propeller links
		tf_props.resize( p_.get(MantisParams::PARAM_MOTOR_NUM) );
		Eigen::Vector3d arm = Eigen::Vector3d( p_.get(MantisParams::PARAM_BASE_ARM_LENGTH), 0.0, 0.046 );

		// Prepare common values, we will preset prop angles soon to "nice" values
		//but we need to make sure it is a valid quaternion now
		for ( int i = 0; i < p_.get(MantisParams::PARAM_MOTOR_NUM); i++ ) {
			tf_props[i].header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_BODY_INERTIAL, 0 ).name;
			tf_props[i].child_frame_id = s_.model_id() + "/link_rotor_" + std::to_string( i + 1 );
			tf_props[i].transform.rotation.w = 1.0;
		}

		std::vector<double> rot_angles(p_.get(MantisParams::PARAM_MOTOR_NUM));
		std::vector<double> cant_angles(p_.get(MantisParams::PARAM_MOTOR_NUM));

		if ( p_.get(MantisParams::PARAM_AIRFRAME_TYPE) == MantisParams::PARAM_AIRFRAME_TYPE_QUAD_P4 ) {
			rot_angles[0] = 6.0 * M_PI / 4.0;
			rot_angles[1] = 2.0 * M_PI / 4.0;
			rot_angles[2] = 0.0 * M_PI / 4.0;
			rot_angles[3] = 4.0 * M_PI / 4.0;
		} else if ( p_.get(MantisParams::PARAM_AIRFRAME_TYPE) == MantisParams::PARAM_AIRFRAME_TYPE_QUAD_X4 ) {
			rot_angles[0] = 7.0 * M_PI / 4.0;
			rot_angles[1] = 3.0 * M_PI / 4.0;
			rot_angles[2] = 1.0 * M_PI / 4.0;
			rot_angles[3] = 5.0 * M_PI / 4.0;
		} else if ( p_.get(MantisParams::PARAM_AIRFRAME_TYPE) == MantisParams::PARAM_AIRFRAME_TYPE_HEX_P6 ) {
			rot_angles[0] =  0.0 * M_PI / 6.0;
			rot_angles[1] =  6.0 * M_PI / 6.0;
			rot_angles[2] =  4.0 * M_PI / 6.0;
			rot_angles[3] = 10.0 * M_PI / 6.0;
			rot_angles[4] =  2.0 * M_PI / 6.0;
			rot_angles[5] =  8.0 * M_PI / 6.0;
		} else if ( p_.get(MantisParams::PARAM_AIRFRAME_TYPE) == MantisParams::PARAM_AIRFRAME_TYPE_HEX_X6 ) {
			rot_angles[0] =  9.0 * M_PI / 6.0;
			rot_angles[1] =  3.0 * M_PI / 6.0;
			rot_angles[2] =  1.0 * M_PI / 6.0;
			rot_angles[3] =  7.0 * M_PI / 6.0;
			rot_angles[4] = 11.0 * M_PI / 6.0;
			rot_angles[5] =  5.0 * M_PI / 6.0;
		} else if ( p_.get(MantisParams::PARAM_AIRFRAME_TYPE) == MantisParams::PARAM_AIRFRAME_TYPE_HEX_FA ) {
			//XXX: Motor cant gets applied as per the "standard" motor layout defined in thesis
			rot_angles[0] =  9.0 * M_PI / 6.0;
			rot_angles[1] =  3.0 * M_PI / 6.0;
			rot_angles[2] =  1.0 * M_PI / 6.0;
			rot_angles[3] =  7.0 * M_PI / 6.0;
			rot_angles[4] = 11.0 * M_PI / 6.0;
			rot_angles[5] =  5.0 * M_PI / 6.0;

			const double c = p_.get(MantisParams::PARAM_BASE_MOTOR_CANT);
			cant_angles[0] =  c;
			cant_angles[1] = -c;
			cant_angles[2] =  c;
			cant_angles[3] = -c;
			cant_angles[4] = -c;
			cant_angles[5] =  c;

		} else {
			ROS_ERROR("Un-mapped airframe type, cannot visualisze");
			ros::shutdown();
		}

		if(ros::ok()) {
			for(int i = 0; i< p_.get(MantisParams::PARAM_MOTOR_NUM); i++) {
				Eigen::AngleAxisd rot( rot_angles[i], Eigen::Vector3d::UnitZ() );
				Eigen::AngleAxisd cant(cant_angles[i], Eigen::Vector3d::UnitX());

				tf_props[i].transform.translation = MDTools::vector_from_eig( rot * arm );
				tf_props[i].transform.rotation = MDTools::quaternion_from_eig(rot * cant);	//rotate to look "nice" plus prop cant
			}

			ROS_INFO( "Forward kinematics running!" );
		}
	} else {
		ros::shutdown();
	}
}

ForwardKinematics::~ForwardKinematics() {
}

void ForwardKinematics::callback_timer( const ros::TimerEvent& e ) {
	check_update_time();

	geometry_msgs::TransformStamped t;

	t.header.frame_id = s_.frame_id();
	t.header.stamp = e.current_real;
	t.child_frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_BODY_INERTIAL, 0).name;

	geometry_msgs::Pose p = MDTools::pose_from_eig( s_.g() );
	t.transform.translation.x = p.position.x;
	t.transform.translation.y = p.position.y;
	t.transform.translation.z = p.position.z;
	t.transform.rotation = p.orientation;

	tfbr_.sendTransform( t );
	tfBuffer_.setTransform( t, ros::this_node::getName() ); // Set the base link

	for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM); i++ ) {
		if ( p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).type != "static" ) {
			Eigen::Affine3d g;
			if ( solver_.calculate_gxy( g, i, i + 1 ) ) {
				geometry_msgs::TransformStamped tf = tf2::eigenToTransform( g );
				tf.header.stamp = e.current_real;

				if ( i < 1 ) {
					tf.header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_BODY_INERTIAL, i).name;
				} else {
					tf.header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i - 1).name;
				}

				tf.child_frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).name;

				tfbr_.sendTransform( tf );
				tfBuffer_.setTransform(
					tf, ros::this_node::getName() ); // Set the internal dynamic joints
			}
		}
	}

	if ( param_do_prop_viz_ ) {
		if ( s_.flight_ready() ) {
			Eigen::Quaterniond r( Eigen::AngleAxisd( prop_rate_, Eigen::Vector3d::UnitZ() ) );
			const Eigen::MatrixXd M = p_.get(MantisParams::PARAM_MIXER);

			Eigen::VectorXd mdir = Eigen::VectorXd::Zero(p_.get(MantisParams::PARAM_MOTOR_NUM));
			if (M.cols() == 4) {
				mdir = M * Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
			} else if  (M.cols() == 6) {
				Eigen::VectorXd cz(6);
				cz <<  0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
				mdir = M * cz;
			}

			for ( int i = 0; i < p_.get(MantisParams::PARAM_MOTOR_NUM); i++ ) {
				// tf_props[i].header.stamp = e.current_real;
				Eigen::Quaterniond q = MDTools::quaternion_from_msg( tf_props[i].transform.rotation );
				// Correct q for motor directions
				Eigen::Quaterniond mr = ( mdir[i] < 0 ) ? r : r.inverse();
				tf_props[i].transform.rotation = MDTools::quaternion_from_eig( q * mr );	//XXX: do q*mr to make sure motor cant is respected
			}

		}

		for ( int i = 0; i < p_.get(MantisParams::PARAM_MOTOR_NUM); i++ ) {
			tf_props[i].header.stamp = e.current_real;
			tfbr_.sendTransform( tf_props[i] );
		}
	}

	if ( param_do_end_effector_pose_ ) {
		geometry_msgs::PoseStamped msg_end_out_;
		Eigen::Affine3d gbe;
		solver_.calculate_gbe( gbe );

		msg_end_out_.header.stamp = e.current_real;
		msg_end_out_.header.frame_id = s_.frame_id();
		msg_end_out_.pose = MDTools::pose_from_eig( s_.g() * gbe );

		pub_end_.publish( msg_end_out_ );
	}
}

void ForwardKinematics::callback_props( const ros::TimerEvent& e ) {
}
/*
void ForwardKinematics::callback_state_odom(const nav_msgs::Odometry::ConstPtr&
msg_in) {

}

void ForwardKinematics::callback_state_joints(const
sensor_msgs::JointState::ConstPtr& msg_in) {
        check_update_time();

        //Send out the end effector pose
        if(param_do_end_effector_pose_) {
                geometry_msgs::PoseStamped msg_end_out_;
            geometry_msgs::TransformStamped tf_end;
                std::string last_joint = param_tf_prefix_ + "/link_" +
std::to_string(param_joints_.size());

                try {
                        tf_end = tfBuffer_.lookupTransform(param_frame_id_,
last_joint, ros::Time(0));

                        msg_end_out_.header = tf_end.header;
                        msg_end_out_.pose.position.x =
tf_end.transform.translation.x;
                        msg_end_out_.pose.position.y =
tf_end.transform.translation.y;
                        msg_end_out_.pose.position.z =
tf_end.transform.translation.z;
                        msg_end_out_.pose.orientation.w =
tf_end.transform.rotation.w;
                        msg_end_out_.pose.orientation.x =
tf_end.transform.rotation.x;
                        msg_end_out_.pose.orientation.y =
tf_end.transform.rotation.y;
                        msg_end_out_.pose.orientation.z =
tf_end.transform.rotation.z;

                        pub_end_.publish(msg_end_out_);
                }
                catch(const tf2::LookupException& e) {
                        ROS_WARN_THROTTLE(1.0, "Lookup error: %s", e.what());
                }
                catch(const tf2::ExtrapolationException& e) {
                        ROS_WARN_THROTTLE(1.0, "Extrapolation error: %s",
e.what());
                }
        }
        //if(param_do_viz_ && !param_done_viz_)
        //	do_viz(&(msg_in->name));
}
*/

void ForwardKinematics::configure_static_joints() {
	ros::Time stamp = ros::Time::now();

	for ( int i = 0; i < p_.get(MantisParams::PARAM_JOINT_NUM); i++ ) {
		Eigen::Affine3d g;

		if ( p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).type == "static" ) {
			if ( solver_.calculate_gxy( g, i, i + 1 ) ) {
				geometry_msgs::TransformStamped tf = tf2::eigenToTransform( g );
				tf.header.stamp = stamp;

				if ( i < 1 ) {
					tf.header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_BODY_INERTIAL, i).name;
				} else {
					tf.header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i-1).name;
				}

				tf.child_frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).name;

				tfsbr_.sendTransform( tf );
				tfBuffer_.setTransform( tf, ros::this_node::getName(),
					true ); // Set the internal static joints
			}
		}

		if ( i > 0 ) {
			geometry_msgs::TransformStamped tf_link;
			tf_link.header.stamp = stamp;
			tf_link.header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).name;
			tf_link.child_frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_BODY_INERTIAL, i).name;
			tf_link.transform.translation.x = -p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).r;
			tf_link.transform.rotation.w = 1.0;
			tfsbr_.sendTransform( tf_link );
			tfBuffer_.setTransform( tf_link, ros::this_node::getName(), true );
		}

		if ( i == ( p_.get(MantisParams::PARAM_JOINT_NUM) - 1 ) ) {
			geometry_msgs::TransformStamped tf_end;
			tf_end.header.stamp = stamp;
			tf_end.header.frame_id = s_.model_id() + "/" + p_.get(MantisParams::PARAM_JOINT_DESCRIPTION, i).name;
			tf_end.child_frame_id = s_.model_id() + "/end_effector";
			tf_end.transform.rotation.w = 1.0;
			tfsbr_.sendTransform( tf_end );
			tfBuffer_.setTransform( tf_end, ros::this_node::getName(), true );
		}
	}
}

void ForwardKinematics::do_reset() {
	tfBuffer_.clear();

	configure_static_joints();

	time_last_update_ = ros::Time::now();
}

void ForwardKinematics::check_update_time() {
	if ( ros::Time::now() < time_last_update_ ) {
		ROS_INFO( "Detected jump back in time, reseting transforms" );
		do_reset();
	} else {
		time_last_update_ = ros::Time::now();
	}
}
