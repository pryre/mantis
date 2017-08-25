#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

#include <string>

class ModelStatesOdom {
	private:
		ros::NodeHandle nh_;

		ros::Subscriber sub_model_states_;
		ros::Publisher pub_odom_;

		std::string topic_model_states_;
		std::string topic_odom_;

		std::string parent_name_;
		std::string model_name_;
		nav_msgs::Odometry msg_odom_;

	public:
		ModelStatesOdom() :
			nh_( "~" ),
			model_name_("mantis_uav"),
			parent_name_("world"),
			topic_model_states_("/gazebo/model_states"),
			topic_odom_("/mantis_uav/odom") {

			sub_model_states_ = nh_.subscribe<gazebo_msgs::ModelStates>(topic_model_states_, 1, &ModelStatesOdom::statesCallback, this);
			pub_odom_ = nh_.advertise<nav_msgs::Odometry>( topic_odom_, 10 );

			msg_odom_.header.frame_id = parent_name_;
			msg_odom_.child_frame_id = model_name_;

			ROS_INFO("Listenning for model states...");
		}

		~ModelStatesOdom() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		void statesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg_in) {
			msg_odom_.header.stamp = ros::Time::now();

			for( int i=0; i<msg_in->name.size(); i++ ) {
				if( msg_in->name[i] == model_name_ ) {
					//Velocities must be in the base_link frame
					//The base_link frame is locked in roll and pitch with the world frame, but can rotate with yaw
					tf2::Quaternion rot_l_vel;
					tf2::Quaternion rot_a_vel;

					//Get the rotation from the world to body frame (for angular measurements
					tf2::Quaternion tmp_q( msg_in->pose[i].orientation.x,
										   msg_in->pose[i].orientation.y,
										   msg_in->pose[i].orientation.z,
										   msg_in->pose[i].orientation.w );

					rot_a_vel = ( tmp_q.inverse() * tf2::Quaternion( msg_in->twist[i].angular.x,
														   msg_in->twist[i].angular.y,
														   msg_in->twist[i].angular.z,
														   0.0 ) ) * tmp_q;

					//Derotate the world->body quaternion
					tf2::Matrix3x3 r( tmp_q );
					tf2::Vector3 body_x;
					tf2::Vector3 body_y( r.getRow(1) );	//Use the y vector for yaw reference
					tf2::Vector3 body_z(0.0, 0.0, 1.0);

					body_x = body_y.cross(body_z);
					body_x.normalize();
					body_y = body_z.cross(body_x);
					body_y.normalize();
					r.setValue( body_x.x(), body_x.y(), body_x.z(), body_y.x(), body_y.y(), body_y.z(), body_z.x(), body_z.y(), body_z.z() );

					r.getRotation( tmp_q );	//Copy the rotation back into the quaternion

					rot_l_vel = ( tmp_q.inverse() * tf2::Quaternion( msg_in->twist[i].linear.x,
														   msg_in->twist[i].linear.y,
														   msg_in->twist[i].linear.z,
														   0.0 ) ) * tmp_q;

					msg_odom_.pose.pose = msg_in->pose[i];

					msg_odom_.twist.twist.linear.x = rot_l_vel.getX();
					msg_odom_.twist.twist.linear.y = rot_l_vel.getY();
					msg_odom_.twist.twist.linear.z = rot_l_vel.getZ();
					msg_odom_.twist.twist.angular.x = rot_a_vel.getX();
					msg_odom_.twist.twist.angular.y = rot_a_vel.getY();
					msg_odom_.twist.twist.angular.z = rot_a_vel.getZ();

					pub_odom_.publish(msg_odom_);

					break;
				}
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "model_states_odom");
	ModelStatesOdom mso;

	ros::spin();

	return 0;
}
