#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <ml_msgs/MarkerDetection.h>
#include <dynamixel_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <vector>
#include <string>


void loadParam(ros::NodeHandle &n, const std::string &str, bool &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %s", str.c_str(), param ? "true" : "false" );
	} else {
		ROS_INFO( "Loaded %s: %s", str.c_str(), param ? "true" : "false" );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, int &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %i", str.c_str(), param );
	} else {
		ROS_INFO( "Loaded %s: %i", str.c_str(), param );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, double &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %f", str.c_str(), param );
	} else {
		ROS_INFO( "Loaded %s: %f", str.c_str(), param );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, std::string &param) {
	if( !n.getParam( str, param ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: %s", str.c_str(), param.c_str() );
	} else {
		ROS_INFO( "Loaded %s: %s", str.c_str(), param.c_str() );
	}
}

void loadParam(ros::NodeHandle &n, const std::string &str, tf2::Vector3 &param) {
	std::vector< double > temp_vec;

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2) );
	}

	param = tf2::Vector3( temp_vec.at(0), temp_vec.at(1), temp_vec.at(2) );
}

void loadParam(ros::NodeHandle &n, const std::string &str, tf2::Quaternion &param) {
	std::vector< double > temp_vec;

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3) );
	}

	param = tf2::Quaternion( temp_vec.at(1), temp_vec.at(2), temp_vec.at(3), temp_vec.at(0) );
}

void loadParam(ros::NodeHandle &n, const std::string &str, geometry_msgs::Transform &param) {
	std::vector< double > temp_vec = {0,0,0,1,0,0,0};

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f], [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3), temp_vec.at(4), temp_vec.at(5), temp_vec.at(6) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f], [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3), temp_vec.at(4), temp_vec.at(5), temp_vec.at(6) );
	}

	param.translation.x = temp_vec.at(0);
	param.translation.y = temp_vec.at(1);
	param.translation.z = temp_vec.at(2);
	param.rotation.w = temp_vec.at(3);
	param.rotation.x = temp_vec.at(4);
	param.rotation.y = temp_vec.at(5);
	param.rotation.z = temp_vec.at(6);
}

void loadParam(ros::NodeHandle &n, const std::string &str, geometry_msgs::Quaternion &param) {
	std::vector< double > temp_vec = {1,0,0,0};

	if( !n.getParam( str, temp_vec ) ) {
		ROS_WARN( "No parameter set for \"%s\", using: [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3) );
	} else {
		ROS_INFO( "Loaded %s: [%f, %f, %f, %f]", str.c_str(), temp_vec.at(0), temp_vec.at(1), temp_vec.at(2), temp_vec.at(3) );
	}
	
	param.w = temp_vec.at(0);
	param.x = temp_vec.at(1);
	param.y = temp_vec.at(2);
	param.z = temp_vec.at(3);
}

void poseToTransform( const geometry_msgs::Pose &p, geometry_msgs::Transform &t ) {
	t.translation.x = p.position.x;
	t.translation.y = p.position.y;
	t.translation.z = p.position.z;

	t.rotation.w = p.orientation.w;
	t.rotation.x = p.orientation.x;
	t.rotation.y = p.orientation.y;
	t.rotation.z = p.orientation.z;
}

void vector3MsgToTF2( const geometry_msgs::Vector3 &vg, tf2::Vector3 &vt ) {
	vt = tf2::Vector3( vg.x, vg.y, vg.z );
}

void vector3TF2ToMsg( const tf2::Vector3 &vt, geometry_msgs::Vector3 &vg ) {
	vg.x = vt.x();
	vg.y = vt.y();
	vg.z = vt.z();
}

void quaternionMsgToTF2( const geometry_msgs::Quaternion &qg, tf2::Quaternion &qt ) {
	qt = tf2::Quaternion (qg.x, qg.y, qg.z, qg.w);
}

void quaternionTF2ToMsg( const tf2::Quaternion &qt, geometry_msgs::Quaternion &qg  ) {
	qg.x = qt.x();
	qg.y = qt.y();
	qg.z = qt.z();
	qg.w = qt.w();
}

void transformMsgToTF2( const geometry_msgs::Transform &g, tf2::Transform &t ) {
	tf2::Vector3 v;
	tf2::Quaternion q;

	vector3MsgToTF2( g.translation, v);
	t.setOrigin(v);

	quaternionMsgToTF2( g.rotation, q );
	t.setRotation(q);
}

void transformTF2ToMsg( const tf2::Transform &t, geometry_msgs::Transform &g ) {
	vector3TF2ToMsg( t.getOrigin(), g.translation );
	quaternionTF2ToMsg( t.getRotation(), g.rotation );
}

class MantisKinematics {
	private:
		ros::NodeHandle nh_;
		
		ros::Subscriber marker_sub_;
		ros::Subscriber state_shoulder_sub_;
		ros::Subscriber state_elbow_sub_;
		ros::Subscriber state_wrist_sub_;
		
		ros::Publisher command_shoulder_pub_;
		ros::Publisher command_elbow_pub_;
		ros::Publisher command_wrist_pub_;

		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfln_;
		tf2_ros::TransformBroadcaster tfbr_;
		tf2_ros::StaticTransformBroadcaster tfbrs_;
		
		std::string mav_frame;
		double arm_length_mount;
		double arm_length_link1;
		double arm_length_link2;

	public:
		MantisKinematics() : nh_(ros::this_node::getName()), tfln_(tfBuffer) {
			std::string topic_marker_detected = "marker_detection";
			std::string topic_state_shoulder = "state_shoulder";
			std::string topic_state_elbow = "state_elbow";
			std::string topic_state_wrist = "state_wrist";
			std::string topic_command_shoulder = "command_shoulder";
			std::string topic_command_elbow = "command_elbow";
			std::string topic_command_wrist = "command_wrist";
			mav_frame = "fcu";

			loadParam(nh_, "topic_marker_detected", topic_marker_detected);
			loadParam(nh_, "topic_state_shoulder", topic_state_shoulder);
			loadParam(nh_, "topic_state_elbow", topic_state_elbow);
			loadParam(nh_, "topic_state_wrist", topic_state_wrist);
			
			loadParam(nh_, "topic_command_shoulder", topic_command_shoulder);
			loadParam(nh_, "topic_command_elbow", topic_command_elbow);
			loadParam(nh_, "topic_command_wrist", topic_command_wrist);

			loadParam(nh_, "mav_frame", mav_frame);
			loadParam(nh_, "arm_length_mount", arm_length_mount);
			loadParam(nh_, "arm_length_link1", arm_length_link1);
			loadParam(nh_, "arm_length_link2", arm_length_link2);

			//Publish the camera transformation
			geometry_msgs::TransformStamped tf_static_camera;
			tf_static_camera.header.stamp = ros::Time::now();
			tf_static_camera.header.frame_id = mav_frame;
			tf_static_camera.child_frame_id = "camera";
			loadParam(nh_, "static_camera_pose", tf_static_camera.transform);
			tf2::Quaternion camera_q;
			quaternionMsgToTF2(tf_static_camera.transform.rotation, camera_q);
			camera_q.normalize();
			quaternionTF2ToMsg(camera_q, tf_static_camera.transform.rotation);
			tfbrs_.sendTransform(tf_static_camera);	//Send off a once-off transform to define camera
			
			geometry_msgs::TransformStamped tf_static_arm;
			tf_static_arm.header.stamp = ros::Time::now();
			tf_static_arm.header.frame_id = mav_frame;
			tf_static_arm.child_frame_id = "arm_mount";
			tf2::Quaternion arm_q;
			loadParam(nh_, "static_arm_rotation", arm_q);
			arm_q.normalize();
			quaternionTF2ToMsg(arm_q, tf_static_arm.transform.rotation);
			tfbrs_.sendTransform(tf_static_arm);	//Send off a once-off transform to define arm mount
			
			marker_sub_ = nh_.subscribe<ml_msgs::MarkerDetection> ( topic_marker_detected, 100, &MantisKinematics::marker_cb, this );
			state_shoulder_sub_ = nh_.subscribe<dynamixel_msgs::JointState> ( topic_state_shoulder, 100, &MantisKinematics::shoulder_cb, this );
			state_elbow_sub_ = nh_.subscribe<dynamixel_msgs::JointState> ( topic_state_elbow, 100, &MantisKinematics::elbow_cb, this );
			state_wrist_sub_ = nh_.subscribe<dynamixel_msgs::JointState> ( topic_state_wrist, 100, &MantisKinematics::wrist_cb, this );
			
			command_shoulder_pub_ = nh_.advertise<std_msgs::Float64> ( topic_command_shoulder, 100 );
			command_elbow_pub_ = nh_.advertise<std_msgs::Float64> ( topic_command_elbow, 100 );
			command_wrist_pub_ = nh_.advertise<std_msgs::Float64> ( topic_command_wrist, 100 );

			ROS_INFO("Mantis kinematics module is ready to go...");
		}

		~MantisKinematics() {
		}

		void shoulder_cb(const dynamixel_msgs::JointState::ConstPtr& state ) {
			geometry_msgs::TransformStamped tf;
			tf2::Quaternion q;
			
			
			tf.header.stamp = ros::Time::now();
			tf.header.frame_id = "arm_mount";
			tf.child_frame_id = "arm_link1";
			
			tf.transform.translation.x = arm_length_mount;
			q.setRPY(0.0f, state->current_pos, 0.0f);
			quaternionTF2ToMsg(q, tf.transform.rotation);
			
			tfbr_.sendTransform(tf);
		}

		void elbow_cb(const dynamixel_msgs::JointState::ConstPtr& state ) {
			geometry_msgs::TransformStamped tf;
			tf2::Quaternion q;
			
			
			tf.header.stamp = ros::Time::now();
			tf.header.frame_id = "arm_link1";
			tf.child_frame_id = "arm_link2";
			
			tf.transform.translation.x = arm_length_link1;
			q.setRPY(0.0f, state->current_pos, 0.0f);
			quaternionTF2ToMsg(q, tf.transform.rotation);
			
			tfbr_.sendTransform(tf);
		}

		void wrist_cb(const dynamixel_msgs::JointState::ConstPtr& state ) {
			geometry_msgs::TransformStamped tf;
			tf2::Quaternion q;
			
			
			tf.header.stamp = ros::Time::now();
			tf.header.frame_id = "arm_link2";
			tf.child_frame_id = "arm_link3";
			
			tf.transform.translation.x = arm_length_link2;
			q.setRPY(0.0f, state->current_pos, 0.0f);
			quaternionTF2ToMsg(q, tf.transform.rotation);
			
			tfbr_.sendTransform(tf);
		}

		void marker_cb(const ml_msgs::MarkerDetection::ConstPtr& msg) {
			ROS_INFO("NEW MARKER_MESSAGE!");
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_kinematics");
	MantisKinematics mk;

	ros::spin();
	
	return 0;
}
