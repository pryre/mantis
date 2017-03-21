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
#include <cmath>
#include <iostream>
#include <fstream>
#include <iterator>
#include <sstream>
#include <limits>

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

//http://alienryderflex.com/polygon/
bool pointInPolygon( double x, double y, const std::vector< double > &px, const std::vector< double > &py ) {
	bool oddNodes = false;
	int j = px.size() - 1;

	for (int i = 0; i < px.size(); i++) {
		if ( ( py.at(i) < y ) && ( py.at(j) >= y ) || ( py.at(j) < y ) && ( py.at(i) >= y ) ) {
			if ( ( px.at(i) + ( y - py.at(i) ) / ( py.at(j) - py.at(i) ) * ( px.at(j) - px.at(i) ) ) < x ) {
				oddNodes = !oddNodes;
			}
		}
		
		j = i; 
	}

	return oddNodes;
}

//https://gist.github.com/atduskgreg/1325002
double distanceFromLine(const double x, const double y, const double lx1, const double ly1, const double lx2, const double ly2, double &cx, double &cy) {
    double xDelta = lx2 - lx1;
    double yDelta = ly2 - ly1;

    double u = ((x - lx1) * xDelta + (y - ly1)*yDelta) / (xDelta * xDelta + yDelta * yDelta);

    if (u < 0) {
		cx = lx1;
		cy = ly1;
	} else if (u > 1) {
		cx = lx2;
		cy = ly2;
	} else {
        cx = lx1 + (u * xDelta);
		cy = ly1 + (u * yDelta);
	}
    
    double dx = x - cx;
    double dy = y - cy;
	
    return std::sqrt(dx * dx + dy * dy);	// distance
}

//https://gist.github.com/atduskgreg/1325002
double distanceFromPoly(const double x, const double y, const std::vector< double > &px, const std::vector< double > &py, double &cx, double &cy) {
    double result = std::numeric_limits<double>::max();
	
    // check each line
	int j = px.size() - 1;
	
    for(int i = 0; i < px.size(); i++) {
		double cx_temp = 0;
		double cy_temp = 0;
		double current_x = px.at(i);	//Current Point
		double current_y = py.at(i);
		double last_x = px.at(j);	//Last Point
		double last_y = py.at(j);
        
        double segmentDistance = distanceFromLine(x, y, last_x, last_y, current_x, current_y, cx_temp, cy_temp);
        
        if(segmentDistance < result){
			cx = cx_temp;
			cy = cy_temp;
            result = segmentDistance;
        }
		
		j = i;
    }
    
    return result;
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
		std::string camera_frame;
		double arm_length_mount;
		double arm_length_link1;
		double arm_length_link2;
		
		std::vector< double > px_;
		std::vector< double > pz_;

	public:
		MantisKinematics() : nh_(ros::this_node::getName()), tfln_(tfBuffer) {
			std::string topic_marker_detected = "marker_detection";
			std::string topic_state_shoulder = "state_shoulder";
			std::string topic_state_elbow = "state_elbow";
			std::string topic_state_wrist = "state_wrist";
			std::string topic_command_shoulder = "command_shoulder";
			std::string topic_command_elbow = "command_elbow";
			std::string topic_command_wrist = "command_wrist";
			
			std::string input_boundary;
			
			mav_frame = "fcu";
			camera_frame = "camera";

			loadParam(nh_, "topic_marker_detected", topic_marker_detected);
			loadParam(nh_, "topic_state_shoulder", topic_state_shoulder);
			loadParam(nh_, "topic_state_elbow", topic_state_elbow);
			loadParam(nh_, "topic_state_wrist", topic_state_wrist);
			
			loadParam(nh_, "topic_command_shoulder", topic_command_shoulder);
			loadParam(nh_, "topic_command_elbow", topic_command_elbow);
			loadParam(nh_, "topic_command_wrist", topic_command_wrist);

			loadParam(nh_, "input_boundary", input_boundary);

			loadParam(nh_, "mav_frame", mav_frame);
			loadParam(nh_, "camera_frame", camera_frame);
			loadParam(nh_, "arm_length_mount", arm_length_mount);
			loadParam(nh_, "arm_length_link1", arm_length_link1);
			loadParam(nh_, "arm_length_link2", arm_length_link2);

			//Publish the camera transformation
			geometry_msgs::TransformStamped tf_static_camera;
			tf_static_camera.header.stamp = ros::Time::now();
			tf_static_camera.header.frame_id = mav_frame;
			tf_static_camera.child_frame_id = camera_frame;
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
			
			//TODO: Read in the boundary polygon (could be generated on the fly...
			std::vector< std::vector < double > > temp_poly;
			readBoundaryPoly( input_boundary, temp_poly);
			px_ = temp_poly.at(0);
			pz_ = temp_poly.at(1);
			
			
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
		
		void readBoundaryPoly( std::string filename, std::vector< std::vector < double > > &p ) {
			std::ifstream in( filename );
			std::string record;
			
			p.clear();
			p.push_back(std::vector < double >());
			p.push_back(std::vector < double >());
			
			double x;
			double z;
			int i = 1;
			
			while ( in >> x >> z ) {
				p.at(0).push_back( x );
				p.at(1).push_back( z );
				
				ROS_INFO("Point %i: [%0.4f, %0.4f]", i, x, z);
				i++;
			}
			
			ROS_ASSERT_MSG( p.at(0).size() == p.at(1).size(), "Polygon definition was not correct (px != py)" );
			
			ROS_INFO("Loaded boundary with %li points", p.at(0).size() );
		}
		
		//TODO: Each joint should be dynamically created according to the model
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
		
		void setXZGoal( const double x, const double z) {
			double gx = 0;
			double gz = 0;
			double dist = 0;
		
			if( pointInPolygon(x, z, px_, pz_) ) {
				gx = x;
				gz = z;
			} else {
				dist = distanceFromPoly(x, z, px_, pz_, gx, gz);
			}
			
			//TODO: ik
			//TODO: Broadcast shoulder and wrist
		}

		//TODO: This should not be here at all
		//Instead it should be pose goal input in mav_frame, and output the rotation commands
		void marker_cb(const ml_msgs::MarkerDetection::ConstPtr& msg) {
			for(int i = 0; i < msg->markers.size(); i++) {
				if(msg->markers.at(i).marker_id == 0) {	//XXX: We only check for the marker 0
					geometry_msgs::TransformStamped tf;
			
					tf.header = msg->header;
					tf.child_frame_id = "marker";
					tf.transform.translation.x = msg->markers.at(i).pose.position.x;
					tf.transform.translation.y = msg->markers.at(i).pose.position.y;
					tf.transform.translation.z = msg->markers.at(i).pose.position.z;
					tf.transform.rotation.w = msg->markers.at(i).pose.orientation.w;
					tf.transform.rotation.x = msg->markers.at(i).pose.orientation.x;
					tf.transform.rotation.y = msg->markers.at(i).pose.orientation.y;
					tf.transform.rotation.z = msg->markers.at(i).pose.orientation.z;
			
					tfbr_.sendTransform(tf);
					
					setXZGoal(tf.transform.translation.x, tf.transform.translation.z);
					//TODO: Point wrist at marker
				}
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mantis_kinematics");
	MantisKinematics mk;

	ros::spin();
	
	return 0;
}
