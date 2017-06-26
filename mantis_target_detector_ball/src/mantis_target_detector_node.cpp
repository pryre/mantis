#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <iostream>

class BallDetector {
	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		ros::Subscriber sub_camera_info_;
		image_transport::Subscriber sub_image_;
		image_transport::Publisher pub_debug_image_;
		image_transport::Publisher pub_overlay_image_;
		ros::Publisher pub_detect_;
		ros::Publisher pub_marker_;

		bool got_camera_info_;
		sensor_msgs::CameraInfo camera_info_;

		std::string ball_name_;
		double ball_size_;
		double ball_hsv_min_hue_;
		double ball_hsv_min_sat_;
		double ball_hsv_min_val_;
		double ball_hsv_max_hue_;
		double ball_hsv_max_sat_;
		double ball_hsv_max_val_;

		std::string topic_input_camera_info_;
		std::string topic_input_image_;
		std::string topic_output_debug_image_;
		std::string topic_output_overlay_image_;
		std::string topic_output_detect_;
		std::string topic_output_marker_;

	public:
		BallDetector() :
			nh_( ros::this_node::getName() ),
			it_(nh_),
			ball_name_( "ball" ),
			ball_size_( 1.0 ),
			got_camera_info_( false ),
			topic_input_camera_info_( "input_camera_info" ),
			topic_input_image_( "input_image" ),
			topic_output_debug_image_( "debug_image" ),
			topic_output_overlay_image_( "overlay_image" ),
			topic_output_detect_( "target_pose" ),
			topic_output_marker_( "target_marker" ) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "topic_input_camera_info", topic_input_camera_info_, topic_input_camera_info_ );
			nh_.param( "topic_input_image", topic_input_image_, topic_input_image_ );
			nh_.param( "topic_output_debug_image", topic_output_debug_image_, topic_output_debug_image_ );
			nh_.param( "topic_output_overlay_image", topic_output_overlay_image_, topic_output_overlay_image_ );
			nh_.param( "topic_output_detect", topic_output_detect_, topic_output_detect_ );	//We use this topic to send our detection results
			nh_.param( "topic_output_marker", topic_output_marker_, topic_output_marker_ );

			// Subscrive to input video feed and publish output video feed
			sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo> ( topic_input_camera_info_, 100, &BallDetector::camera_info_cb, this );
			pub_debug_image_ = it_.advertise(topic_output_debug_image_, 100);
			pub_overlay_image_ = it_.advertise(topic_output_overlay_image_, 100);
			pub_detect_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_detect_, 100 );

			nh_.param( "ball_description/name", ball_name_, ball_name_ );
			nh_.param( "ball_description/size", ball_size_, ball_size_ );
			nh_.param( "ball_description/hsv_min/hue", ball_hsv_min_hue_, ball_hsv_min_hue_ );
			nh_.param( "ball_description/hsv_min/sat", ball_hsv_min_sat_, ball_hsv_min_sat_ );
			nh_.param( "ball_description/hsv_min/val", ball_hsv_min_val_, ball_hsv_min_val_ );
			nh_.param( "ball_description/hsv_max/hue", ball_hsv_max_hue_, ball_hsv_max_hue_ );
			nh_.param( "ball_description/hsv_max/sat", ball_hsv_max_sat_, ball_hsv_max_sat_ );
			nh_.param( "ball_description/hsv_max/val", ball_hsv_max_val_, ball_hsv_max_val_ );

			bool do_marker_pub = false;
			nh_.param( "show_marker", do_marker_pub, do_marker_pub );

			ROS_INFO("Waiting for camera info...");

			while( !got_camera_info_ && ros::ok() ) {
				ros::spinOnce();
				ros::Rate(20).sleep();
			}

			ROS_INFO("Recieved camera info!");

			if(do_marker_pub) {
				pub_marker_ = nh_.advertise<visualization_msgs::Marker>( topic_output_marker_, 1, true );

				double mid_hue = ( ball_hsv_max_hue_ + (ball_hsv_min_hue_ > ball_hsv_max_hue_ ? ball_hsv_min_hue_ + 360.0 : ball_hsv_min_hue_ ) ) / 2.0;
				double mid_sat = ( ball_hsv_max_sat_ + ball_hsv_min_sat_ ) / 2.0;
				double mid_val = ( ball_hsv_max_val_ + ball_hsv_min_val_ ) / 2.0;

				visualization_msgs::Marker marker;
				marker.header.frame_id = camera_info_.header.frame_id;
				marker.header.stamp = ros::Time::now();

				marker.ns = ball_name_;
				marker.id = 0;
				marker.type = visualization_msgs::Marker::SPHERE;
				marker.action = visualization_msgs::Marker::ADD;
				marker.lifetime = ros::Duration();

				marker.pose.position.x = 0;
				marker.pose.position.y = 0;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;

				marker.scale.x = ball_size_;
				marker.scale.y = ball_size_;
				marker.scale.z = ball_size_;

				//Convert the average HSV detection to a RGBA value for ROS
				//CV::HSV:
				//	H: 0->180
				//	S: 0->255
				//	V: 0->255
				//CV::RGB:
				//	R: 0->255
				//	G: 0->255
				//	B: 0->255
				//ROS::RGBA:
				//	R: 0->1.0
				//	G: 0->1.0
				//	B: 0->1.0
				//	A: 0->1.0
				cv::Scalar mid_hsv( (uint8_t)( ( mid_hue / 2 ) ),
									(uint8_t)( mid_sat * 255.0 ),
									(uint8_t)( mid_val * 255.0 ) );
				cv::Mat tmp_mat( 1,1, CV_8UC3, mid_hsv );
				cv::cvtColor( tmp_mat, tmp_mat, CV_HSV2BGR );
				cv::Scalar mid_rgb = tmp_mat.at<cv::Vec3b>(0,0);

				marker.color.r = ( (double)mid_rgb.val[2] ) / 255;
				marker.color.g = ( (double)mid_rgb.val[1] ) / 255;
				marker.color.b = ( (double)mid_rgb.val[0] ) / 255;
				marker.color.a = 1.0;

				pub_marker_.publish(marker);
			}

			sub_image_ = it_.subscribe(topic_input_image_, 100, &BallDetector::image_cb, this);

			ROS_INFO("Beginning detection...");
		}

		~BallDetector() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		//We can use the camera info to correct for distortions, get image size, and other useful things
		void camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg_in) {
			camera_info_ = *msg_in;

			got_camera_info_ = true;	//Allow processing to begin
		}

		void image_cb( const sensor_msgs::Image::ConstPtr& msg_in ) {
			cv_bridge::CvImagePtr cv_ptr;

			try	{
				cv_ptr = cv_bridge::toCvCopy( msg_in, sensor_msgs::image_encodings::BGR8 );
			} catch ( cv_bridge::Exception& e ) {
				ROS_ERROR( "cv_bridge exception: %s", e.what() );
				return;
			}

			//Copy image to HSV format, and to keep clean image for later
			cv::Mat hsv_image;
			cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

			//Create a spectrum mask
			if( ball_hsv_min_hue_ < ball_hsv_max_hue_ ) {
				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( ball_hsv_min_hue_ / 2.0 ),
										 (uint8_t)( ball_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( ball_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( ball_hsv_max_hue_ / 2.0 ),
										 (uint8_t)( ball_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( ball_hsv_max_val_ * 255.0 ) ),
							 hsv_image);
			} else {
				cv::Mat low_range;
				cv::Mat high_range;

				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( ball_hsv_min_hue_ / 2.0 ),
										 (uint8_t)( ball_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( ball_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( 0.0 ),
										 (uint8_t)( ball_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( ball_hsv_max_val_ * 255.0 ) ),
							 low_range);

				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( 0.0 ),
										 (uint8_t)( ball_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( ball_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( ball_hsv_max_hue_ / 2.0 ),
										 (uint8_t)( ball_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( ball_hsv_max_val_ * 255.0 ) ),
							 high_range);

				cv::bitwise_or( low_range, high_range, hsv_image );
			}

			//Perform an Openning and Closing morphology transform to fill in and smooth out the ball
			cv::Mat structuring_element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 9, 9 ) );
			cv::morphologyEx( hsv_image, hsv_image, cv::MORPH_OPEN, structuring_element );	//Apply the specified morphology operation
			cv::morphologyEx( hsv_image, hsv_image, cv::MORPH_CLOSE, structuring_element );	//Apply the specified morphology operation

			//Blur slightly to increase chances of detection
			cv::GaussianBlur(hsv_image, hsv_image, cv::Size(9, 9), 2, 2);

			// Use the Hough transform to detect circles in the combined threshold image
			std::vector<cv::Vec3f> found_circles;
			double min_dist = hsv_image.rows / 8;	//TODO: Params
			double param1 = 100;	//TODO: Params
			double param2 = 20;	//TODO: Params
			double min_radius = 50;	//TODO: Params
			double max_radius = 200;	//TODO: Params
			cv::HoughCircles(hsv_image, found_circles, CV_HOUGH_GRADIENT, 1, min_dist, param1, param2, min_radius, max_radius);

			if(found_circles.size() > 0) {
				//Transmit the detection message
				geometry_msgs::PoseStamped msg_out;

				msg_out.header.frame_id = camera_info_.header.frame_id;
				msg_out.header.stamp = msg_in->header.stamp;
				/*
				tmp_dim.label = "num_detected";
				tmp_dim.size = circles.size();
				tmp_dim.stride = 3*circles.size();	//dim[0].size is just the size of the entire array
				msg_out.layout.dim.push_back( tmp_dim );

				tmp_dim.label = "circle_x_y_size";
				tmp_dim.size = 3;
				tmp_dim.stride = 3;
				msg_out.layout.dim.push_back( tmp_dim );

				msg_out.layout.data_offset = 0;

				for( size_t current_circle = 0; current_circle < circles.size(); ++current_circle ) {
					msg_out.data.push_back( circles[current_circle][0] );
					msg_out.data.push_back( circles[current_circle][1] );
					msg_out.data.push_back( circles[current_circle][2] );
				}
				*/

				pub_detect_.publish( msg_out );
			}

			//Only compute and send the debug image if a node is subscribed
			if( pub_debug_image_.getNumSubscribers() > 0 ) {
				//==-- Output modified video stream
				pub_debug_image_.publish( cv_bridge::CvImage( msg_in->header, "mono8", hsv_image ).toImageMsg() );
			}

			//Only compute and send the overlay image if a node is subscribed
			if( pub_overlay_image_.getNumSubscribers() > 0 ) {
				if( found_circles.size() > 0 ) {
					for( size_t current_circle = 0; current_circle < found_circles.size(); ++current_circle ) {
						cv::Point center( std::round( found_circles[current_circle][0] ), std::round( found_circles[current_circle][1] ) );
						int radius = std::round( found_circles[current_circle][2] );
						cv::circle( cv_ptr->image, center, radius, cv::Scalar( 0, 255, 0 ), 5 );
					}
				}

				//==-- Output modified video stream
				pub_overlay_image_.publish( cv_ptr->toImageMsg() );
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "goal_detector");
	BallDetector bd;

	ros::spin();

	return 0;
}
