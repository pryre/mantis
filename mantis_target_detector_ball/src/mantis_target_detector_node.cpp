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

		std::string param_ball_name_;
		double param_ball_size_;
		double param_hsv_min_hue_;
		double param_hsv_min_sat_;
		double param_hsv_min_val_;
		double param_hsv_max_hue_;
		double param_hsv_max_sat_;
		double param_hsv_max_val_;

		double param_hough_accum_;
		double param_hough_min_dist_;
		double param_hough_param1_;
		double param_hough_param2_;
		double param_hough_min_rad_;
		double param_hough_max_rad_;

		double param_morph_open_size_;
		double param_morph_close_size_;
		double param_blur_size_;
		double param_blur_dist_;
		double param_low_pass_pos_a_;
		double param_low_pass_size_a_;

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
			param_ball_name_( "ball" ),
			param_ball_size_( 1.0 ),
			param_hough_accum_( 2 ),
			param_hough_min_dist_( 4 ),
			param_hough_param1_( 100 ),
			param_hough_param2_( 40 ),
			param_hough_min_rad_( 50 ),
			param_hough_max_rad_( 100 ),
			param_morph_open_size_( 11 ),
			param_morph_close_size_( 11 ),
			param_blur_size_( 9 ),
			param_blur_dist_( 2 ),
			param_low_pass_pos_a_( 0.2 ),
			param_low_pass_size_a_( 0.2 ),
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

			nh_.param( "ball_description/name", param_ball_name_, param_ball_name_ );
			nh_.param( "ball_description/size", param_ball_size_, param_ball_size_ );
			nh_.param( "ball_description/hsv_min/hue", param_hsv_min_hue_, param_hsv_min_hue_ );
			nh_.param( "ball_description/hsv_min/sat", param_hsv_min_sat_, param_hsv_min_sat_ );
			nh_.param( "ball_description/hsv_min/val", param_hsv_min_val_, param_hsv_min_val_ );
			nh_.param( "ball_description/hsv_max/hue", param_hsv_max_hue_, param_hsv_max_hue_ );
			nh_.param( "ball_description/hsv_max/sat", param_hsv_max_sat_, param_hsv_max_sat_ );
			nh_.param( "ball_description/hsv_max/val", param_hsv_max_val_, param_hsv_max_val_ );

			nh_.param( "hough_detector/accum_scale", param_hough_accum_, param_hough_accum_);
			nh_.param( "hough_detector/min_dist_scale", param_hough_min_dist_, param_hough_min_dist_ );
			nh_.param( "hough_detector/param1", param_hough_param1_, param_hough_param1_ );
			nh_.param( "hough_detector/param2", param_hough_param2_, param_hough_param2_ );
			nh_.param( "hough_detector/min_radius", param_hough_min_rad_, param_hough_min_rad_ );
			nh_.param( "hough_detector/max_radius", param_hough_max_rad_, param_hough_max_rad_ );

			nh_.param( "filters/openning/size", param_morph_open_size_, param_morph_open_size_ );
			nh_.param( "filters/closing/size", param_morph_close_size_, param_morph_close_size_ );
			nh_.param( "filters/blur/size", param_blur_size_, param_blur_size_ );
			nh_.param( "filters/blur/distribution", param_blur_dist_, param_blur_dist_ );
			nh_.param( "filters/pose_low_pass/position_alpha", param_low_pass_pos_a_, param_low_pass_pos_a_ );
			nh_.param( "filters/pose_low_pass/size_alpha", param_low_pass_size_a_, param_low_pass_size_a_ );

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

				double mid_hue = ( param_hsv_max_hue_ + (param_hsv_min_hue_ > param_hsv_max_hue_ ? param_hsv_min_hue_ + 360.0 : param_hsv_min_hue_ ) ) / 2.0;
				double mid_sat = ( param_hsv_max_sat_ + param_hsv_min_sat_ ) / 2.0;
				double mid_val = ( param_hsv_max_val_ + param_hsv_min_val_ ) / 2.0;

				visualization_msgs::Marker marker;
				marker.header.frame_id = camera_info_.header.frame_id;
				marker.header.stamp = ros::Time::now();

				marker.ns = param_ball_name_;
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

				marker.scale.x = param_ball_size_;
				marker.scale.y = param_ball_size_;
				marker.scale.z = param_ball_size_;

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
			if( param_hsv_min_hue_ < param_hsv_max_hue_ ) {
				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( param_hsv_min_hue_ / 2.0 ),
										 (uint8_t)( param_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( param_hsv_max_hue_ / 2.0 ),
										 (uint8_t)( param_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_max_val_ * 255.0 ) ),
							 hsv_image);
			} else {
				cv::Mat low_range;
				cv::Mat high_range;

				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( param_hsv_min_hue_ / 2.0 ),
										 (uint8_t)( param_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( 0.0 ),
										 (uint8_t)( param_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_max_val_ * 255.0 ) ),
							 low_range);

				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( 0.0 ),
										 (uint8_t)( param_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( param_hsv_max_hue_ / 2.0 ),
										 (uint8_t)( param_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_max_val_ * 255.0 ) ),
							 high_range);

				cv::bitwise_or( low_range, high_range, hsv_image );
			}

			//Perform an Openning and Closing morphology transform to fill in and smooth out the ball
			cv::Mat stuct_el_open = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( param_morph_open_size_, param_morph_open_size_ ) );
			cv::Mat stuct_el_close = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( param_morph_close_size_, param_morph_close_size_ ) );
			cv::morphologyEx( hsv_image, hsv_image, cv::MORPH_OPEN, stuct_el_open );	//Apply the specified morphology operation
			cv::morphologyEx( hsv_image, hsv_image, cv::MORPH_CLOSE, stuct_el_close );	//Apply the specified morphology operation

			//Blur slightly to increase chances of detection
			cv::GaussianBlur(hsv_image, hsv_image, cv::Size(param_blur_size_, param_blur_size_), param_blur_dist_);

			// Use the Hough transform to detect circles in the combined threshold image
			std::vector<cv::Vec3f> found_circles;

			cv::HoughCircles(hsv_image,
							 found_circles,
							 CV_HOUGH_GRADIENT,
							 param_hough_accum_,
							 hsv_image.rows / param_hough_min_dist_,
							 param_hough_param1_,
							 param_hough_param2_,
							 param_hough_min_rad_,
							 param_hough_max_rad_);

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
				//TODO: Red for detected position and size
				//TODO: Blue for filtered position and size
				if( found_circles.size() > 0 ) {
					for( size_t current_circle = 0; current_circle < found_circles.size(); ++current_circle ) {
						cv::Point center( std::round( found_circles[current_circle][0] ), std::round( found_circles[current_circle][1] ) );
						int radius = std::round( found_circles[current_circle][2] );
						cv::circle( cv_ptr->image, center, radius, cv::Scalar( 255, 0, 0 ), 5 );
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
