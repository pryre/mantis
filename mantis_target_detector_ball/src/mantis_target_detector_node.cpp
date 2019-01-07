#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>

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

	tf2_ros::TransformBroadcaster tfbr_;

	bool got_camera_info_;
	bool param_camera_rectified_;
	sensor_msgs::CameraInfo camera_info_;
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;

	cv::Vec3f detected_ball_filtered_;
	std::vector<cv::Point3d> model_points_;

	bool param_show_marker_;
	visualization_msgs::Marker ball_marker_;

	std::string param_ball_name_;
	double param_ball_diameter_;
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
	BallDetector()
		: nh_( ros::this_node::getName() )
		, it_( nh_ )
		, param_ball_name_( "ball" )
		, param_ball_diameter_( 1.0 )
		, param_hough_accum_( 2 )
		, param_hough_min_dist_( 4 )
		, param_hough_param1_( 100 )
		, param_hough_param2_( 40 )
		, param_hough_min_rad_( 50 )
		, param_hough_max_rad_( 100 )
		, param_morph_open_size_( 11 )
		, param_morph_close_size_( 11 )
		, param_blur_size_( 9 )
		, param_blur_dist_( 2 )
		, param_low_pass_pos_a_( 0.2 )
		, param_low_pass_size_a_( 0.2 )
		, param_camera_rectified_( false )
		, param_show_marker_( false )
		, got_camera_info_( false )
		, topic_input_camera_info_( "input_camera_info" )
		, topic_input_image_( "input_image" )
		, topic_output_debug_image_( "debug_image" )
		, topic_output_overlay_image_( "overlay_image" )
		, topic_output_detect_( "target_pose" )
		, topic_output_marker_( "target_marker" ) {

		// Get parameters, or if not defined, use the defaults
		nh_.param( "topic_input_camera_info", topic_input_camera_info_,
			topic_input_camera_info_ );
		nh_.param( "topic_input_image", topic_input_image_, topic_input_image_ );
		nh_.param( "topic_output_debug_image", topic_output_debug_image_,
			topic_output_debug_image_ );
		nh_.param( "topic_output_overlay_image", topic_output_overlay_image_,
			topic_output_overlay_image_ );
		nh_.param( "topic_output_detect", topic_output_detect_,
			topic_output_detect_ ); // We use this topic to send our detection
		// results
		nh_.param( "topic_output_marker", topic_output_marker_,
			topic_output_marker_ );

		// Subscrive to input video feed and publish output video feed
		sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>(
			topic_input_camera_info_, 100, &BallDetector::camera_info_cb, this );
		pub_debug_image_ = it_.advertise( topic_output_debug_image_, 100 );
		pub_overlay_image_ = it_.advertise( topic_output_overlay_image_, 100 );
		pub_detect_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_detect_, 100 );

		nh_.param( "ball_description/name", param_ball_name_, param_ball_name_ );
		nh_.param( "ball_description/diameter", param_ball_diameter_,
			param_ball_diameter_ );
		nh_.param( "ball_description/hsv_min/hue", param_hsv_min_hue_,
			param_hsv_min_hue_ );
		nh_.param( "ball_description/hsv_min/sat", param_hsv_min_sat_,
			param_hsv_min_sat_ );
		nh_.param( "ball_description/hsv_min/val", param_hsv_min_val_,
			param_hsv_min_val_ );
		nh_.param( "ball_description/hsv_max/hue", param_hsv_max_hue_,
			param_hsv_max_hue_ );
		nh_.param( "ball_description/hsv_max/sat", param_hsv_max_sat_,
			param_hsv_max_sat_ );
		nh_.param( "ball_description/hsv_max/val", param_hsv_max_val_,
			param_hsv_max_val_ );

		nh_.param( "hough_detector/accum_scale", param_hough_accum_,
			param_hough_accum_ );
		nh_.param( "hough_detector/min_dist_scale", param_hough_min_dist_,
			param_hough_min_dist_ );
		nh_.param( "hough_detector/param1", param_hough_param1_,
			param_hough_param1_ );
		nh_.param( "hough_detector/param2", param_hough_param2_,
			param_hough_param2_ );
		nh_.param( "hough_detector/min_radius", param_hough_min_rad_,
			param_hough_min_rad_ );
		nh_.param( "hough_detector/max_radius", param_hough_max_rad_,
			param_hough_max_rad_ );

		nh_.param( "filters/openning/size", param_morph_open_size_,
			param_morph_open_size_ );
		nh_.param( "filters/closing/size", param_morph_close_size_,
			param_morph_close_size_ );
		nh_.param( "filters/blur/size", param_blur_size_, param_blur_size_ );
		nh_.param( "filters/blur/distribution", param_blur_dist_, param_blur_dist_ );
		nh_.param( "filters/pose_low_pass/position_alpha", param_low_pass_pos_a_,
			param_low_pass_pos_a_ );
		nh_.param( "filters/pose_low_pass/size_alpha", param_low_pass_size_a_,
			param_low_pass_size_a_ );

		double ball_rad = param_ball_diameter_ / 2.0;
		model_points_.push_back( cv::Point3d( 0.0, 0.0, 0.0 ) ); // Center
		model_points_.push_back( cv::Point3d( -ball_rad, ball_rad, 0.0 ) ); // Top Left
		model_points_.push_back( cv::Point3d( ball_rad, ball_rad, 0.0 ) ); // Top Right
		model_points_.push_back(
			cv::Point3d( -ball_rad, -ball_rad, 0.0 ) ); // Bottom Left
		model_points_.push_back(
			cv::Point3d( ball_rad, -ball_rad, 0.0 ) ); // Bottom Right

		nh_.param( "camera_is_rectified", param_camera_rectified_,
			param_camera_rectified_ );

		nh_.param( "show_marker", param_show_marker_, param_show_marker_ );

		// TODO: Move this to another node
		if ( param_show_marker_ ) {
			pub_marker_ = nh_.advertise<visualization_msgs::Marker>(
				topic_output_marker_, 1, true );

			double mid_hue = ( param_hsv_max_hue_ + ( param_hsv_min_hue_ > param_hsv_max_hue_
															? param_hsv_min_hue_ + 360.0
															: param_hsv_min_hue_ ) )
				/ 2.0;
			double mid_sat = ( param_hsv_max_sat_ + param_hsv_min_sat_ ) / 2.0;
			double mid_val = ( param_hsv_max_val_ + param_hsv_min_val_ ) / 2.0;

			ball_marker_.header.frame_id = param_ball_name_;
			ball_marker_.header.stamp = ros::Time::now();

			ball_marker_.ns = param_ball_name_;
			ball_marker_.id = 0;
			ball_marker_.type = visualization_msgs::Marker::SPHERE;
			ball_marker_.action = visualization_msgs::Marker::ADD;
			ball_marker_.lifetime = ros::Duration();
			ball_marker_.frame_locked = true;

			ball_marker_.pose.position.x = 0;
			ball_marker_.pose.position.y = 0;
			ball_marker_.pose.position.z = 0;
			ball_marker_.pose.orientation.x = 0.0;
			ball_marker_.pose.orientation.y = 0.0;
			ball_marker_.pose.orientation.z = 0.0;
			ball_marker_.pose.orientation.w = 1.0;

			ball_marker_.scale.x = param_ball_diameter_;
			ball_marker_.scale.y = param_ball_diameter_;
			ball_marker_.scale.z = param_ball_diameter_;

			// Convert the average HSV detection to a RGBA value for ROS
			// CV::HSV:
			//	H: 0->180
			//	S: 0->255
			//	V: 0->255
			// CV::RGB:
			//	R: 0->255
			//	G: 0->255
			//	B: 0->255
			// ROS::RGBA:
			//	R: 0->1.0
			//	G: 0->1.0
			//	B: 0->1.0
			//	A: 0->1.0
			cv::Scalar mid_hsv( ( uint8_t )( ( mid_hue / 2 ) ), ( uint8_t )( mid_sat * 255.0 ),
				( uint8_t )( mid_val * 255.0 ) );
			cv::Mat tmp_mat( 1, 1, CV_8UC3, mid_hsv );
			cv::cvtColor( tmp_mat, tmp_mat, CV_HSV2BGR );
			cv::Scalar mid_rgb = tmp_mat.at<cv::Vec3b>( 0, 0 );

			ball_marker_.color.r = ( (double)mid_rgb.val[2] ) / 255;
			ball_marker_.color.g = ( (double)mid_rgb.val[1] ) / 255;
			ball_marker_.color.b = ( (double)mid_rgb.val[0] ) / 255;
			ball_marker_.color.a = 1.0;

			pub_marker_.publish( ball_marker_ );
		}

		ROS_INFO( "Waiting for camera info..." );

		while ( !got_camera_info_ && ros::ok() ) {
			ros::spinOnce();
			ros::Rate( 20 ).sleep();
		}

		ROS_INFO( "Recieved camera info!" );

		sub_image_ = it_.subscribe( topic_input_image_, 100, &BallDetector::image_cb, this );

		ROS_INFO( "Beginning detection..." );
	}

	~BallDetector() {
		// This message won't actually send here, as the node will have already shut
		// down
		ROS_INFO( "Shutting down..." );
	}

	// We can use the camera info to correct for distortions, get image size, and
	// other useful things
	void camera_info_cb( const sensor_msgs::CameraInfo::ConstPtr& msg_in ) {
		camera_info_ = *msg_in;

		// XXX: Here we are relying on the definition that ROS and OpenCV are both
		// expecting 1x5 vectors
		cv::Mat_<double>( msg_in->D ).reshape( 0, 1 ).copyTo( dist_coeffs_ ); // Create a 3xN matrix with the raw data and copy the
		// data to the right location

		cv::Mat_<double> m;
		if ( param_camera_rectified_ ) {
			m.push_back( msg_in->P[0] );
			m.push_back( msg_in->P[1] );
			m.push_back( msg_in->P[2] );
			m.push_back( msg_in->P[4] );
			m.push_back( msg_in->P[5] );
			m.push_back( msg_in->P[6] );
			m.push_back( msg_in->P[8] );
			m.push_back( msg_in->P[9] );
			m.push_back( msg_in->P[10] );
		} else {
			for ( int i = 0; i < 9; i++ ) // Copy the raw data into the matrix
				m.push_back( msg_in->K[i] );
		}

		m.reshape( 0, 3 ).copyTo( camera_matrix_ ); // Reshape to 3x3 and copy the data
		// to the right location

		got_camera_info_ = true; // Allow processing to begin
	}

	void image_cb( const sensor_msgs::Image::ConstPtr& msg_in ) {
		cv_bridge::CvImagePtr cv_ptr;

		try {
			cv_ptr = cv_bridge::toCvCopy( msg_in, sensor_msgs::image_encodings::BGR8 );
		} catch ( cv_bridge::Exception& e ) {
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}

		// Copy image to HSV format, and to keep clean image for later
		cv::Mat hsv_image;
		cv::cvtColor( cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV );

		// Create a spectrum mask
		if ( param_hsv_min_hue_ < param_hsv_max_hue_ ) {
			cv::inRange( hsv_image, cv::Scalar( ( uint8_t )( param_hsv_min_hue_ / 2.0 ),
										( uint8_t )( param_hsv_min_sat_ * 255.0 ),
										( uint8_t )( param_hsv_min_val_ * 255.0 ) ),
				cv::Scalar( ( uint8_t )( param_hsv_max_hue_ / 2.0 ),
							 ( uint8_t )( param_hsv_max_sat_ * 255.0 ),
							 ( uint8_t )( param_hsv_max_val_ * 255.0 ) ),
				hsv_image );
		} else {
			cv::Mat low_range;
			cv::Mat high_range;

			cv::inRange( hsv_image, cv::Scalar( ( uint8_t )( param_hsv_min_hue_ / 2.0 ),
										( uint8_t )( param_hsv_min_sat_ * 255.0 ),
										( uint8_t )( param_hsv_min_val_ * 255.0 ) ),
				cv::Scalar( ( uint8_t )( 0.0 ),
							 ( uint8_t )( param_hsv_max_sat_ * 255.0 ),
							 ( uint8_t )( param_hsv_max_val_ * 255.0 ) ),
				low_range );

			cv::inRange( hsv_image, cv::Scalar( ( uint8_t )( 0.0 ),
										( uint8_t )( param_hsv_min_sat_ * 255.0 ),
										( uint8_t )( param_hsv_min_val_ * 255.0 ) ),
				cv::Scalar( ( uint8_t )( param_hsv_max_hue_ / 2.0 ),
							 ( uint8_t )( param_hsv_max_sat_ * 255.0 ),
							 ( uint8_t )( param_hsv_max_val_ * 255.0 ) ),
				high_range );

			cv::bitwise_or( low_range, high_range, hsv_image );
		}

		// Perform an Openning and Closing morphology transform to fill in and
		// smooth out the ball
		cv::Mat stuct_el_open = getStructuringElement(
			cv::MORPH_ELLIPSE,
			cv::Size( param_morph_open_size_, param_morph_open_size_ ) );
		cv::Mat stuct_el_close = getStructuringElement(
			cv::MORPH_ELLIPSE,
			cv::Size( param_morph_close_size_, param_morph_close_size_ ) );
		cv::morphologyEx( hsv_image, hsv_image, cv::MORPH_OPEN,
			stuct_el_open ); // Apply the specified morphology operation
		cv::morphologyEx(
			hsv_image, hsv_image, cv::MORPH_CLOSE,
			stuct_el_close ); // Apply the specified morphology operation

		// Blur slightly to increase chances of detection
		cv::GaussianBlur( hsv_image, hsv_image,
			cv::Size( param_blur_size_, param_blur_size_ ),
			param_blur_dist_ );

		// Use the Hough transform to detect circles in the combined threshold image
		std::vector<cv::Vec3f> found_circles;

		cv::HoughCircles( hsv_image, found_circles, CV_HOUGH_GRADIENT,
			param_hough_accum_, hsv_image.rows / param_hough_min_dist_,
			param_hough_param1_, param_hough_param2_,
			param_hough_min_rad_, param_hough_max_rad_ );

		cv::Vec3f detected_ball_raw;

		if ( found_circles.size() > 0 ) {
			// XXX: Ideally there should ounly be one circle found, pick out the best
			// one here
			detected_ball_raw = found_circles.at( 0 );

			// Filter reading
			detected_ball_filtered_[0] = detected_ball_filtered_[0] - ( param_low_pass_pos_a_ * ( detected_ball_filtered_[0] - detected_ball_raw[0] ) );
			detected_ball_filtered_[1] = detected_ball_filtered_[1] - ( param_low_pass_pos_a_ * ( detected_ball_filtered_[1] - detected_ball_raw[1] ) );
			detected_ball_filtered_[2] = detected_ball_filtered_[2] - ( param_low_pass_size_a_ * ( detected_ball_filtered_[2] - detected_ball_raw[2] ) );

			// Take circle estimate, and transform it to a 2D Square
			std::vector<cv::Point2d> image_points;

			// TODO: Should the be rounded?
			double p_x = std::round( detected_ball_filtered_[0] ); // Center of circle X
			double p_y = std::round( detected_ball_filtered_[1] ); // Center of circle Y
			double p_d = std::round( detected_ball_filtered_[2] ); // Radius of circle

			image_points.push_back( cv::Point2d( p_x, p_y ) ); // Center
			image_points.push_back( cv::Point2d( p_x - p_d, p_y + p_d ) ); // Top Left
			image_points.push_back( cv::Point2d( p_x + p_d, p_y + p_d ) ); // Top Right
			image_points.push_back( cv::Point2d( p_x - p_d, p_y - p_d ) ); // Bottom Left
			image_points.push_back( cv::Point2d( p_x + p_d, p_y - p_d ) ); // Bottom Right

			// Output rotation and translation
			cv::Mat rvec; // Rotation in axis-angle form
			cv::Mat tvec;

			// Solve for pose
			cv::solvePnP( model_points_, image_points, camera_matrix_, dist_coeffs_,
				rvec, tvec );

			// Transmit the detection message
			geometry_msgs::PoseStamped msg_out;

			msg_out.header.frame_id = camera_info_.header.frame_id;
			msg_out.header.stamp = msg_in->header.stamp;

			msg_out.pose.position.x = tvec.at<double>( 0, 0 );
			msg_out.pose.position.y = tvec.at<double>( 1, 0 );
			msg_out.pose.position.z = tvec.at<double>( 2, 0 );

			// We don't have any orientation data about the ball, so don't even bother
			msg_out.pose.orientation.w = 1.0;
			msg_out.pose.orientation.x = 0.0;
			msg_out.pose.orientation.y = 0.0;
			msg_out.pose.orientation.z = 0.0;

			geometry_msgs::TransformStamped tf_out;
			tf_out.header = msg_out.header;
			tf_out.child_frame_id = param_ball_name_;
			tf_out.transform.translation.x = msg_out.pose.position.x;
			tf_out.transform.translation.y = msg_out.pose.position.y;
			tf_out.transform.translation.z = msg_out.pose.position.z;
			tf_out.transform.rotation.w = msg_out.pose.orientation.w;
			tf_out.transform.rotation.x = msg_out.pose.orientation.x;
			tf_out.transform.rotation.y = msg_out.pose.orientation.y;
			tf_out.transform.rotation.z = msg_out.pose.orientation.z;

			pub_detect_.publish( msg_out );
			tfbr_.sendTransform( tf_out );
		}

		// Only compute and send the debug image if a node is subscribed
		if ( pub_debug_image_.getNumSubscribers() > 0 ) {
			//==-- Output modified video stream
			pub_debug_image_.publish(
				cv_bridge::CvImage( msg_in->header, "mono8", hsv_image ).toImageMsg() );
		}

		// Only compute and send the overlay image if a node is subscribed
		if ( pub_overlay_image_.getNumSubscribers() > 0 ) {
			if ( found_circles.size() > 0 ) {
				// Green for filtered position and size
				for ( size_t current_circle = 0; current_circle < found_circles.size();
					  ++current_circle ) {
					cv::Point center( std::round( found_circles[current_circle][0] ),
						std::round( found_circles[current_circle][1] ) );
					int radius = std::round( found_circles[current_circle][2] );
					cv::circle( cv_ptr->image, center, radius, cv::Scalar( 0, 255, 0 ), 5 );
				}

				// Red for detected position and size
				cv::Point raw_center( std::round( detected_ball_raw[0] ),
					std::round( detected_ball_raw[1] ) );
				int raw_radius = std::round( detected_ball_raw[2] );
				cv::circle( cv_ptr->image, raw_center, raw_radius, cv::Scalar( 255, 0, 0 ),
					5 );

				// Blue for filtered position and size
				cv::Point filtered_center( std::round( detected_ball_filtered_[0] ),
					std::round( detected_ball_filtered_[1] ) );
				int filtered_radius = std::round( detected_ball_filtered_[2] );
				cv::circle( cv_ptr->image, filtered_center, filtered_radius,
					cv::Scalar( 0, 0, 255 ), 5 );
			}

			//==-- Output modified video stream
			pub_overlay_image_.publish( cv_ptr->toImageMsg() );
		}
	}
};

int main( int argc, char** argv ) {
	ros::init( argc, argv, "goal_detector" );
	BallDetector bd;

	ros::spin();

	return 0;
}
