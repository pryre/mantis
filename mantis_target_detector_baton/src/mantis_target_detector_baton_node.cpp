#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Time.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <iostream>

class BatonDetector {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		image_transport::ImageTransport it_;

		ros::Subscriber sub_camera_info_;
		image_transport::Subscriber sub_image_;
		image_transport::Publisher pub_debug_image_;
		image_transport::Publisher pub_overlay_image_;
		ros::Publisher pub_detect_;

		tf2_ros::TransformBroadcaster tfbr_;

		bool got_camera_info_;
		bool param_camera_rectified_;
		sensor_msgs::CameraInfo camera_info_;
		cv::Mat camera_matrix_;
		cv::Mat dist_coeffs_;

		cv::Vec3f detected_baton_filtered_;
		std::vector<cv::Point3d> model_points_;

		std::string param_baton_name_;
		double param_baton_length_;
		double param_baton_radius_;
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

		std::string param_camera_namespace_;

	public:
		BatonDetector() :
			nh_(),
			nhp_("~"),
			it_(nhp_),
			param_baton_name_( "baton" ),
			param_baton_length_( 0.12 ),
			param_baton_radius_(0.01),
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
			param_camera_rectified_( false ),
			got_camera_info_( false ),
			param_camera_namespace_( "camera" ) {

			//Get parameters, or if not defined, use the defaults
			nhp_.param( "camera_namespace", param_camera_namespace_, param_camera_namespace_ );

			// Subscribe to input video feed and publish output video feed
			sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo> ( param_camera_namespace_ + "/camera_info", 100, &BatonDetector::camera_info_cb, this );
			pub_debug_image_ = it_.advertise( "debug_image", 10);
			pub_overlay_image_ = it_.advertise( "overlay_image", 10);
			pub_detect_ = nh_.advertise<std_msgs::Time>( "target_found", 10 );

			nhp_.param( "baton_description/name", param_baton_name_, param_baton_name_ );
			nhp_.param( "baton_description/length", param_baton_length_, param_baton_length_ );
			nhp_.param( "baton_description/radius", param_baton_radius_, param_baton_radius_ );
			nhp_.param( "baton_description/hsv_min/hue", param_hsv_min_hue_, param_hsv_min_hue_ );
			nhp_.param( "baton_description/hsv_min/sat", param_hsv_min_sat_, param_hsv_min_sat_ );
			nhp_.param( "baton_description/hsv_min/val", param_hsv_min_val_, param_hsv_min_val_ );
			nhp_.param( "baton_description/hsv_max/hue", param_hsv_max_hue_, param_hsv_max_hue_ );
			nhp_.param( "baton_description/hsv_max/sat", param_hsv_max_sat_, param_hsv_max_sat_ );
			nhp_.param( "baton_description/hsv_max/val", param_hsv_max_val_, param_hsv_max_val_ );

			nhp_.param( "hough_detector/accum_scale", param_hough_accum_, param_hough_accum_);
			nhp_.param( "hough_detector/min_dist_scale", param_hough_min_dist_, param_hough_min_dist_ );
			nhp_.param( "hough_detector/param1", param_hough_param1_, param_hough_param1_ );
			nhp_.param( "hough_detector/param2", param_hough_param2_, param_hough_param2_ );
			nhp_.param( "hough_detector/min_radius", param_hough_min_rad_, param_hough_min_rad_ );
			nhp_.param( "hough_detector/max_radius", param_hough_max_rad_, param_hough_max_rad_ );

			nhp_.param( "filters/openning/size", param_morph_open_size_, param_morph_open_size_ );
			nhp_.param( "filters/closing/size", param_morph_close_size_, param_morph_close_size_ );
			nhp_.param( "filters/blur/size", param_blur_size_, param_blur_size_ );
			nhp_.param( "filters/blur/distribution", param_blur_dist_, param_blur_dist_ );
			nhp_.param( "filters/pose_low_pass/position_alpha", param_low_pass_pos_a_, param_low_pass_pos_a_ );
			nhp_.param( "filters/pose_low_pass/size_alpha", param_low_pass_size_a_, param_low_pass_size_a_ );

			model_points_.push_back( cv::Point3d( 0.0, 0.0, 0.0 ) );	//Center
			model_points_.push_back( cv::Point3d( 0.0, param_baton_radius_, param_baton_length_ / 2 ) );	//Top Left
			model_points_.push_back( cv::Point3d( 0.0, -param_baton_radius_, param_baton_length_ / 2 ) );	//Top Right
			model_points_.push_back( cv::Point3d( 0.0, param_baton_radius_, -param_baton_length_ / 2 ) );	//Bottom Left
			model_points_.push_back( cv::Point3d( 0.0, -param_baton_radius_, -param_baton_length_ / 2 ) );	//Bottom Right

			nh_.param( "camera_is_rectified", param_camera_rectified_, param_camera_rectified_ );

			ROS_INFO("Waiting for camera info...");

			while( !got_camera_info_ && ros::ok() ) {
				ros::spinOnce();
				ros::Rate(20).sleep();
			}

			ROS_INFO("Recieved camera info!");

			sub_image_ = it_.subscribe(param_camera_namespace_ + "/image_raw", 100, &BatonDetector::image_cb, this);

			ROS_INFO("Beginning detection...");
		}

		~BatonDetector() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		//We can use the camera info to correct for distortions, get image size, and other useful things
		void camera_info_cb( const sensor_msgs::CameraInfo::ConstPtr& msg_in ) {
			camera_info_ = *msg_in;

			//XXX: Here we are relying on the definition that ROS and OpenCV are both expecting 1x5 vectors
			cv::Mat_<double>( msg_in->D ).reshape( 0, 1 ).copyTo( dist_coeffs_ );	//Create a 3xN matrix with the raw data and copy the data to the right location

			cv::Mat_<double> m;
			if( param_camera_rectified_ ) {
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
				for(int i = 0; i < 9; i++)	//Copy the raw data into the matrix
					m.push_back( msg_in->K[i] );
			}

			m.reshape( 0, 3 ).copyTo( camera_matrix_ );	//Reshape to 3x3 and copy the data to the right location

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

			cv::Mat mask_image;
			//Create a spectrum mask
			if( param_hsv_min_hue_ < param_hsv_max_hue_ ) {
				cv::inRange( hsv_image,
							 cv::Scalar( (uint8_t)( param_hsv_min_hue_ / 2.0 ),
										 (uint8_t)( param_hsv_min_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_min_val_ * 255.0 ) ),
							 cv::Scalar( (uint8_t)( param_hsv_max_hue_ / 2.0 ),
										 (uint8_t)( param_hsv_max_sat_ * 255.0 ),
										 (uint8_t)( param_hsv_max_val_ * 255.0 ) ),
							 mask_image);
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

				cv::bitwise_or( low_range, high_range, mask_image );
			}

			//Perform an Openning and Closing morphology transform to fill in and smooth out the ball
			cv::Mat stuct_el_open = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( param_morph_open_size_, param_morph_open_size_ ) );
			cv::Mat stuct_el_close = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( param_morph_close_size_, param_morph_close_size_ ) );
			cv::morphologyEx( mask_image, mask_image, cv::MORPH_OPEN, stuct_el_open );	//Apply the specified morphology operation
			cv::morphologyEx( mask_image, mask_image, cv::MORPH_CLOSE, stuct_el_close );	//Apply the specified morphology operation

			//Blur slightly to increase chances of detection
			cv::GaussianBlur(mask_image, mask_image, cv::Size(param_blur_size_, param_blur_size_), param_blur_dist_);

			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			std::vector<std::vector<cv::Point>> polys;

			// Detect edges using canny
			int thresh = 100;
			cv::Canny( mask_image, mask_image, thresh, thresh*2, 3 );
			// Find contours
			cv::findContours( mask_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

			polys.resize(contours.size());
			for( int i=0; i<contours.size(); i++ ) {
				cv::approxPolyDP(cv::Mat(contours[i]), polys[i], 3, false);
				if(polys[i].size() == 4) {
					ROS_INFO("Square!");
				}
			}

			bool have_estimate = false;
			std::vector<cv::Point> detected_baton_raw;

			for( int i=0; i<polys.size(); i++ ) {
				//XXX: Ideally there should ounly be one circle found
				//pick out the "best" one here
				//in this case, the fisrt square/rectangle found
				if(polys[i].size() == 4) {

					detected_baton_raw = polys.at(0);

					//Filter reading
					/*
					detected_ball_filtered_[0] = detected_ball_filtered_[0] - ( param_low_pass_pos_a_ * ( detected_ball_filtered_[0] - detected_ball_raw[0] ) ) ;
					detected_ball_filtered_[1] = detected_ball_filtered_[1] - ( param_low_pass_pos_a_ * ( detected_ball_filtered_[1] - detected_ball_raw[1] ) ) ;
					detected_ball_filtered_[2] = detected_ball_filtered_[2] - ( param_low_pass_size_a_ * ( detected_ball_filtered_[2] - detected_ball_raw[2] ) ) ;


					//TODO: Should the be rounded?
					double p_x = std::round( detected_ball_filtered_[0] );	//Center of circle X
					double p_y = std::round( detected_ball_filtered_[1] );	//Center of circle Y
					double p_d = std::round( detected_ball_filtered_[2] );	//Radius of circle
					*/

					//Take circle estimate, and transform it to a 2D Square
					std::vector<cv::Point2d> image_points;
				/*
					image_points.push_back( cv::Point2d( p_x, p_y ) );	//Center
					image_points.push_back( cv::Point2d( p_x - p_d, p_y + p_d ) );	//Top Left
					image_points.push_back( cv::Point2d( p_x + p_d, p_y + p_d ) );	//Top Right
					image_points.push_back( cv::Point2d( p_x - p_d, p_y - p_d ) );	//Bottom Left
					image_points.push_back( cv::Point2d( p_x + p_d, p_y - p_d ) );	//Bottom Right

					// Output rotation and translation
					cv::Mat rvec; // Rotation in axis-angle form
					cv::Mat tvec;

					// Solve for pose
					cv::solvePnP( model_points_,
								  image_points,
								  camera_matrix_,
								  dist_coeffs_,
								  rvec,
								  tvec );

					//Transmit the detection message
					geometry_msgs::PoseStamped msg_out;

					msg_out.header.frame_id = camera_info_.header.frame_id;
					msg_out.header.stamp = msg_in->header.stamp;

					msg_out.pose.position.x = tvec.at<double>(0,0);
					msg_out.pose.position.y = tvec.at<double>(1,0);
					msg_out.pose.position.z = tvec.at<double>(2,0);

					//We don't have any orientation data about the ball, so don't even bother
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
				*/
					have_estimate = true;
					break;
				}
			}

			//Only compute and send the debug image if a node is subscribed
			if( pub_debug_image_.getNumSubscribers() > 0 ) {
				//==-- Output modified video stream
				pub_debug_image_.publish( cv_bridge::CvImage( msg_in->header, "mono8", mask_image ).toImageMsg() );
			}

			//Only compute and send the overlay image if a node is subscribed
			if( pub_overlay_image_.getNumSubscribers() > 0 ) {
				cv::RNG rng(12345);

				for( int i=0; i<contours.size(); i++ ) {
					drawContours( cv_ptr->image, polys, i, cv::Scalar(0,0,255), 2, 4, hierarchy, 0, cv::Point() );
				}

				if(have_estimate) {

				}

					/*
					//Green for filtered position and size
					for( size_t current_circle = 0; current_circle < found_circles.size(); ++current_circle ) {
						cv::Point center( std::round( found_circles[current_circle][0] ), std::round( found_circles[current_circle][1] ) );
						int radius = std::round( found_circles[current_circle][2] );
						cv::circle( cv_ptr->image, center, radius, cv::Scalar( 0, 255, 0 ), 5 );
					}

					//Red for detected position and size
					cv::Point raw_center( std::round( detected_ball_raw[0] ), std::round( detected_ball_raw[1] ) );
					int raw_radius = std::round( detected_ball_raw[2] );
					cv::circle( cv_ptr->image, raw_center, raw_radius, cv::Scalar( 255, 0, 0 ), 5 );

					//Blue for filtered position and size
					cv::Point filtered_center( std::round( detected_ball_filtered_[0] ), std::round( detected_ball_filtered_[1] ) );
					int filtered_radius = std::round( detected_ball_filtered_[2] );
					cv::circle( cv_ptr->image, filtered_center, filtered_radius, cv::Scalar( 0, 0, 255 ), 5 );
					*/

				//==-- Output modified video stream
				pub_overlay_image_.publish( cv_ptr->toImageMsg() );
			}
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "baton_detector");
	BatonDetector bd;

	ros::spin();

	return 0;
}
