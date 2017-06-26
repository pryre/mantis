#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

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

		bool got_camera_info_;
		sensor_msgs::CameraInfo camera_info_;

		std::string topic_input_camera_info_;
		std::string topic_input_image_;
		std::string topic_output_debug_image_;
		std::string topic_output_overlay_image_;
		std::string topic_output_detect_;

	public:
		BallDetector() :
			nh_( ros::this_node::getName() ),
			it_(nh_),
			got_camera_info_( false ),
			topic_input_camera_info_( "input_camera_info" ),
			topic_input_image_( "input_image" ),
			topic_output_debug_image_( "debug_image" ),
			topic_output_overlay_image_( "overlay_image" ),
			topic_output_detect_( "circles" ) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "topic_input_camera_info", topic_input_camera_info_, topic_input_camera_info_ );
			nh_.param( "topic_input_image", topic_input_image_, topic_input_image_ );
			nh_.param( "topic_output_debug_image", topic_output_debug_image_, topic_output_debug_image_ );
			nh_.param( "topic_output_overlay_image", topic_output_overlay_image_, topic_output_overlay_image_ );
			nh_.param( "topic_output_detect", topic_output_detect_, topic_output_detect_ );	//We use this topic to send our detection results

			// Subscrive to input video feed and publish output video feed
			sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo> ( topic_input_camera_info_, 100, &Processor::camera_info_cb, this );
			pub_debug_image_ = it_.advertise(topic_output_debug_image_, 100);
			pub_overlay_image_ = it_.advertise(topic_output_overlay_image_, 100);
			pub_detect_ = nh_.advertise<std_msgs::Float64MultiArray>( topic_output_detect_, 100 );

			ROS_INFO("Waiting for camera info...");

			while( !got_camera_info_ && ros::ok() ) {
				ros::spinOnce();
				ros::Rate(20).sleep();
			}

			ROS_INFO("Recieved camera info!");

			sub_image_ = it_.subscribe(topic_input_image_, 100, &Processor::image_cb, this);

			ROS_INFO("Begining detection...");
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
				cv_ptr = cv_bridge::toCvCopy(msg_in, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			//To give an example of the processing, here is some basic red circle detection from:
			//	https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/

			//Convert input image to HSV
			cv::Mat hsv_image;
			cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
			//Threshold the HSV image, keep only the red pixels
			cv::Mat lower_red_hue_range;
			cv::Mat upper_red_hue_range;
			cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
			cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
			//Combine the above two images
			cv::Mat red_hue_image;
			cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
			//Blur slightly to increase chances of detection
			cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
			// Use the Hough transform to detect circles in the combined threshold image
			std::vector<cv::Vec3f> circles;
			cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

			if(circles.size() > 0) {
				//Transmit the detection message
				std_msgs::Float64MultiArray msg_out;
				std_msgs::MultiArrayDimension tmp_dim;

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

				pub_detect_.publish( msg_out );
			}

			//Only compute and send the debug image if a node is subscribed
			if( pub_debug_image_.getNumSubscribers() > 0 ) {
				//==-- Output modified video stream
				pub_debug_image_.publish( cv_bridge::CvImage( msg_in->header, "mono8", red_hue_image ).toImageMsg() );
			}

			//Only compute and send the overlay image if a node is subscribed
			if( pub_overlay_image_.getNumSubscribers() > 0 ) {
				if( circles.size() > 0 ) {
					for( size_t current_circle = 0; current_circle < circles.size(); ++current_circle ) {
						cv::Point center( std::round( circles[current_circle][0] ), std::round( circles[current_circle][1] ) );
						int radius = std::round( circles[current_circle][2] );
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
