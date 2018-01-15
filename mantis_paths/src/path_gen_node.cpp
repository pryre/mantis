#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <math.h>

class PathGen {
	private:
		ros::NodeHandle nh_;

		ros::Publisher pub_path_;

		std::string param_frame_id_;
		double param_duration_;
		int param_resolution_;
		double param_path_height_;
		std::string param_path_type_;
		bool param_orient_forward_;	//TODO

	public:
		PathGen( void );

		~PathGen( void );
};



PathGen::PathGen() :
	nh_("~"),
	param_frame_id_("map"),
	param_duration_(0.0),
	param_resolution_(1),
	param_path_height_(0.0) {

	//Setup publisher and get known parameters
	pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1, true);

	nh_.param("frame_id", param_frame_id_, param_frame_id_);
	nh_.param("duration", param_duration_, param_duration_);
	nh_.param("resolution", param_resolution_, param_resolution_);
	nh_.param("path/type", param_path_type_, param_path_type_);
	nh_.param("path/height", param_path_height_, param_path_height_);

	//Give the node a moment to recieve a clock message (to allow it to work with simulated time)
	ros::Duration(0.2).sleep();
	ros::spinOnce();

	bool error = false;

	//Check that params were loaded correctly
	if( (param_duration_ > 0.0) &&
		(param_resolution_ > 1) &&
		(param_path_height_ > 0.0) &&
		(nh_.getParam("path/type", param_path_type_) ) ) {
			bool success = false;
			nav_msgs::Path msg_out;
			double time_step = param_duration_ / param_resolution_;

			//Find the path to generate
			if(param_path_type_ == "line ") {

				success = true;
			} else if (param_path_type_ == "points") {

				success = true;
			} else if (param_path_type_ == "square") {

				success = true;
			} else if(param_path_type_ == "circle") {
				geometry_msgs::PoseStamped p;
				p.header.frame_id = param_frame_id_;

				p.pose.position.z = param_path_height_;
				p.pose.orientation.w = 1.0;
				p.pose.orientation.x = 0.0;
				p.pose.orientation.y = 0.0;
				p.pose.orientation.z = 0.0;

				double rot_step = 2*M_PI/(param_resolution_);
				double r = 0.0;

				//Fill in the rest of the path
				if(nh_.getParam("path/radius", r)) {
					for(int i=0; i<=(param_resolution_); i++) {
						p.header.seq = i;
						p.header.stamp = ros::Time(0) + ros::Duration((i)*time_step);

						p.pose.position.x = r*std::cos(rot_step*i);
						p.pose.position.y = r*std::sin(rot_step*i);

						msg_out.poses.push_back(p);
					}

					success = true;
				} else {
					ROS_ERROR("Type 'circle' needs parameter 'path/radius'");
				}
			} else if (param_path_type_ == "figure_8") {


				success = true;
			} else {
				error = true;
				ROS_ERROR("Path type (%s) is not supported.", param_path_type_.c_str());
			}

			if(success) {
				msg_out.header.frame_id = param_frame_id_;
				msg_out.header.stamp = ros::Time::now();

				pub_path_.publish(msg_out);

				ROS_INFO("Path generated!");
			} else {
				ROS_ERROR("A path was unable to be generated.");
				error = true;
			}
	} else {
		error = true;
		ROS_ERROR("Invalid base parameters (duration, resolution, or type)");
	}

	if(error)
		ros::shutdown();
}

PathGen::~PathGen() {
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "path_gen");
	PathGen pt;

	ros::spin();

	return 0;
}
