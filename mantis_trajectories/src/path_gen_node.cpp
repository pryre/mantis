#include <ros/ros.h>

#include <mavros_msgs/State.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <dynamic_reconfigure/server.h>
#include <mantis_controller_id/JointGoalsConfig.h>

#include <math.h>

class PathGen {
	private:
		ros::NodeHandle nh_;

		ros::Publisher pub_path_;

		double param_duration_;
		int param_resolution_;
		std::string param_path_type_;

	public:
		PathGen( void );

		~PathGen( void );
};



PathGen::PathGen() :
	nh_("~"),
	param_duration_(0.0),
	param_resolution_(0) {

	//Setup publisher and get known parameters
	pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1);

	nh_.param("path/duration", param_duration_, param_duration_);
	nh_.param("path/resolution", param_resolution_, param_resolution_);
	nh_.param("path/type", param_path_type_, param_path_type_);

	bool error = false;

	//Check that params were loaded correctly
	if( (param_duration_ > 0.0),
		(param_resolution_ > 0),
		(nh_.getParam("path/type", param_path_type_) ) {
			bool success = false;

			//Find the path to generate
			if(param_path_type_ == "line ") {

				success = true;
			} else if (param_path_type_ == "points") {

				success = true;
			} else if (param_path_type_ == "square") {

				success = true;
			} else if(param_path_type_ == "circle") {

				success = true;
			} else if (param_path_type_ == "figure_8") {


				success = true;
			} else {
				error = true;
				ROS_ERROR("Path type (%s) is not supported.", param_path_type_.c_str());
			}

			if(success) {
				//TODO: Send path message


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
