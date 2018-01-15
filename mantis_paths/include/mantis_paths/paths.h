#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>

class Paths {
	private:
		ros::NodeHandle nh_;

		ros::Publisher pub_path_;

		nav_msgs::Path msg_out_;
		int running_seq_;
		ros::Time running_stamp_;

		std::string param_frame_id_;
		int param_arc_res_;
		Eigen::Vector3d param_start_position_;
		Eigen::Quaterniond param_start_direction_;
		Eigen::Quaterniond param_start_orientation_;

	public:
		Paths( void );

		~Paths( void );

		bool load_start_params(void);
		bool generate_path();

		ros::Duration travel_time(const double len, const double vel);
		void add_pose(const ros::Duration dt, const geometry_msgs::Pose pose);

		geometry_msgs::Pose pose_from_eigen(const Eigen::Vector3d p, const Eigen::Quaterniond q);
		Eigen::Vector3d vector_from_doubles(std::vector<double> &a);
		Eigen::Quaterniond quaternion_from_doubles(std::vector<double> &a);
};
