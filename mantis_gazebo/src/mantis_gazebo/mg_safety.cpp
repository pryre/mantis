#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <mantis_gazebo/DoArm.h>

namespace gazebo
{
class MantisGazeboSafety : public ModelPlugin
{
	public:
		MantisGazeboSafety() : ModelPlugin() {
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			// Make sure the ROS node for Gazebo has already been initialized
			if( !ros::isInitialized() ) {
			  ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  return;
			}

			model_ = _parent;
			srv_arm_system_ = nh_.advertiseService(model_->GetName() + "/arm/system", &MantisGazeboSafety::arm_safety_srv, this);

			cli_arm_motors_ = nh_.serviceClient<mantis_gazebo::DoArm>(model_->GetName() + "/arm/motors");
			cli_arm_servos_ = nh_.serviceClient<mantis_gazebo::DoArm>(model_->GetName() + "/arm/servos");

			ROS_INFO("Loaded mantis safety plugin!");
		}

		bool arm_safety_srv( mantis_gazebo::DoArm::Request  &req,
							 mantis_gazebo::DoArm::Response &res ) {

			mantis_gazebo::DoArm srv_motors;
			mantis_gazebo::DoArm srv_servos;
			srv_motors.request = req;
			srv_servos.request = req;

			bool success_motors = false;
			bool success_servos = false;

			if( req.arm ) {
				ROS_INFO("Mantis: Attempting to arm system...");
			} else {
				ROS_INFO("Mantis: Attempting to disarm system...");
			}

			if(cli_arm_motors_.call(srv_motors)) {
				if(srv_motors.response.success) {
					success_motors = true;
				} else {
					ROS_ERROR("Mantis: Failed to (dis)arm motors!");
				}
			} else {
				ROS_ERROR("Mantis: Failed to call motor safety service.");
			}

			if(cli_arm_servos_.call(srv_servos)) {
				if(srv_servos.response.success) {
					success_servos = true;
				} else {
					ROS_ERROR("Mantis: Failed to (dis)arm servos!");
				}
			} else {
				ROS_ERROR("Mantis: Failed to call servos safety service.");
			}

			res.success = success_motors & success_servos;

			return true;
		}

	private:
		physics::ModelPtr model_;	// Pointer to the model

		ros::NodeHandle nh_;
		ros::ServiceServer srv_arm_system_;
		ros::ServiceClient cli_arm_motors_;
		ros::ServiceClient cli_arm_servos_;
};

GZ_REGISTER_MODEL_PLUGIN(MantisGazeboSafety)
}
