#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <mantis_gazebo_msgs/DoArm.h>

namespace gazebo
{
class MantisGazeboServo : public ModelPlugin
{
	public:
		MantisGazeboServo() : ModelPlugin(),
							  safety_armed_(false) {
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			// Make sure the ROS node for Gazebo has already been initialized
			if( !ros::isInitialized() ) {
			  ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  return;
			}

			model_ = _parent;
			updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind( &MantisGazeboServo::OnUpdate, this, _1 ) );
			joint_shoulder_upperarm_servo_ = model_->GetJoint("joint_shoulder");
			joint_elbow_forearm_servo_ = model_->GetJoint("joint_elbow");

			joint_shoulder_upperarm_servo_->SetPosition(0, -1.570796327);

			sub_shoulder_ = nh_.subscribe<std_msgs::Float64>( model_->GetName() + "/command/servo/torque/shoulder", 100, &MantisGazeboServo::cmd_shoulder_cb, this );
			sub_elbow_ = nh_.subscribe<std_msgs::Float64>( model_->GetName() + "/command/servo/torque/elbow", 100, &MantisGazeboServo::cmd_elbow_cb, this );
			pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>(model_->GetName() + "/state/joint_states", 10);

			msg_joint_state_.header.frame_id = model_->GetName();
			msg_joint_state_.name.push_back("shoulder");
			msg_joint_state_.name.push_back("elbow");
			msg_joint_state_.position.push_back(joint_shoulder_upperarm_servo_->GetAngle(0).Radian());
			msg_joint_state_.position.push_back(joint_elbow_forearm_servo_->GetAngle(0).Radian());
			msg_joint_state_.velocity.push_back(joint_shoulder_upperarm_servo_->GetVelocity(0));
			msg_joint_state_.velocity.push_back(joint_elbow_forearm_servo_->GetVelocity(0));
			msg_joint_state_.effort.push_back(0);
			msg_joint_state_.effort.push_back(0);

			srv_arm_servos_ = nh_.advertiseService(model_->GetName() + "/arm/servos", &MantisGazeboServo::arm_safety_srv, this);

			ROS_INFO("Loaded mantis servo plugin!");
		}

		void cmd_shoulder_cb( const std_msgs::Float64::ConstPtr& msg_in ) {
			joint_effort_cmd_shoulder_ = msg_in->data;
		}

		void cmd_elbow_cb( const std_msgs::Float64::ConstPtr& msg_in ) {
			joint_effort_cmd_elbow_ = msg_in->data;
		}

		// Called by the world update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/) {
			double cmd_shoulder = 0.0;
			double cmd_elbow = 0.0;

			if(safety_armed_) {
				cmd_shoulder = joint_effort_cmd_shoulder_;
				cmd_elbow = joint_effort_cmd_elbow_;
			}

			joint_shoulder_upperarm_servo_->SetForce(0, cmd_shoulder);
			joint_elbow_forearm_servo_->SetForce(0, cmd_elbow);

			msg_joint_state_.header.stamp = ros::Time::now();
			msg_joint_state_.position[0] = joint_shoulder_upperarm_servo_->GetAngle(0).Radian();
			msg_joint_state_.position[1] = joint_elbow_forearm_servo_->GetAngle(0).Radian();
			msg_joint_state_.velocity[0] = joint_shoulder_upperarm_servo_->GetVelocity(0);
			msg_joint_state_.velocity[1] = joint_elbow_forearm_servo_->GetVelocity(0);
			msg_joint_state_.effort[0] = cmd_shoulder;
			msg_joint_state_.effort[1] = cmd_elbow;

			pub_joint_state_.publish(msg_joint_state_);
		}

		bool arm_safety_srv( mantis_gazebo_msgs::DoArm::Request  &req,
							 mantis_gazebo_msgs::DoArm::Response &res ) {

			safety_armed_ = req.arm;

			if( safety_armed_ ) {
				ROS_INFO("Mantis: Servos armed!");
			} else {
				ROS_INFO("Mantis: Servos disarmed!");
			}

			res.success = true;

			return true;
		}

	private:
		physics::ModelPtr model_;	// Pointer to the model
		event::ConnectionPtr updateConnection_;	// Pointer to the update event connection

		//pose of every joint
		physics::JointPtr joint_shoulder_upperarm_servo_;
		physics::JointPtr joint_elbow_forearm_servo_;

		double joint_effort_cmd_shoulder_;
		double joint_effort_cmd_elbow_;

		bool safety_armed_;

		ros::NodeHandle nh_;
		ros::Subscriber sub_shoulder_;
		ros::Subscriber sub_elbow_;
		ros::Publisher pub_joint_state_;
		ros::ServiceServer srv_arm_servos_;

		sensor_msgs::JointState msg_joint_state_;
};

GZ_REGISTER_MODEL_PLUGIN(MantisGazeboServo)
}
