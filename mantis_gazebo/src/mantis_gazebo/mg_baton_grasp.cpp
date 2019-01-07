#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>

#include <math.h>
#include <std_srvs/Trigger.h>

namespace gazebo {
class MantisGazeboBatonGrasp : public ModelPlugin {
private:
	physics::ModelPtr model_; // Pointer to the model
	// event::ConnectionPtr updateConnection_;	// Pointer to the update event
	// connection

	ros::NodeHandle nh_;
	ros::ServiceServer srv_do_grasp_;

public:
	MantisGazeboBatonGrasp()
		: ModelPlugin() {
	}

	void Load( physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/ ) {
		// Make sure the ROS node for Gazebo has already been initialized
		if ( !ros::isInitialized() ) {
			ROS_FATAL( "A ROS node for Gazebo has not been initialized, unable to "
					   "load plugin." );
			return;
		}

		model_ = _parent;
		// updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(
		// &MantisGazeboServo::OnUpdate, this, _1 ) );
		// Need a better way to do these

		srv_do_grasp_ = nh_.advertiseService( model_->GetName() + "/do_grasp",
			&MantisGazeboBatonGrasp::callback_do_grasp, this );

		ROS_INFO( "Loaded mantis baton grasping plugin!" );
	}

	// Called by the world update start event
	// void OnUpdate(const common::UpdateInfo & /*_info*/) {
	//}

private:
	bool callback_do_grasp( std_srvs::Trigger::Request& req,
		std_srvs::Trigger::Response& res ) {

		physics::LinkPtr link_baton = model_->GetWorld()
										  ->GetModel( "baton_and_stand::baton" )
										  ->GetLink( "baton" );
		physics::LinkPtr link_end = model_->GetLink( "link_forearm" );
		physics::JointPtr joint_grasp = model_->GetJoint( "joint_baton_grasp" );
		/*
    std::vector<physics::LinkPtr> links = model_->GetLinks();
    for(int i=0; i < links.size(); i++) {
            ROS_INFO("Link found: %s", links[i]->GetName().c_str());
    }
    */
		if ( ( link_baton != NULL ) && ( link_end != NULL ) && ( joint_grasp == NULL ) ) {
			// Move the ball so it is at the grasp point
			math::Pose end_effector_pose = math::Pose( 0.2, 0.0, 0.0, M_PI / 2.0, 0.0, 0.0 ) + link_end->GetWorldPose();

			physics::JointPtr joint;
			// link_ball->SetCollideMode("none");
			link_baton->SetSelfCollide( false );
			link_baton->SetWorldPose( end_effector_pose );
			// link_ball->SetRelativePose(math::Pose(0.18325,0.0,0.0,0.0,0.0,0.0));
			joint = model_->CreateJoint( "joint_baton_grasp", "fixed", link_end,
				link_baton );
			// joint->SetAnchor(0, ignition::math::Vector3d(0.18325,0.0,0.0));
			// link_ball->SetRelativePose(math::Pose(0.0,0.0,0.0,0.0,0.0,0.0));

			res.success = true;
		} else {
			if ( joint_grasp != NULL ) {
				ROS_INFO( "Releasing baton!" );
				joint_grasp->Detach();
				model_->RemoveJoint( joint_grasp->GetName() );
				// ROS_ERROR("Mantis: Grasp already done, cannot redo!");
			}

			if ( link_baton == NULL )
				ROS_ERROR( "Mantis: Unable to find baton link, cannot grasp!" );

			if ( link_end == NULL )
				ROS_ERROR( "Mantis: Unable to find end link, cannot grasp!" );

			res.success = false;
		}

		return true;
	}
};

GZ_REGISTER_MODEL_PLUGIN( MantisGazeboBatonGrasp )
}
