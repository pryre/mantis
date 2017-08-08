#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/RCOut.h>

#include <mantis_gazebo_msgs/DoArmMotors.h>

#include <vector>
#include <string>

namespace gazebo
{
class MantisGazeboPWMMotor : public ModelPlugin
{
	public:
		MantisGazeboPWMMotor() : ModelPlugin(),
								 param_pwm_min_(1000),
								 param_pwm_max_(2000),
								 param_motor_vel_max_(1200),
								 param_motor_vel_armed_(100),
								 safety_armed_(false) {
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			// Make sure the ROS node for Gazebo has already been initialized
			if( !ros::isInitialized() ) {
			  ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  return;
			}

			model_ = _parent;

			sub_rc_out_ = nh_.subscribe<mavros_msgs::RCOut>( model_->GetName() + "/command/motor_pwm", 100, &MantisGazeboPWMMotor::rc_out_cb, this );
			pub_motor_velocity_ = nh_.advertise<std_msgs::Float64MultiArray>(model_->GetName() + "/command/motor_velocity", 10);

			srv_arm_motors_ = nh_.advertiseService(model_->GetName() + "/arm/motors", &MantisGazeboPWMMotor::arm_safety_srv, this);

			ROS_INFO("Loaded mantis pwm motor plugin!");
		}

		void rc_out_cb( const mavros_msgs::RCOut::ConstPtr& msg_in ) {
			msg_motor_velocity_.data.resize( msg_in->channels.size() );

			for(int i=0; i<msg_in->channels.size(); i++){
				double pwm = param_pwm_min_;

				if( safety_armed_ )
					pwm = ( msg_in->channels[i] < param_pwm_min_ ) ? param_pwm_min_ : ( ( msg_in->channels[i] > param_pwm_max_ ) ? param_pwm_max_ : msg_in->channels[i] );

				double norm_ref = ( pwm - param_pwm_min_ ) / ( param_pwm_max_ - param_pwm_min_ );

				msg_motor_velocity_.data[i] = ( norm_ref * param_motor_vel_max_ ) + param_motor_vel_armed_;
			}

			pub_motor_velocity_.publish(msg_motor_velocity_);
		}

		bool arm_safety_srv( mantis_gazebo_msgs::DoArmMotors::Request  &req,
							 mantis_gazebo_msgs::DoArmMotors::Response &res ) {

			safety_armed_ = req.arm;

			if( safety_armed_ ) {
				ROS_INFO("Mantis: Motors armed!");
			} else {
				ROS_INFO("Mantis: Motors disarmed!");
			}

			return true;
		}

	private:
		physics::ModelPtr model_;	// Pointer to the model

		uint16_t param_pwm_min_;
		uint16_t param_pwm_max_;
		double param_motor_vel_max_;	//TODO: Params
		double param_motor_vel_armed_;
		//TODO: service for motor arm?

		bool safety_armed_;

		ros::NodeHandle nh_;
		ros::Subscriber sub_rc_out_;
		ros::Publisher pub_motor_velocity_;
		ros::ServiceServer srv_arm_motors_;

		std_msgs::Float64MultiArray msg_motor_velocity_;
};

GZ_REGISTER_MODEL_PLUGIN(MantisGazeboPWMMotor)
}
