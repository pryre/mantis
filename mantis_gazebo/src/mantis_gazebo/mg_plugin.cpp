#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <mantis_gazebo/DoArm.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <dh_parameters/dh_parameters.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>

#include <eigen3/Eigen/Dense>

#include <string>
#include <vector>
#include <math.h>

#define BATTERY_CELLS 4
#define VOLTAGE_LIPO_CELL 4.2

namespace gazebo
{
class MantisGazeboPlugin : public ModelPlugin
{

	private:
		physics::ModelPtr model_;	// Pointer to the model

		ros::NodeHandle nh_;

		//Arming Service
		ros::ServiceServer srv_arm_motors_;
		bool safety_armed_;

		//Model Odom
		ros::Timer timer_odom_;
		ros::Publisher pub_odom_;

		//PWM Motor
		ros::Subscriber sub_rc_out_;
		ros::Publisher pub_motor_velocity_;

		//Additional Sensors
		ros::Timer timer_state_;
		ros::Publisher pub_state_;
		ros::Publisher pub_battery_;

		//Parameters
		uint16_t param_pwm_min_;
		uint16_t param_pwm_max_;
		double param_motor_vel_max_;	//TODO: Params
		double param_motor_vel_armed_;

		std::string topic_odom_;

		std::string parent_name_;
		nav_msgs::Odometry msg_odom_;

		std_msgs::Float64MultiArray msg_motor_velocity_;

	public:
		MantisGazeboPlugin() : ModelPlugin(),
								 param_pwm_min_(1000),
								 param_pwm_max_(2000),
								 param_motor_vel_max_(1618),
								 param_motor_vel_armed_(100),
								 safety_armed_(false),
								 parent_name_("map") {
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			model_ = _parent;
			nh_ = ros::NodeHandle(model_->GetName());

			// Make sure the ROS node for Gazebo has already been initialized
			if( !ros::isInitialized() ) {
			  ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  return;
			}

			//Arming
			srv_arm_motors_ = nh_.advertiseService("arm/motors", &MantisGazeboPlugin::arm_safety_srv, this);

			//PWM
			sub_rc_out_ = nh_.subscribe<mavros_msgs::OverrideRCIn>("command/motor_pwm", 100, &MantisGazeboPlugin::rc_out_cb, this );
			pub_motor_velocity_ = nh_.advertise<std_msgs::Float64MultiArray>("command/motor_velocity", 10);

			//Additional Sensors
			timer_state_ = nh_.createTimer(ros::Duration(1.0), &MantisGazeboPlugin::callback_state, this );
			pub_state_ = nh_.advertise<mavros_msgs::State>("state/mav_state", 10);
			pub_battery_ = nh_.advertise<sensor_msgs::BatteryState>("state/battery", 10);

			//Model Odom
			timer_odom_ = nh_.createTimer(ros::Duration(0.01), &MantisGazeboPlugin::callback_odom, this );
			pub_odom_ = nh_.advertise<nav_msgs::Odometry>( "state/odom", 10 );

			msg_odom_.header.frame_id = parent_name_;
			msg_odom_.child_frame_id = model_->GetName();

			ROS_INFO("Loaded mantis gazebo plugin!");
		}

	private:
		int32_t int32_constrain(const int32_t i, const int32_t min, const int32_t max) {
			return (i < min) ? min : (i > max) ? max : i;
		}

		void callback_state( const ros::TimerEvent& e ) {
			mavros_msgs::State state_out;
			sensor_msgs::BatteryState battery_out;

			state_out.header.stamp = e.current_real;
			state_out.header.frame_id = model_->GetName();
			state_out.connected = true;
			state_out.armed = safety_armed_;
			state_out.guided = true;
			state_out.mode = "OFFBOARD";
			if(safety_armed_) {
				state_out.system_status = 4;
			} else {
				state_out.system_status = 3;
			}

			battery_out.header.stamp = e.current_real;
			battery_out.header.frame_id = model_->GetName();
			battery_out.current = gazebo::math::NAN_D;
			battery_out.charge = gazebo::math::NAN_D;
			battery_out.capacity = gazebo::math::NAN_D;
			battery_out.design_capacity = gazebo::math::NAN_D;
			battery_out.percentage = 1.0;
			battery_out.power_supply_status = battery_out.POWER_SUPPLY_STATUS_DISCHARGING;
			battery_out.power_supply_health = battery_out.POWER_SUPPLY_HEALTH_GOOD;
			battery_out.power_supply_technology = battery_out.POWER_SUPPLY_TECHNOLOGY_LIPO;
			double voltage = 0.0;
			for(int i=0; i<BATTERY_CELLS; i++) {
				voltage += VOLTAGE_LIPO_CELL;
				battery_out.cell_voltage.push_back(VOLTAGE_LIPO_CELL);
			}
			battery_out.voltage = voltage;
			battery_out.location = "PLUG0";
			battery_out.serial_number = "00000001";

			pub_state_.publish(state_out);
			pub_battery_.publish(battery_out);
		}

		void rc_out_cb( const mavros_msgs::OverrideRCIn::ConstPtr& msg_in ) {
			msg_motor_velocity_.data.resize( msg_in->channels.size() );

			for(int i=0; i<msg_in->channels.size(); i++){
				if( safety_armed_ ) {
					//Cut down ignored channels
					int pwm_raw = (msg_in->channels[i] != 0 ) && (msg_in->channels[i] != 65535) ? msg_in->channels[i] : 0;

					double pwm = int32_constrain( pwm_raw, param_pwm_min_, param_pwm_max_);

					double norm_ref = ( pwm - param_pwm_min_ ) / ( param_pwm_max_ - param_pwm_min_ );
					//double cmd_vel = norm_ref * param_motor_vel_max_;
					double cmd_vel = std::sqrt(norm_ref * param_motor_vel_max_ * param_motor_vel_max_);

					msg_motor_velocity_.data[i] =  (cmd_vel < param_motor_vel_armed_) ? param_motor_vel_armed_ : cmd_vel;
				} else {
					msg_motor_velocity_.data[i] = 0;
				}
			}

			pub_motor_velocity_.publish(msg_motor_velocity_);
		}

		bool arm_safety_srv( mantis_gazebo::DoArm::Request  &req,
							 mantis_gazebo::DoArm::Response &res ) {

			safety_armed_ = req.arm;

			if( safety_armed_ ) {
				ROS_INFO("Mantis: Motors armed!");
			} else {
				ROS_INFO("Mantis: Motors disarmed!");
			}

			res.success = true;

			return true;
		}

		void callback_odom( const ros::TimerEvent& e ) {
			msg_odom_.header.stamp = e.current_real;

			msg_odom_.pose.pose.position.x = model_->GetWorldPose().pos.x;
			msg_odom_.pose.pose.position.y = model_->GetWorldPose().pos.y;
			msg_odom_.pose.pose.position.z = model_->GetWorldPose().pos.z;
			msg_odom_.pose.pose.orientation.w = model_->GetWorldPose().rot.w;
			msg_odom_.pose.pose.orientation.x = model_->GetWorldPose().rot.x;
			msg_odom_.pose.pose.orientation.y = model_->GetWorldPose().rot.y;
			msg_odom_.pose.pose.orientation.z = model_->GetWorldPose().rot.z;

			msg_odom_.twist.twist.linear.x = model_->GetRelativeLinearVel().x;
			msg_odom_.twist.twist.linear.y = model_->GetRelativeLinearVel().y;
			msg_odom_.twist.twist.linear.z = model_->GetRelativeLinearVel().z;
			msg_odom_.twist.twist.angular.x = model_->GetRelativeAngularVel().x;
			msg_odom_.twist.twist.angular.y = model_->GetRelativeAngularVel().y;
			msg_odom_.twist.twist.angular.z = model_->GetRelativeAngularVel().z;

			pub_odom_.publish(msg_odom_);
		}

		void statesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg_in) {
			msg_odom_.header.stamp = ros::Time::now();

			for( int i=0; i<msg_in->name.size(); i++ ) {
				if( msg_in->name[i] == model_->GetName() ) {
					//Velocities must be in the base_link frame
					//The base_link frame is locked in roll and pitch with the world frame, but can rotate with yaw
					tf2::Quaternion rot_l_vel;
					tf2::Quaternion rot_a_vel;

					//Get the rotation from the world to body frame (for angular measurements
					tf2::Quaternion tmp_q( msg_in->pose[i].orientation.x,
										   msg_in->pose[i].orientation.y,
										   msg_in->pose[i].orientation.z,
										   msg_in->pose[i].orientation.w );

					rot_a_vel = ( tmp_q.inverse() * tf2::Quaternion( msg_in->twist[i].angular.x,
														   msg_in->twist[i].angular.y,
														   msg_in->twist[i].angular.z,
														   0.0 ) ) * tmp_q;

					//Derotate the world->body quaternion
					tf2::Matrix3x3 r( tmp_q );
					tf2::Vector3 body_x;
					tf2::Vector3 body_y( r.getRow(1) );	//Use the y vector for yaw reference
					tf2::Vector3 body_z(0.0, 0.0, 1.0);

					body_x = body_y.cross(body_z);
					body_x.normalize();
					body_y = body_z.cross(body_x);
					body_y.normalize();
					r.setValue( body_x.x(), body_x.y(), body_x.z(), body_y.x(), body_y.y(), body_y.z(), body_z.x(), body_z.y(), body_z.z() );

					r.getRotation( tmp_q );	//Copy the rotation back into the quaternion

					rot_l_vel = ( tmp_q.inverse() * tf2::Quaternion( msg_in->twist[i].linear.x,
														   msg_in->twist[i].linear.y,
														   msg_in->twist[i].linear.z,
														   0.0 ) ) * tmp_q;

					msg_odom_.pose.pose = msg_in->pose[i];

					msg_odom_.twist.twist.linear.x = rot_l_vel.getX();
					msg_odom_.twist.twist.linear.y = rot_l_vel.getY();
					msg_odom_.twist.twist.linear.z = rot_l_vel.getZ();
					msg_odom_.twist.twist.angular.x = rot_a_vel.getX();
					msg_odom_.twist.twist.angular.y = rot_a_vel.getY();
					msg_odom_.twist.twist.angular.z = rot_a_vel.getZ();

					pub_odom_.publish(msg_odom_);

					break;
				}
			}
		}
};

GZ_REGISTER_MODEL_PLUGIN(MantisGazeboPlugin)
}
