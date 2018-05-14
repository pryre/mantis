#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>

#include <mantis_gazebo_msgs/DoArm.h>

#include <vector>
#include <string>

#define BATTERY_CELLS 4

#define VOLTAGE_LIPO_CELL 4.2


namespace gazebo
{
class MantisGazeboPWMMotor : public ModelPlugin
{
	public:
		MantisGazeboPWMMotor() : ModelPlugin(),
								 param_pwm_min_(1000),
								 param_pwm_max_(2000),
								 param_motor_vel_max_(1618),
								 param_motor_vel_armed_(100),
								 safety_armed_(false) {
		}

		int32_t int32_constrain(const int32_t i, const int32_t min, const int32_t max) {
			return (i < min) ? min : (i > max) ? max : i;
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
			// Make sure the ROS node for Gazebo has already been initialized
			if( !ros::isInitialized() ) {
			  ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.");
			  return;
			}

			model_ = _parent;

			sub_rc_out_ = nh_.subscribe<mavros_msgs::OverrideRCIn>( model_->GetName() + "/command/motor_pwm", 100, &MantisGazeboPWMMotor::rc_out_cb, this );
			pub_motor_velocity_ = nh_.advertise<std_msgs::Float64MultiArray>(model_->GetName() + "/command/motor_velocity", 10);

			srv_arm_motors_ = nh_.advertiseService(model_->GetName() + "/arm/motors", &MantisGazeboPWMMotor::arm_safety_srv, this);

			timer_state_ = nh_.createTimer(ros::Duration(1.0), &MantisGazeboPWMMotor::callback_state, this );
			pub_state_ = nh_.advertise<mavros_msgs::State>(model_->GetName() + "/state", 10);
			pub_battery_ = nh_.advertise<sensor_msgs::BatteryState>(model_->GetName() + "/battery", 10);

			ROS_INFO("Loaded mantis pwm motor plugin!");
		}

		void callback_state( const ros::TimerEvent& e ) {
			mavros_msgs::State state_out;
			sensor_msgs::BatteryState battery_out;

			state_out.header.stamp = e.current_real;
			state_out.header.frame_id = model_->GetName();
			state_out.connected = true;
			state_out.armed = safety_armed_;
			state_out.guided = true;
			state_out.mode = "AUTO";
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

		bool arm_safety_srv( mantis_gazebo_msgs::DoArm::Request  &req,
							 mantis_gazebo_msgs::DoArm::Response &res ) {

			safety_armed_ = req.arm;

			if( safety_armed_ ) {
				ROS_INFO("Mantis: Motors armed!");
			} else {
				ROS_INFO("Mantis: Motors disarmed!");
			}

			res.success = true;

			return true;
		}

	private:
		physics::ModelPtr model_;	// Pointer to the model

		uint16_t param_pwm_min_;
		uint16_t param_pwm_max_;
		double param_motor_vel_max_;	//TODO: Params
		double param_motor_vel_armed_;

		bool safety_armed_;

		ros::NodeHandle nh_;
		ros::Subscriber sub_rc_out_;
		ros::Publisher pub_motor_velocity_;
		ros::ServiceServer srv_arm_motors_;

		ros::Timer timer_state_;
		ros::Publisher pub_state_;
		ros::Publisher pub_battery_;

		std_msgs::Float64MultiArray msg_motor_velocity_;
};

GZ_REGISTER_MODEL_PLUGIN(MantisGazeboPWMMotor)
}
