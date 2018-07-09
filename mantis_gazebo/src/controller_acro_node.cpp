#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Imu.h>

#include <mantis_description/se_tools.h>
#include <mantis_description/param_client.h>
#include <pid_controller_lib/pidController.h>

#include <eigen3/Eigen/Dense>

#include <string>

typedef struct controlCommand_s {
	double T;
	double r;
	double p;
	double y;
} controlCommand_t;

class ControllerAcro {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		MantisParamClient p_;

		ros::Subscriber sub_attitude_target_;
		ros::Subscriber sub_actuator_control_;
		ros::Subscriber sub_imu_;
		ros::Publisher pub_rc_out_;
		ros::Publisher pub_attitude_target_;

		std::string model_name_;
		sensor_msgs::Imu model_imu_;

		ros::Timer tmr_rc_out_;
		double pwm_update_rate_;
		std::string frame_id_;

		double force_comp_timeout_;

		mavros_msgs::AttitudeTarget goal_att_;
		mavros_msgs::ActuatorControl force_compensation_;

		double param_ang_r_ff_;
		double param_ang_p_ff_;
		double param_ang_y_ff_;
		double param_rate_r_max_;
		double param_rate_p_max_;
		double param_rate_y_max_;
		pidController controller_rates_x_;
		pidController controller_rates_y_;
		pidController controller_rates_z_;

		controlCommand_t control_angle_;
		controlCommand_t control_rates_;
		controlCommand_t control_forces_;

	public:
		int32_t int32_constrain(const int32_t i, const int32_t min, const int32_t max) {
			return (i < min) ? min : (i > max) ? max : i;
		}

		double double_constrain(const double i, const double min, const double max) {
			return (i < min) ? min : (i > max) ? max : i;
		}

		Eigen::Vector3d calc_ang_error(const Eigen::Matrix3d &R_sp, const Eigen::Matrix3d &R) {
			Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

			//Method derived from px4 attitude controller:
			//DCM from for state and setpoint

			//Calculate shortest path to goal rotation without yaw (as it's slower than roll/pitch)
			Eigen::Vector3d R_z = R.col(2);
			Eigen::Vector3d R_sp_z = R_sp.col(2);

			//px4: axis and sin(angle) of desired rotation
			//px4: math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);
			Eigen::Vector3d e_R = R.transpose() * R_z.cross(R_sp_z);

			double e_R_z_sin = e_R.norm();
			double e_R_z_cos = R_z.dot(R_sp_z);

			//px4: calculate rotation matrix after roll/pitch only rotation
			Eigen::Matrix3d R_rp;

			if(e_R_z_sin > 0) {
				//px4: get axis-angle representation
				Eigen::Vector3d e_R_z_axis = e_R / e_R_z_sin;
				e_R = e_R_z_axis * std::atan2(e_R_z_sin, e_R_z_cos);

				//px4: cross product matrix for e_R_axis
				Eigen::Matrix3d e_R_cp;
				e_R_cp(0,1) = -e_R_z_axis.z();
				e_R_cp(0,2) = e_R_z_axis.y();
				e_R_cp(1,0) = e_R_z_axis.z();
				e_R_cp(1,2) = -e_R_z_axis.x();
				e_R_cp(2,0) = -e_R_z_axis.y();
				e_R_cp(2,1) = e_R_z_axis.x();

				//px4: rotation matrix for roll/pitch only rotation
				R_rp = R * ( I + (e_R_cp * e_R_z_sin) + ( (e_R_cp * e_R_cp) * (1.0 - e_R_z_cos) ) );
			} else {
				//px4: zero roll/pitch rotation
				R_rp = R;
			}

			//px4: R_rp and R_sp has the same Z axis, calculate yaw error
			Eigen::Vector3d R_sp_x = R_sp.col(0);
			Eigen::Vector3d R_rp_x = R_rp.col(0);

			//px4: calculate weight for yaw control
			double yaw_w = e_R_z_cos * e_R_z_cos;

			//ROS_INFO_STREAM("e_R_z_cos: " << e_R_z_cos << std::endl);

			//px4: e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;
			//Eigen::Vector3d R_rp_c_sp = R_rp_x.cross(R_sp_x);
			//e_R(2) = std::atan2(R_rp_c_sp.dot(R_sp_z), R_rp_x.dot(R_sp_x)) * yaw_w;
			e_R(2) = std::atan2( (R_rp_x.cross(R_sp_x)).dot(R_sp_z), R_rp_x.dot(R_sp_x)) * yaw_w;

			if(e_R_z_cos < 0) {
				//px4: for large thrust vector rotations use another rotation method:
				//For normal operation, this implies a roll/pitch change greater than pi
				//Should never be an issue for us
				ROS_WARN_THROTTLE(1.0, "Large thrust vector detected!");
			}

			return e_R;
		}

		void mixerCallback(const ros::TimerEvent& event) {
			double dt = (event.current_real - event.last_real).toSec();

			mavros_msgs::AttitudeTarget msg_att_out;
			msg_att_out.header.stamp = event.current_real;
			msg_att_out.header.frame_id = frame_id_;
			msg_att_out.type_mask = goal_att_.type_mask;

			if(!( goal_att_.type_mask & goal_att_.IGNORE_THRUST)) {
				control_angle_.T = goal_att_.thrust;
				control_rates_.T = goal_att_.thrust;
				control_forces_.T = goal_att_.thrust;
			} else {
				control_angle_.T = 0.0;
				control_rates_.T = 0.0;
				control_forces_.T = 0.0;
			}

			if(!( goal_att_.type_mask & goal_att_.IGNORE_ATTITUDE)) {
				msg_att_out.orientation = goal_att_.orientation;
				Eigen::Quaterniond q_s = quaternion_from_msg(goal_att_.orientation);
				Eigen::Quaterniond q_c = quaternion_from_msg(model_imu_.orientation);

				Eigen::Vector3d e_R = calc_ang_error(q_s.toRotationMatrix(), q_c.toRotationMatrix());

				control_angle_.r = e_R.x();
				control_angle_.p = e_R.y();
				control_angle_.y = e_R.z();
			} else {
				msg_att_out.orientation.w = 1.0;
				msg_att_out.orientation.x = 0.0;
				msg_att_out.orientation.y = 0.0;
				msg_att_out.orientation.z = 0.0;

				control_angle_.r = 0.0;
				control_angle_.p = 0.0;
				control_angle_.y = 0.0;
			}

			if(!( goal_att_.type_mask & goal_att_.IGNORE_ROLL_RATE )) {
				control_rates_.r = goal_att_.body_rate.x;
			} else {
				control_rates_.r = param_ang_r_ff_ * control_angle_.r;
			}

			if(!( goal_att_.type_mask & goal_att_.IGNORE_PITCH_RATE )) {
				control_rates_.p = goal_att_.body_rate.y;
			} else {
				control_rates_.p = param_ang_p_ff_ * control_angle_.p;
			}

			if(!( goal_att_.type_mask & goal_att_.IGNORE_YAW_RATE )) {
				control_rates_.y = goal_att_.body_rate.z;
			} else {
				control_rates_.y = param_ang_y_ff_ * control_angle_.y;
			}

			double_constrain(control_rates_.r, param_rate_r_max_, param_rate_r_max_);
			double_constrain(control_rates_.p, param_rate_p_max_, param_rate_p_max_);
			double_constrain(control_rates_.y, param_rate_y_max_, param_rate_y_max_);

			control_forces_.r = controller_rates_x_.step( dt, control_rates_.r, model_imu_.angular_velocity.x );
			control_forces_.p = controller_rates_y_.step( dt, control_rates_.p, model_imu_.angular_velocity.y );
			control_forces_.y = controller_rates_z_.step( dt, control_rates_.y, model_imu_.angular_velocity.z );

			msg_att_out.body_rate.x = control_rates_.r;
			msg_att_out.body_rate.y = control_rates_.p;
			msg_att_out.body_rate.z = control_rates_.y;

			//Check to see if the force compensation message is still recent
			if( (event.current_real - force_compensation_.header.stamp).toSec() < force_comp_timeout_) {
				//Add in the force compensation
				control_forces_.r += force_compensation_.controls[0];
				control_forces_.p += force_compensation_.controls[1];
				control_forces_.y += force_compensation_.controls[2];
			}

			//Perform the PWM mapping
			int32_t max_output = 1000;
			int32_t scale_factor = 1000;
			int32_t prescaled_outputs[p_.motor_num()];
			int32_t pwm_output_requested[p_.motor_num()];

			Eigen::Vector4d cforces;
			cforces << control_forces_.T, control_forces_.r, control_forces_.p, control_forces_.y;
			Eigen::VectorXd thrust_calc = p_.get_mixer()*cforces;

			for (uint8_t i=0; i<p_.motor_num(); i++) {
				prescaled_outputs[i] = (int)(thrust_calc(i) * 1000);

				//If the thrust is 0, zero motor outputs, as we don't want any thrust at all for safety
				if( control_forces_.T <= 0.05 )
					prescaled_outputs[i] = 0;

				if( prescaled_outputs[i] > max_output )
					max_output = prescaled_outputs[i];
			}

			// saturate outputs to maintain controllability even during aggressive maneuvers
			if (max_output > 1000)
				scale_factor = 1000 * 1000 / max_output;

			scale_factor = 1000;

			mavros_msgs::OverrideRCIn msg_rc_out;

			for (uint8_t i=0; i<p_.motor_num(); i++) {
					 int32_t channel_out = (prescaled_outputs[i] * scale_factor / 1000); // divide by scale factor
					 msg_rc_out.channels[i] = 1000 + int32_constrain( channel_out, 0, 1000 );
			}

			pub_rc_out_.publish(msg_rc_out);
			pub_attitude_target_.publish(msg_att_out);
		}

		void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in) {
			model_imu_ = *msg_in;
		}

		void forceCompCallback(const mavros_msgs::ActuatorControl::ConstPtr& msg_in) {
			force_compensation_ = *msg_in;
		}

		void attitudeTargetCallback( const mavros_msgs::AttitudeTarget::ConstPtr& msg_in ) {
			goal_att_ = *msg_in;
		}

		ControllerAcro() :
			nh_(),
			nhp_( "~" ),
			p_(&nh_),
			frame_id_("map"),
			model_name_("mantis_uav"),
			pwm_update_rate_(250.0),
			force_comp_timeout_(0.1),
			controller_rates_x_( &nhp_, "rates/x"),
			controller_rates_y_( &nhp_, "rates/y"),
			controller_rates_z_( &nhp_, "rates/z") {

			sub_attitude_target_ = nh_.subscribe<mavros_msgs::AttitudeTarget>("command/attitude", 1, &ControllerAcro::attitudeTargetCallback, this);
			pub_attitude_target_ = nhp_.advertise<mavros_msgs::AttitudeTarget>( "feedback/attitude_target", 10 );
			sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("state/imu_data", 10, &ControllerAcro::imuCallback, this);
			pub_rc_out_ = nh_.advertise<mavros_msgs::OverrideRCIn>( "command/motor_pwm", 10 );
			sub_actuator_control_ = nh_.subscribe<mavros_msgs::ActuatorControl>("command/force_compensation", 1, &ControllerAcro::forceCompCallback, this);

			tmr_rc_out_ = nh_.createTimer(ros::Duration(1 / pwm_update_rate_), &ControllerAcro::mixerCallback, this);

			param_ang_r_ff_ = 6.0;
			param_ang_p_ff_ = 6.0;
			param_ang_y_ff_ = 2.8;
			param_rate_r_max_ = 3.85;	//~220 deg/s
			param_rate_p_max_ = 3.85;	//~220 deg/s
			param_rate_y_max_ = 1.05;	//~60 deg/s

			ROS_INFO("Listening for outputting motor commands...");
		}

		~ControllerAcro() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller_acro");
	ControllerAcro ca;

	ros::spin();

	return 0;
}
