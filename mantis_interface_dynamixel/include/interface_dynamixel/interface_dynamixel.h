#pragma once

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <sensor_msgs/JointState.h>

#include <string>

typedef struct {
  std::vector<uint8_t>  torque;
  std::vector<int16_t>  current;
} dynamixel_write_value_t;

typedef struct {
  std::vector<uint32_t> cur_pos;
  std::vector<uint32_t> des_pos;
} dynamixel_motor_pos_t;

class InterfaceDynamixel {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Publisher pub_states_;
		ros::Subscriber sub_setpoints_;

		sensor_msgs::JointState joint_states_;
		sensor_msgs::JointState joint_setpoints_;

		//Parameters
		std::string topic_input_setpoints_;
		std::string topic_output_states_;
		double param_update_rate_;
		std::string param_port_name_;
		int param_port_buad_;
		double param_port_version_;
		int param_num_motors_;
		std::string param_frame_id_;

		//Flags
		bool flag_setpoints_received_;

		// Dynamixel Workbench Parameters
		//std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
		//dynamixel_multi_driver::DynamixelMultiDriver *dynamixel_multi_driver_;
		dynamixel::PortHandler *portHandler_;
		dynamixel::PacketHandler *packetHandler_;
		std::vector<dynamixel_tool::DynamixelTool> dynamixel_;

		dynamixel_write_value_t *dynamixel_write_value_;
		dynamixel_motor_pos_t *dynamixel_motor_pos_;

	public:
		InterfaceDynamixel( void );

		~InterfaceDynamixel( void );

	private:
		//ROS
		void callback_timer(const ros::TimerEvent& e);
		void callback_setpoints(const sensor_msgs::JointState::ConstPtr& msg_in);

		//Interfacing
		bool load_dynamixel();
		bool check_load_dynamixel();
		void shutdown_node( void );
		void init_motor(std::string motor_model, uint8_t motor_id, double protocol_version);
		bool add_motors();

		bool readMotorState(std::string addr_name, int motor_number, int64_t *read_value);
		bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);

		//Control
		bool enable_torque(bool onoff);
		bool set_current_setpoint(int16_t pan_cur, int16_t tilt_cur);
		bool read_dynamixel_state();
		//bool dynamixelStatePublish();

		//Conversion
		double  convert_value_torque(int16_t value);
		int16_t convert_torque_value(double torque);
		uint32_t convert_radian_value(double radian);
		double convert_value_radian(int32_t value);
};
