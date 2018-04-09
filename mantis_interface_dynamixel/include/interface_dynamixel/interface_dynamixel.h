#pragma once

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <mantis_interface_dynamixel/EnableTorque.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <math.h>


enum dynamixel_mode_t {
	MOTOR_MODE_TORQUE = 0,
	MOTOR_MODE_VELOCITY = 1,
	MOTOR_MODE_POSITION = 3,
	MOTOR_MODE_INVALID = 255
};

enum dynamixel_control_items_t {
	DCI_TORQUE_ENABLE,
	DCI_OPERATING_MODE,
	DCI_PROFILE_VELOCITY,
	DCI_PROFILE_ACCELERATION,
	DCI_PRESENT_POSITION,
	DCI_PRESENT_VELOCITY,
	DCI_PRESENT_CURRENT,
	DCI_GOAL_POSITION,
	DCI_GOAL_VELOCITY,
	DCI_GOAL_CURRENT,
	DCI_NUM_ITEMS
};

class InterfaceDynamixel {
	private:
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Publisher pub_states_;
		ros::Subscriber sub_setpoints_;
		ros::ServiceServer srv_enable_torque_specific_;
		ros::ServiceServer srv_enable_torque_all_;

		sensor_msgs::JointState joint_states_;
		sensor_msgs::JointState joint_setpoints_;
		uint8_t motor_output_mode_;

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
		//bool flag_setpoints_received_;

		// Dynamixel Workbench Parameters
		//std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
		//dynamixel_multi_driver::DynamixelMultiDriver *dynamixel_multi_driver_;
		dynamixel::PortHandler *portHandler_;
		dynamixel::PacketHandler *packetHandler_;
		std::vector<DynamixelTool> dxl_;
		std::vector<std::string> dxl_names_;
		std::vector<ControlTableItem*> dynamixel_control_items_;
		//dynamixel::GroupSyncWrite groupSyncWrite;
		//dynamixel::GroupSyncRead groupSyncRead;

		//dynamixel_write_value_t *dynamixel_write_value_;
		//dynamixel_motor_pos_t *dynamixel_motor_pos_;

	public:
		InterfaceDynamixel( void );

		~InterfaceDynamixel( void );

	private:
		//ROS
		void callback_timer(const ros::TimerEvent& e);
		void callback_setpoints(const sensor_msgs::JointState::ConstPtr& msg_in);
		bool enable_torque_specific(mantis_interface_dynamixel::EnableTorque::Request& req, mantis_interface_dynamixel::EnableTorque::Response& res);
		bool enable_torque_all(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

		//Interfacing
		bool load_dynamixel();
		bool check_load_dynamixel();
		void shutdown_node( void );
		uint8_t get_id( int motor_number );
		void init_motor(std::string motor_model, uint8_t motor_id, double protocol_version, std::string motor_name);
		bool add_motors();

		//Control
		bool set_torque_enable(int motor_number, bool onoff);

		//bool readMotorState(std::string addr_name, int motor_number, int64_t *read_value);
		bool readMotorState(dynamixel_control_items_t item_id, int motor_number, int64_t *read_value);
		bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);

		//bool writeMotorState(std::string addr_name, int motor_number, uint32_t write_value);
		bool writeMotorState(dynamixel_control_items_t item_id, int motor_number, uint32_t write_value);
		bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t value);

		void initSyncRead();
		bool doSyncRead(std::vector<std::vector<std::int32_t>> *states);
		//void doSyncWrite(std::string addr_name);
		void doSyncWrite(dynamixel_control_items_t item_id);

		//bool bulk_read_states(std::vector<std::string> *states, std::vector<std::vector<int32_t>> *result);

		//Conversion
		int convert_torque_value(double torque, int motor_number);
		double convert_value_torque(int value, int motor_number);
		int convert_velocity_value(double velocity, int motor_number);	//rad/s to value rpm
		double convert_value_velocity(int value, int motor_number);	//Reading to rad/s
		int32_t convert_radian_value(double radian, int motor_number);
		double convert_value_radian(int32_t value, int motor_number);
};
