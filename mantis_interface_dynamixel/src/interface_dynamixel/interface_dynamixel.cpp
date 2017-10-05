#include <ros/ros.h>

#include <interface_dynamixel/interface_dynamixel.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>
#include <string>

InterfaceDynamixel::InterfaceDynamixel() :
	nh_("~"),
	param_update_rate_(50.0),
	topic_input_setpoints_("setpoints"),
	topic_output_states_("states"),
	param_port_name_("/dev/ttyUSB0"),
	param_port_buad_(57600),
	param_port_version_(0.0),
	param_num_motors_(0),
	param_frame_id_("robot"),
	//param_motor_model_(""),
	//param_motor_id_(0),
	flag_setpoints_received_(false) {


	//ROS Setup
	nh_.param("topic_input_setpoints", topic_input_setpoints_, topic_input_setpoints_);
	nh_.param("topic_output_states", topic_output_states_, topic_output_states_);
	nh_.param("port_name", param_port_name_, param_port_name_);
	nh_.param("port_baud", param_port_buad_, param_port_buad_);
	nh_.param("protocol", param_port_version_, param_port_version_);
	nh_.param("num_motors", param_num_motors_, param_num_motors_);
	nh_.param("frame_id", param_frame_id_, param_frame_id_);
	nh_.param("update_rate", param_update_rate_, param_update_rate_);

	sub_setpoints_ = nh_.subscribe<sensor_msgs::JointState>( topic_input_setpoints_, 10, &InterfaceDynamixel::callback_setpoints, this );
	pub_states_ = nh_.advertise<sensor_msgs::JointState>(topic_output_states_, 10);


	//Dynamixel Setup
	portHandler_ = dynamixel::PortHandler::getPortHandler( param_port_name_.c_str() );
	packetHandler_ = dynamixel::PacketHandler::getPacketHandler( param_port_version_ );

	bool port_ok = false;

	//Open port
	if ( portHandler_->openPort() ) {
		ROS_INFO( "Succeeded to open the port(%s)!", param_port_name_.c_str() );

		//Set port baudrate
		if( portHandler_->setBaudRate(param_port_buad_) ) {
			ROS_INFO( "Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate() );
			port_ok = true;
		} else {
			ROS_ERROR("Failed to change the baudrate!");
		}
	} else {
		ROS_ERROR( "Failed to open the port!" );
	}

	if( port_ok ) {
		ROS_INFO("Scanning for motors...");

		if( add_motors() ) {
			ROS_INFO("All motors successfully added!");

			timer_ = nh_.createTimer(ros::Duration(1.0 / param_update_rate_), &InterfaceDynamixel::callback_timer, this );

			ROS_INFO("Dynamixel interface started successfully!");
		} else {
			ROS_ERROR("Failed to contact motors!");
			shutdown_node();
		}
	} else {
		shutdown_node();
	}
}

InterfaceDynamixel::~InterfaceDynamixel() {
	shutdown_node();
}

void InterfaceDynamixel::shutdown_node( void ) {
	ROS_ERROR("Shutting down dynamixel interface");
	//TODO: writeTorque(false);
	portHandler_->closePort();
	ros::shutdown();
}

void InterfaceDynamixel::init_motor(std::string motor_model, uint8_t motor_id, double protocol_version) {
  dynamixel_tool::DynamixelTool dynamixel_motor(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dynamixel_motor);
}



bool InterfaceDynamixel::add_motors() {
	bool success = true;

	for(int i=0; i<param_num_motors_; i++) {
		std::string motor_model;
		int motor_id;
		double protocol_version;
		nh_.getParam("motors/motor_" + std::to_string(i) + "/model", motor_model);
		nh_.getParam("motors/motor_" + std::to_string(i) + "/id", motor_id);
		protocol_version = param_port_version_;

		init_motor(motor_model, (uint8_t)motor_id, protocol_version);

		int64_t tmp_val;

		if( readMotorState("torque_enable", i, &tmp_val) ) {
			ROS_INFO("Motor %i successfully added", i);
		} else {
			ROS_ERROR("Motor %i could not be read!", i);
			success = false;
		}
	}

	return success;
}

void InterfaceDynamixel::callback_timer(const ros::TimerEvent& e) {



	//TODO:
	//	Read in new states
	//	Send out current goals

	sensor_msgs::JointState joint_states;

	joint_states.header.stamp = e.current_real;
	joint_states.header.frame_id = param_frame_id_;;

	joint_states.position.clear();
	joint_states.velocity.clear();
	joint_states.effort.clear();

	for(int i=0; i<dynamixel_.size(); i++) {
		/*
		readMotorState("torque_enable");
		readMotorState("moving");
		readMotorState("goal_position");
		readMotorState("goal_velocity");
		readMotorState("goal_current");
		readMotorState("profile_velocity");
		readMotorState("profile_acceleration");
		readMotorState("present_position");
		readMotorState("present_velocity");
		readMotorState("present_current");
		readMotorState("min_position_limit");
		readMotorState("max_position_limit");
		*/

		int64_t reading_position = 0;
		readMotorState("present_position", i, &reading_position);

		joint_states.name.push_back("motor_" + std::to_string(i));
		joint_states.position.push_back(reading_position);
		joint_states.velocity.push_back(0.0);
		joint_states.effort.push_back(0.0);
	}

	pub_states_.publish(joint_states);
}

void InterfaceDynamixel::callback_setpoints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	//TODO: Some checks, expect a stream?
	joint_setpoints_ = *msg_in;
}


bool InterfaceDynamixel::readMotorState(std::string addr_name, int motor_number, int64_t *read_value) {
	dynamixel_[motor_number].item_ = dynamixel_[motor_number].ctrl_table_[addr_name];

	return( readDynamixelRegister(dynamixel_[motor_number].id_, dynamixel_[motor_number].item_->address, dynamixel_[motor_number].item_->data_length, read_value) );
}

bool InterfaceDynamixel::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value) {
	uint8_t dynamixel_error = 0;
	int8_t dynamixel_comm_result_;

	int8_t value_8_bit = 0;
	int16_t value_16_bit = 0;
	int32_t value_32_bit = 0;

	if (length == 1) {
		dynamixel_comm_result_ = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
	} else if (length == 2) {
		dynamixel_comm_result_ = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
	} else if (length == 4) {
		dynamixel_comm_result_ = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
	}

	if (dynamixel_comm_result_ == COMM_SUCCESS) {
		if (dynamixel_error != 0) {
			packetHandler_->printRxPacketError(dynamixel_error);
			return false;
		}

		if (length == 1) {
			*value = value_8_bit;
		} else if (length == 2) {
			*value = value_16_bit;
		} else if (length == 4) {
			*value = value_32_bit;
		}

		return true;
	} else {
		packetHandler_->printTxRxResult(dynamixel_comm_result_);
		ROS_ERROR("[ID] %u, Fail to read!", id);

		return false;
	}
}
