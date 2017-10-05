#include <ros/ros.h>

#include <interface_dynamixel/interface_dynamixel.h>
#include <mantis_interface_dynamixel/EnableTorque.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <sensor_msgs/JointState.h>

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
			srv_enable_torque_ = nh_.advertiseService("enable_torque", &InterfaceDynamixel::enable_torque, this);

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

	for(int i=0; i<dynamixel_.size(); i++)
		set_torque(i, false);

	portHandler_->closePort();
	ros::shutdown();
}

bool InterfaceDynamixel::set_torque(int motor_number, bool onoff) {
	return writeMotorState("torque_enable", motor_number, onoff);
}

bool InterfaceDynamixel::enable_torque(mantis_interface_dynamixel::EnableTorque::Request& req, mantis_interface_dynamixel::EnableTorque::Response& res) {
	bool success = true;

	if(req.set_enable.size() == dynamixel_.size())
		for(int i=0; i<dynamixel_.size(); i++)
			res.success &= set_torque(i, req.set_enable[i]);

	return true;
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
		int64_t reading_velocity = 0;
		int64_t reading_current = 0;

		readMotorState("present_position", i, &reading_position);
		readMotorState("present_velocity", i, &reading_velocity);
		readMotorState("present_current", i, &reading_current);

		joint_states.name.push_back("motor_" + std::to_string(i));
		joint_states.position.push_back(reading_position);
		joint_states.velocity.push_back(reading_velocity);
		joint_states.effort.push_back(reading_current);
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
		} else {
			if (length == 1) {
				*value = value_8_bit;
			} else if (length == 2) {
				*value = value_16_bit;
			} else if (length == 4) {
				*value = value_32_bit;
			}

			return true;
		}
	} else {
		packetHandler_->printTxRxResult(dynamixel_comm_result_);
		ROS_ERROR("[ID] %u, Fail to read!", id);
	}

	return false;
}

bool InterfaceDynamixel::writeMotorState(std::string addr_name, int motor_number, uint32_t write_value) {
	dynamixel_[motor_number].item_ = dynamixel_[motor_number].ctrl_table_[addr_name];

	return( writeDynamixelRegister(dynamixel_[motor_number].id_, dynamixel_[motor_number].item_->address, dynamixel_[motor_number].item_->data_length, write_value) );
}

bool InterfaceDynamixel::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t value) {
	uint8_t dynamixel_error = 0;
	int comm_result = COMM_TX_FAIL;

	if (length == 1) {
		comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (uint8_t)value, &dynamixel_error);
	} else if (length == 2) {
		comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (uint16_t)value, &dynamixel_error);
	} else if (length == 4) {
		comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (uint32_t)value, &dynamixel_error);
	}

	if (comm_result == COMM_SUCCESS) {
		if (dynamixel_error != 0) {
			packetHandler_->printRxPacketError(dynamixel_error);
		} else {
			return true;
		}
	} else {
		packetHandler_->printTxRxResult(comm_result);
		ROS_ERROR("[ID] %u, Fail to write!", id);
	}

	return false;
}


int16_t InterfaceDynamixel::convert_torque_value(double torque, int motor_number) {
	return (int16_t)(torque * dynamixel_[motor_number].torque_to_current_value_ratio_);
}

double InterfaceDynamixel::convert_value_torque(int16_t value, int motor_number) {
	return (double)value / dynamixel_[motor_number].torque_to_current_value_ratio_;
}

uint32_t InterfaceDynamixel::convert_radian_value(double radian, int motor_number) {
	int64_t value = 0;
	int64_t rad_max = dynamixel_[motor_number].max_radian_;
	int64_t rad_min = dynamixel_[motor_number].min_radian_;
	int64_t val_max = dynamixel_[motor_number].value_of_max_radian_position_;
	int64_t val_min = dynamixel_[motor_number].value_of_min_radian_position_;
	int64_t val_zero = dynamixel_[motor_number].value_of_0_radian_position_;

	if (radian > 0.0) {
		value = (radian * (val_max - val_zero) / rad_max) + val_zero;
	} else if (radian < 0.0) {
		value = (radian * (val_min - val_zero) / rad_min) + val_zero;
	} else {
		value = val_zero;
	}

	value = (value > val_max) ? val_max : (value < val_min) ? val_min : value;

	return value;
}

double InterfaceDynamixel::convert_value_radian(uint32_t value, int motor_number) {
	double radian = 0.0;
	int64_t rad_max = dynamixel_[motor_number].max_radian_;
	int64_t rad_min = dynamixel_[motor_number].min_radian_;
	int64_t val_max = dynamixel_[motor_number].value_of_max_radian_position_;
	int64_t val_min = dynamixel_[motor_number].value_of_min_radian_position_;
	int64_t val_zero = dynamixel_[motor_number].value_of_0_radian_position_;

	if (value > val_zero) {
		radian = (double) (value - val_zero) * rad_max / (double) (val_max - val_zero);
	} else if (value < val_zero) {
		radian = (double) (value - val_zero) * rad_min / (double) (val_min - val_zero);
	}

	radian = (radian > rad_max) ? rad_max : (radian < rad_min) ? rad_min : radian;

	return radian;
}
