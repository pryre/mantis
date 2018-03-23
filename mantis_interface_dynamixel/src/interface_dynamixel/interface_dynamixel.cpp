#include <ros/ros.h>

#include <interface_dynamixel/interface_dynamixel.h>
#include <mantis_interface_dynamixel/EnableTorque.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>
#include <math.h>

const double position_to_value_ratio = 1.0 / ( 0.088 * M_PI / 180.0 );

InterfaceDynamixel::InterfaceDynamixel() :
	nh_("~"),
	motor_output_mode_(MOTOR_MODE_INVALID),
	param_update_rate_(50.0),
	topic_input_setpoints_("setpoints"),
	topic_output_states_("states"),
	param_port_name_("/dev/ttyUSB0"),
	param_port_buad_(57600),
	param_port_version_(0.0),
	param_num_motors_(0),
	//param_motor_model_(""),
	//param_motor_id_(0),
	//flag_setpoints_received_(false),
	param_frame_id_("robot") {


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

			initSyncRead();

			timer_ = nh_.createTimer(ros::Duration(1.0 / param_update_rate_), &InterfaceDynamixel::callback_timer, this );
			srv_enable_torque_ = nh_.advertiseService("enable_torque", &InterfaceDynamixel::enable_torque, this);

			ROS_INFO("Dynamixel interface started successfully!");

			//writeMotorState("id", 0, 1);
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

	for(int i=0; i<dynamixel_.ids.size(); i++)
		set_torque_enable(i, false);

	portHandler_->closePort();
	ros::shutdown();
}

bool InterfaceDynamixel::set_torque_enable(int motor_number, bool onoff) {
	return writeMotorState("torque_enable", motor_number, onoff);
}

bool InterfaceDynamixel::enable_torque(mantis_interface_dynamixel::EnableTorque::Request& req, mantis_interface_dynamixel::EnableTorque::Response& res) {
	bool success = true;

	if(req.set_enable.size() == dynamixel_.ids.size()) {
		for(int i=0; i<dynamixel_.ids.size(); i++) {
			if(req.set_enable[i]) {
				ROS_INFO("Turning on motor_%i!", i);
			} else {
				ROS_INFO("Turning off motor_%i!", i);
			}

			res.success &= set_torque_enable(i, req.set_enable[i]);
		}
	} else {
		ROS_ERROR("Torque enable vector must be same size as motors!");
		success = false;
	}

	res.success = success;

	return true;
}

void InterfaceDynamixel::init_motor(std::string motor_model, uint8_t motor_id, double protocol_version, std::string motor_name) {
	DynamixelTool dynamixel_motor;
	dynamixel_.tools.push_back(dynamixel_motor);
	dynamixel_.tools[dynamixel_.tools.size() - 1].addDXL(motor_model.c_str(), motor_id);
	dynamixel_.ids.push_back(motor_id);
	dynamixel_.names.push_back(motor_name);
}

bool InterfaceDynamixel::add_motors() {
	bool success = true;

	for(int i=0; i<param_num_motors_; i++) {
		std::string motor_name;
		std::string motor_model;
		int motor_id;
		double protocol_version;
		nh_.getParam("motors/motor_" + std::to_string(i) + "/name", motor_name);
		nh_.getParam("motors/motor_" + std::to_string(i) + "/model", motor_model);
		nh_.getParam("motors/motor_" + std::to_string(i) + "/id", motor_id);
		protocol_version = param_port_version_;

		init_motor(motor_model, (uint8_t)motor_id, protocol_version, motor_name);

		int64_t tmp_val;

		if( readMotorState("torque_enable", i, &tmp_val) ) {
			set_torque_enable(i, false);	//Make sure motor is not turned on
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


	//Allocated as MxN vector of motor states
	//	M is motor_id's
	//	N is [torque_enable, position, velocity, current]
	std::vector<std::vector<std::int32_t>> states;

	if( doSyncRead(&states) ) {
		joint_states.name = dynamixel_.names;

		for(int i=0; i<dynamixel_.ids.size(); i++) {
			//joint_states.name.push_back("motor_" + std::to_string(i));
			joint_states.position.push_back(convert_value_radian(states[i][1], i));
			joint_states.velocity.push_back(convert_value_velocity(states[i][2], i));
			joint_states.effort.push_back(convert_value_torque(states[i][3], i));

			/*
			if(states[i][0] && (motor_output_mode != MOTOR_MODE_INVALID)) {
				switch(motor_output_mode) {
					case MOTOR_MODE_TORQUE: {
						writeMotorState("goal_current", i, convert_torque_value(joint_setpoints_.effort[i], i));

						break;
					}
					case MOTOR_MODE_VELOCITY: {
						writeMotorState("goal_velocity", i, convert_velocity_value(joint_setpoints_.velocity[i], i));

						break;
					}
					case MOTOR_MODE_POSITION: {
						writeMotorState("goal_position", i, convert_radian_value(joint_setpoints_.position[i], i));

						break;
					}
					default: {
						ROS_ERROR("Setpoint has not been received yet, but motors are on!");
					}
				}
			}
			*/
		}

		pub_states_.publish(joint_states);
	}

	if(motor_output_mode_ != MOTOR_MODE_INVALID) {
		switch(motor_output_mode_) {
			case MOTOR_MODE_TORQUE: {
				doSyncWrite("goal_current");
				//writeMotorState("goal_current", i, convert_torque_value(joint_setpoints_.effort[i], i));

				break;
			}
			case MOTOR_MODE_VELOCITY: {
				doSyncWrite("goal_velocity");
				//writeMotorState("goal_velocity", i, convert_velocity_value(joint_setpoints_.velocity[i], i));

				break;
			}
			case MOTOR_MODE_POSITION: {
				doSyncWrite("goal_position");
				//writeMotorState("goal_position", i, convert_radian_value(joint_setpoints_.position[i], i));

				break;
			}
			default: {
				ROS_ERROR("Mode write error: unknown mode!");
			}
		}
	}
}

void InterfaceDynamixel::initSyncRead() {
	for(int i=0; i<dynamixel_.ids.size(); i++) {
		//dynamixel_[i].item_ = dynamixel_[i].ctrl_table_["indirect_address"];

		uint8_t id = dynamixel_.ids[i];
		//uint16_t addr = dynamixel_[i].item_->address;
		uint16_t addr = 168; //XXX: XM430-W350 indirect_address_1
		uint8_t length = 2;

		ControlTableItem* item;
		item = dynamixel_.tools[i].getControlItem("torque_enable");
		writeDynamixelRegister(id, addr + 0, length, item->address + 0);
		item = dynamixel_.tools[i].getControlItem("present_position");
		writeDynamixelRegister(id, addr + 2, length, item->address + 0);
		writeDynamixelRegister(id, addr + 4, length, item->address + 1);
		writeDynamixelRegister(id, addr + 6, length, item->address + 2);
		writeDynamixelRegister(id, addr + 8, length, item->address + 3);
		item = dynamixel_.tools[i].getControlItem("present_velocity");
		writeDynamixelRegister(id, addr + 10, length, item->address + 0);
		writeDynamixelRegister(id, addr + 12, length, item->address + 1);
		writeDynamixelRegister(id, addr + 14, length, item->address + 2);
		writeDynamixelRegister(id, addr + 16, length, item->address + 3);
		item = dynamixel_.tools[i].getControlItem("present_current");
		writeDynamixelRegister(id, addr + 18, length, item->address + 0);
		writeDynamixelRegister(id, addr + 20, length, item->address + 1);
	}
}

//TODO: This needs to be switched to a groupBulkRead so that multiple different models can be supported
bool InterfaceDynamixel::doSyncRead(std::vector<std::vector<std::int32_t>> *states) {
	bool success = true;
	//dynamixel_[0].item_ = dynamixel_[0].ctrl_table_["indirect_data"];
	//uint8_t read_addr = dynamixel_[0].item_->address;
	uint8_t read_addr = 224;	//XXX: XM430-W350 indirect_data_1
	dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, read_addr, 13); //XXX: 13 from length of data stored in indirect

	for(int j=0; j<dynamixel_.ids.size(); j++) {
		uint8_t dxl_addparam_result = groupSyncRead.addParam(dynamixel_.ids[j]);

		if(!dxl_addparam_result) {
			ROS_ERROR("ID:%d] groupSyncRead addparam failed", dynamixel_.ids[j]);
			success = false;
		}
	}

	if(success) {
		uint8_t comm_result = groupSyncRead.txRxPacket();

		if ( comm_result == COMM_SUCCESS ) {
			for(int i=0; i<dynamixel_.ids.size(); i++) {
				// Get Dynamixel present position value
				uint8_t id = dynamixel_.ids[i];

				ControlTableItem* item;
				item = dynamixel_.tools[i].getControlItem("torque_enable");
				uint8_t len_te = item->data_length;
				item = dynamixel_.tools[i].getControlItem("present_position");
				uint8_t len_pos = item->data_length;
				item = dynamixel_.tools[i].getControlItem("present_velocity");
				uint8_t len_vel = item->data_length;
				item = dynamixel_.tools[i].getControlItem("present_current");
				uint8_t len_cur = item->data_length;

				uint32_t offset = 0;
				int32_t torque_enable = groupSyncRead.getData(id, read_addr + offset, len_te);
				offset += len_te;
				int32_t present_position = groupSyncRead.getData(id, read_addr + offset, len_pos);
				offset += len_pos;
				int32_t present_velocity = groupSyncRead.getData(id, read_addr + offset, len_vel);
				offset += len_vel;
				int32_t present_current = (int16_t)groupSyncRead.getData(id, read_addr + offset, len_cur);

				ROS_DEBUG("SYNC_READ: [%d, %d, %d, %d]", torque_enable, present_position, present_velocity, present_current);

				std::vector<int32_t> vals;
				vals.push_back(torque_enable);
				vals.push_back(present_position);
				vals.push_back(present_velocity);
				vals.push_back(present_current);

				states->push_back( vals );
			}
		} else {
			ROS_ERROR("Dynamixel Tx/Rx result error: %i", comm_result);
			//packetHandler_->printTxRxResult(comm_result);
			success = false;
		}
	}

	return success;
}


void InterfaceDynamixel::doSyncWrite(std::string addr_name) {
	bool do_write = false;

	// Initialize GroupSyncWrite instance

	ControlTableItem* item;
	//XXX: Need to do this to hardcode this to get anything working
	item = dynamixel_.tools[0].getControlItem(addr_name.c_str());
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, item->address, item->data_length);

	if(addr_name == "goal_current") {
		for(int i=0; i<dynamixel_.ids.size(); i++) {
			item = dynamixel_.tools[i].getControlItem(addr_name.c_str());

			// Allocate goal position value into byte array
			uint8_t param_goal_current[2];
			uint16_t goal_current = convert_torque_value(joint_setpoints_.effort[i], i);

			param_goal_current[0] = DXL_LOBYTE(goal_current);
			param_goal_current[1] = DXL_HIBYTE(goal_current);

			// Add Dynamixel goal position value to the Syncwrite parameter storage
			uint8_t dxl_addparam_result = groupSyncWrite.addParam(dynamixel_.ids[i], param_goal_current);

			if(!dxl_addparam_result) {
				ROS_ERROR("ID:%d] groupSyncRead addparam failed", dynamixel_.ids[i]);
				//success = false;
			}
		}

		do_write = true;
	}
	/*
	if(addr_name == "goal_velocity") {
		for(int i=0; i<dynamixel_.size(); i++) {
			dynamixel_[i].item_ = dynamixel_[i].ctrl_table_[addr_name];

			// Allocate goal position value into byte array
			uint8_t param_goal_current[4];
			uint16_t goal_current = convert_torque_value(joint_setpoints_.effort[i], i);

			param_goal_current[0] = DXL_LOBYTE(goal_current);
			param_goal_current[1] = DXL_HIBYTE(goal_current);

			// Add Dynamixel goal position value to the Syncwrite parameter storage
			uint8_t dxl_addparam_result = groupSyncWrite.addParam(dynamixel_[i].id_, param_goal_current);

			if(!dxl_addparam_result) {
				ROS_ERROR("ID:%d] groupSyncRead addparam failed", dynamixel_[i].id_);
				//success = false;
			}
		}

		do_write = true;
	}
	*/
	if(addr_name == "goal_position") {
		for(int i=0; i<dynamixel_.ids.size(); i++) {
			item = dynamixel_.tools[i].getControlItem(addr_name.c_str());

			// Allocate goal position value into byte array
			uint8_t param_goal_position[4];
			int32_t goal_position = convert_radian_value(joint_setpoints_.position[i], i);

			param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
			param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
			param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
			param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

			// Add Dynamixel goal position value to the Syncwrite parameter storage
			uint8_t dxl_addparam_result = groupSyncWrite.addParam(dynamixel_.ids[i], param_goal_position);

			if(!dxl_addparam_result) {
				ROS_ERROR("ID:%d] groupSyncRead addparam failed", dynamixel_.ids[i]);
				//success = false;
			}
		}

		do_write = true;
	}

	if(do_write) {
		// Syncwrite goal position
		groupSyncWrite.txPacket();
		// Clear syncwrite parameter storage
		groupSyncWrite.clearParam();
	}
}

void InterfaceDynamixel::callback_setpoints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	//TODO: expect a stream?

	bool success = true;
	int num_motors = dynamixel_.ids.size();

	//Checks to make sure that at least one input is of the right size
	bool input_size_name_ok = msg_in->name.size() == num_motors;
	bool input_size_position_ok = msg_in->position.size() == num_motors;
	bool input_size_velocity_ok = msg_in->velocity.size() == num_motors;
	bool input_size_effort_ok = msg_in->effort.size() == num_motors;

	bool input_ok = input_size_name_ok && (input_size_position_ok || input_size_velocity_ok || input_size_effort_ok);

	std::string failure_reason;

	if( input_ok ) {
		joint_setpoints_ = *msg_in;

		for(int i=0; i<num_motors; i++) {
			//Use the lowest level mode that has been sent
			if(input_size_effort_ok) {
				if(motor_output_mode_ != MOTOR_MODE_TORQUE) {
					ROS_INFO("Torque control setpoint accepted");
					set_torque_enable(i, false);
					writeMotorState("operating_mode", i, MOTOR_MODE_TORQUE);
					motor_output_mode_ = MOTOR_MODE_TORQUE;
				}
			} else if( input_size_velocity_ok ) {
				if(motor_output_mode_ != MOTOR_MODE_VELOCITY) {
					ROS_INFO("Velocity control setpoint accepted");
					set_torque_enable(i, false);
					writeMotorState("operating_mode", i, MOTOR_MODE_VELOCITY);
					motor_output_mode_ = MOTOR_MODE_VELOCITY;
				}
			} else if( input_size_position_ok ) {
				if(motor_output_mode_ != MOTOR_MODE_POSITION) {
					ROS_INFO("Position control setpoint accepted");
					set_torque_enable(i, false);
					writeMotorState("operating_mode", i, MOTOR_MODE_POSITION);
					motor_output_mode_ = MOTOR_MODE_POSITION;
				}
			} else {
				failure_reason = "Something went wrong when selecting setpoint mode!";
				success = false;
			}
		}
	} else {
		if(!input_size_name_ok) {
			//TODO: We should probably actual match motor inputs with the specified names
			failure_reason = "Name vector length doesn't match motors avaliable";
		} else if ( (msg_in->effort.size() > 0) && !input_size_effort_ok) {
			failure_reason = "Effort vector length doesn't match motors avaliable";
		} else if ( (msg_in->velocity.size() > 0) && !input_size_velocity_ok) {
			failure_reason = "Velocity vector length doesn't match motors avaliable";
		} else if ( (msg_in->position.size() > 0) && !input_size_position_ok) {
			failure_reason = "Position vector length doesn't match motors avaliable";
		} else {
			failure_reason = "Unkown sepoint error!";
		}

		success = false;
	}

	if(!success) {
		ROS_WARN_THROTTLE(1.0, "Ignoring setpoint: %s", failure_reason.c_str());
	}
}

bool InterfaceDynamixel::readMotorState(std::string addr_name, int motor_number, int64_t *read_value) {
	ControlTableItem* item;
	item = dynamixel_.tools[motor_number].getControlItem(addr_name.c_str());

	return( readDynamixelRegister(dynamixel_.ids[motor_number], item->address, item->data_length, read_value) );
}

bool InterfaceDynamixel::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value) {
	uint8_t dynamixel_error = 0;
	int8_t comm_result;

	int8_t value_8_bit = 0;
	int16_t value_16_bit = 0;
	int32_t value_32_bit = 0;

	if (length == 1) {
		comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
	} else if (length == 2) {
		comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
	} else if (length == 4) {
		comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
	}

	if (comm_result == COMM_SUCCESS) {
		if (dynamixel_error != 0) {
			ROS_ERROR("Dynamixel Rx packet error: %i", dynamixel_error);
			//packetHandler_->printRxPacketError(dynamixel_error);
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
		ROS_ERROR("Dynamixel Tx/Rx result error: %i", comm_result);
		//packetHandler_->printTxRxResult(comm_result);
		ROS_ERROR("[ID] %u, Fail to read!", id);
	}

	return false;
}

bool InterfaceDynamixel::writeMotorState(std::string addr_name, int motor_number, uint32_t write_value) {
	ControlTableItem* item;
	item = dynamixel_.tools[motor_number].getControlItem(addr_name.c_str());

	return writeDynamixelRegister(dynamixel_.ids[motor_number], item->address, item->data_length, write_value);
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
			ROS_ERROR("Dynamixel Rx packet error: %i", dynamixel_error);
			//packetHandler_->printRxPacketError(dynamixel_error);
		} else {
			return true;
		}
	} else {
		ROS_ERROR("Dynamixel Tx/Rx result error: %i", comm_result);
	}

	ROS_ERROR("[ID] %u, Fail to write!", id);

	return false;
}

int InterfaceDynamixel::convert_torque_value(double torque, int motor_number) {
	return (int)(torque * dynamixel_.tools[motor_number].getTorqueToCurrentValueRatio());
}

double InterfaceDynamixel::convert_value_torque(int value, int motor_number) {
	return (double)value / dynamixel_.tools[motor_number].getTorqueToCurrentValueRatio();
}


int InterfaceDynamixel::convert_velocity_value(double velocity, int motor_number) {
	return (int)(velocity * 2 * dynamixel_.tools[motor_number].getVelocityToValueRatio());
}

double InterfaceDynamixel::convert_value_velocity(int value, int motor_number) {
	return ( (double)value / 2.0 ) / dynamixel_.tools[motor_number].getVelocityToValueRatio();
}

int InterfaceDynamixel::convert_radian_value(double radian, int motor_number) {
	int64_t value = 0;
	int64_t rad_max = dynamixel_.tools[motor_number].getMaxRadian();
	int64_t rad_min = dynamixel_.tools[motor_number].getMinRadian();
	int64_t val_max = dynamixel_.tools[motor_number].getValueOfMaxRadianPosition();
	int64_t val_min = dynamixel_.tools[motor_number].getValueOfMinRadianPosition();
	int64_t val_zero = dynamixel_.tools[motor_number].getValueOfZeroRadianPosition();

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

double InterfaceDynamixel::convert_value_radian(int value, int motor_number) {
	double radian = (double) value / position_to_value_ratio;

	//Constrain angle from -M_PI to +M_PI
    radian = fmod(radian, 2*M_PI);

    if (radian < 0)
        radian += 2 * M_PI;

	return radian - M_PI;
}

/*
int32_t InterfaceDynamixel::convert_radian_value(double radian, int motor_number) {
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

double InterfaceDynamixel::convert_value_radian(int32_t value, int motor_number) {
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

	//radian = (radian > rad_max) ? rad_max : (radian < rad_min) ? rad_min : radian;

	return radian;
}
*/

/*
bool InterfaceDynamixel::bulk_read_states(std::vector<std::string> *states, std::vector<std::vector<int32_t>> *result) {
	bool success = true;

	dynamixel::GroupBulkRead bulk_read(portHandler_, packetHandler_);

	for(int j=0; j<states->size(); j++) {
		for(int i=0; i<dynamixel_.size(); i++) {
			dynamixel_[i].item_ = dynamixel_[i].ctrl_table_[(*states)[j]];

			uint8_t dxl_addparam_result = bulk_read.addParam(dynamixel_[i].id_,
															 dynamixel_[i].item_->address,
															 dynamixel_[i].item_->data_length);
			if (!dxl_addparam_result) {
				ROS_ERROR("[ID:%i] grouBulkRead addparam failed", dynamixel_[i].id_);
				success = false;
			}
		}
	}

	if(success) {
		int8_t dynamixel_comm_result = bulk_read.txRxPacket();

		if (dynamixel_comm_result == COMM_SUCCESS) {
			for(int a=0; a<states->size(); a++) {
				std::vector<int32_t> state_result_set;

				for(int b=0; b<dynamixel_.size(); b++) {
					dynamixel_[b].item_ = dynamixel_[b].ctrl_table_[(*states)[a]];

					if( !bulk_read.isAvailable(dynamixel_[b].id_, dynamixel_[b].item_->address, dynamixel_[b].item_->data_length) ) {
						ROS_ERROR("[ID:%d] groupBulkRead getdata failed", dynamixel_[b].id_);
						success = false;
					} else {
						uint32_t data = bulk_read.getData(dynamixel_[b].id_,
														  dynamixel_[b].item_->address,
														  dynamixel_[b].item_->data_length);

						ROS_INFO("RAW_DATA(%s, %i): %i", (*states)[a].c_str(), dynamixel_[b].item_->data_length, data);

						int32_t value = 0;
						int8_t value_8_bit = 0;
						int16_t value_16_bit = 0;
						int32_t value_32_bit = 0;

						if (dynamixel_[b].item_->data_length == 1) {
							*((uint8_t*)&value_8_bit) = data;
							value = value_8_bit;
						} else if (dynamixel_[b].item_->data_length == 2) {
							*((uint16_t*)&value_16_bit) = data;
							value = value_16_bit;
						} else if (dynamixel_[b].item_->data_length == 4) {
							*((uint32_t*)&value_32_bit) = data;
							value = value_32_bit;
						}

						state_result_set.push_back(value);
					}
				}

				result->push_back(state_result_set);
			}
		} else {
			packetHandler_->printTxRxResult(dynamixel_comm_result);
			ROS_ERROR("Fail to perform bulk read!");
			success = false;
		}
	}

	return success;



}
*/
