#include <ros/ros.h>

#include <interface_dynamixel/interface_dynamixel.h>
#include <mantis_interface_dynamixel/EnableTorque.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>
#include <math.h>

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

			ROS_INFO("Initializing SyncRead...");
			initSyncRead();
			ROS_INFO("SyncRead done!");

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

		ROS_INFO("Initializing motor #%i", i);
		init_motor(motor_model, (uint8_t)motor_id, protocol_version, motor_name);

		ROS_INFO("Contacting motor #%i", i);
		int64_t tmp_val;

		if( readMotorState("Torque_Enable", i, &tmp_val) ) {
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


