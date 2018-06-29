#include <ros/ros.h>

#include <interface_dynamixel/interface_dynamixel.h>
#include <mantis_interface_dynamixel/EnableTorque.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <sensor_msgs/JointState.h>

#include <vector>
#include <string>
#include <math.h>

static const std::string dynamixel_control_items_names[DCI_NUM_ITEMS] {
	"Torque_Enable",
	"Operating_Mode",
	"Profile_Velocity",
	"Profile_Acceleration",
	"Present_Position",
	"Present_Velocity",
	"Present_Current",
	"Goal_Position",
	"Goal_Velocity",
	"Goal_Current"
};

InterfaceDynamixel::InterfaceDynamixel() :
	nh_("~"),
	nhp_("~"),
	motor_output_mode_(MOTOR_MODE_INVALID),
	param_update_rate_(50.0),
	param_port_name_("/dev/ttyUSB0"),
	param_port_buad_(57600),
	param_port_version_(0.0),
	param_num_motors_(0),
	param_frame_id_("robot") {


	//ROS Setup
	nhp_.param("port_name", param_port_name_, param_port_name_);
	nhp_.param("port_baud", param_port_buad_, param_port_buad_);
	nhp_.param("protocol", param_port_version_, param_port_version_);
	nhp_.param("num_motors", param_num_motors_, param_num_motors_);
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("update_rate", param_update_rate_, param_update_rate_);

	sub_setpoints_ = nh_.subscribe<sensor_msgs::JointState>( "joint_setpoints", 10, &InterfaceDynamixel::callback_setpoints, this );
	pub_states_ = nh_.advertise<sensor_msgs::JointState>( "joint_states", 10);


	//Dynamixel Setup
	portHandler_ = dynamixel::PortHandler::getPortHandler( param_port_name_.c_str() );
	packetHandler_ = dynamixel::PacketHandler::getPacketHandler( param_port_version_ );

	bool port_ok = false;

	/*
	ROS_WARN("START TOOL TEST");
	DynamixelTool tool;
	tool.addTool("XM430-W350", 3);
	ROS_INFO("id: %d, mdl: %d, cnt: %d", tool.dxl_info_[tool.dxl_info_cnt_-1].id, tool.dxl_info_[tool.dxl_info_cnt_-1].model_num, tool.dxl_info_cnt_);
	ControlTableItem* item = tool.getControlItem("Torque_Enable");
	ROS_INFO("addr: %d, len: %d", item->address, item->data_length);
	ROS_WARN("END TOOL TEST");

	shutdown_node();
	*/

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

			timer_ = nhp_.createTimer(ros::Duration(1.0 / param_update_rate_), &InterfaceDynamixel::callback_timer, this );
			srv_enable_torque_specific_ = nhp_.advertiseService("enable_torque_specific", &InterfaceDynamixel::enable_torque_specific, this);
			srv_enable_torque_all_ = nhp_.advertiseService("enable_torque_all", &InterfaceDynamixel::enable_torque_all, this);

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


uint8_t InterfaceDynamixel::get_id( int motor_number ) {
	return dxl_[motor_number].dxl_info_[dxl_[motor_number].dxl_info_cnt_-1].id;
}

void InterfaceDynamixel::shutdown_node( void ) {
	ROS_ERROR("Shutting down dynamixel interface");

	for(int i=0; i<dxl_.size(); i++)
		set_torque_enable(i, false);

	portHandler_->closePort();
	ros::shutdown();
}

bool InterfaceDynamixel::set_torque_enable(int motor_number, bool onoff) {
	return writeMotorState(DCI_TORQUE_ENABLE, motor_number, onoff);
}

bool InterfaceDynamixel::enable_torque_specific(mantis_interface_dynamixel::EnableTorque::Request& req, mantis_interface_dynamixel::EnableTorque::Response& res) {
	bool success = true;

	if(req.set_enable.size() == dxl_.size()) {
		for(int i=0; i<dxl_.size(); i++) {
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

bool InterfaceDynamixel::enable_torque_all(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
	res.success = true;

	for(int i=0; i<dxl_.size(); i++) {
		if(req.data) {
			ROS_INFO("Turning on motor_%i!", i);
		} else {
			ROS_INFO("Turning off motor_%i!", i);
		}

		res.success &= set_torque_enable(i, req.data);
	}

	if(req.data) {
		res.message = "Turning on all motors";
	} else {
		res.message = "Turning off all motors";
	}

	return true;
}

void InterfaceDynamixel::init_motor(std::string motor_model, uint8_t motor_id, double protocol_version, std::string motor_name) {
	DynamixelTool tool;
	dxl_.push_back(tool);
	dxl_.back().addTool(motor_model.c_str(), motor_id);
	dxl_names_.push_back(motor_name);
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

		//XXX: This is a pretty bad hack for this
		if(i==0){
			for(int j=0; j<DCI_NUM_ITEMS; j++) {
				dynamixel_control_items_.push_back(*dxl_[0].getControlItem(dynamixel_control_items_names[j].c_str()));
				ROS_INFO("Loaded ControlTableItem \"%s\": %i; %i", dynamixel_control_items_names[j].c_str(), dynamixel_control_items_[j].address, dynamixel_control_items_[j].data_length);
			}
		}
		//XXX: This is a pretty bad hack for this

		ROS_INFO("Contacting motor #%i", i);
		int64_t tmp_val;

		//if( readMotorState("Torque_Enable", i, &tmp_val) ) {
		if( readMotorState(DCI_TORQUE_ENABLE, i, &tmp_val) ) {
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
		joint_states.name = dxl_names_;

		for(int i=0; i<dxl_.size(); i++) {
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
				doSyncWrite(DCI_GOAL_CURRENT);
				//writeMotorState("goal_current", i, convert_torque_value(joint_setpoints_.effort[i], i));

				break;
			}
			case MOTOR_MODE_VELOCITY: {
				doSyncWrite(DCI_GOAL_VELOCITY);
				//writeMotorState("goal_velocity", i, convert_velocity_value(joint_setpoints_.velocity[i], i));

				break;
			}
			case MOTOR_MODE_POSITION: {
				doSyncWrite(DCI_GOAL_POSITION);
				//writeMotorState("goal_position", i, convert_radian_value(joint_setpoints_.position[i], i));

				break;
			}
			default: {
				ROS_ERROR("Mode write error: unknown mode!");
			}
		}
	}
}

void InterfaceDynamixel::callback_setpoints(const sensor_msgs::JointState::ConstPtr& msg_in) {
	//TODO: expect a stream?

	bool success = true;
	int num_motors = dxl_.size();

	//Checks to make sure that at least one input is of the right size
	bool input_size_name_ok = msg_in->name.size() == num_motors;
	bool input_size_position_ok = msg_in->position.size() == num_motors;
	bool input_size_velocity_ok = msg_in->velocity.size() == num_motors;
	bool input_size_effort_ok = msg_in->effort.size() == num_motors;

	bool input_ok = input_size_name_ok && (input_size_position_ok || input_size_velocity_ok || input_size_effort_ok);

	std::string failure_reason;

	if( input_ok ) {
		joint_setpoints_ = *msg_in;

		//Use the lowest level mode that has been sent
		if(input_size_effort_ok) {
			if(motor_output_mode_ != MOTOR_MODE_TORQUE) {
				ROS_INFO("Torque control setpoint accepted");
				for(int i=0; i<num_motors; i++) {
					set_torque_enable(i, false);
					writeMotorState(DCI_OPERATING_MODE, i, MOTOR_MODE_TORQUE);
					motor_output_mode_ = MOTOR_MODE_TORQUE;
				}
			}
		} else if( input_size_velocity_ok ) {
			if(motor_output_mode_ != MOTOR_MODE_VELOCITY) {
				ROS_INFO("Velocity control setpoint accepted");
				for(int i=0; i<num_motors; i++) {
					set_torque_enable(i, false);
					writeMotorState(DCI_OPERATING_MODE, i, MOTOR_MODE_VELOCITY);
					motor_output_mode_ = MOTOR_MODE_VELOCITY;
				}
			}
		} else if( input_size_position_ok ) {
			if(motor_output_mode_ != MOTOR_MODE_POSITION) {
				ROS_INFO("Position control setpoint accepted");
				for(int i=0; i<num_motors; i++) {
					set_torque_enable(i, false);
					writeMotorState(DCI_OPERATING_MODE, i, MOTOR_MODE_POSITION);
					motor_output_mode_ = MOTOR_MODE_POSITION;

					ROS_WARN("--- HACK TO SET MOVEMENT PROFILE ---");
					writeMotorState(DCI_PROFILE_ACCELERATION, i, 50);
					writeMotorState(DCI_PROFILE_VELOCITY, i, 50);
				}
			}
		} else {
			failure_reason = "Something went wrong when selecting setpoint mode!";
			success = false;
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

