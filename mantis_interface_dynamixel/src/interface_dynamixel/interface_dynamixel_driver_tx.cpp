#include <interface_dynamixel/interface_dynamixel.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

bool InterfaceDynamixel::writeMotorState(dynamixel_control_items_t item_id, int motor_number, uint32_t write_value) {
	return writeDynamixelRegister(get_id(motor_number), dynamixel_control_items_[item_id].address, dynamixel_control_items_[item_id].data_length, write_value);
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
			ROS_ERROR("Dynamixel Rx packet error: %s", packetHandler_->getRxPacketError(dynamixel_error));
		} else {
			return true;
		}
	} else {
		ROS_ERROR("Dynamixel Tx/Rx result error: %s", packetHandler_->getTxRxResult(comm_result));
	}

	ROS_ERROR("[ID: %u] Fail to write!", id);

	return false;
}

void InterfaceDynamixel::doSyncWrite(dynamixel_control_items_t item_id, std::vector<double>* ref) {
	bool do_write = false;

	// Initialize GroupSyncWrite instance
	ControlTableItem item = dynamixel_control_items_[item_id];
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, item.address, item.data_length);

	if(item_id == DCI_GOAL_CURRENT) {
		for(int i=0; i<dxl_.size(); i++) {
			// Allocate goal position value into byte array
			uint8_t param_goal_current[2];
			uint16_t goal_current = convert_torque_value(ref->at(i), i);

			param_goal_current[0] = DXL_LOBYTE(goal_current);
			param_goal_current[1] = DXL_HIBYTE(goal_current);

			// Add Dynamixel goal position value to the Syncwrite parameter storage
			uint8_t dxl_addparam_result = groupSyncWrite.addParam(get_id(i), param_goal_current);

			if(!dxl_addparam_result) {
				ROS_ERROR("[ID: %u] groupSyncRead addparam failed", get_id(i));
				//success = false;
			}
		}

		do_write = true;
	} else if(item_id == DCI_GOAL_VELOCITY) {
		for(int i=0; i<dxl_.size(); i++) {
			// Allocate goal position value into byte array
			uint8_t param_goal_velocity[4];
			int32_t goal_velocity = convert_velocity_value(ref->at(i), i);

			param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(goal_velocity));
			param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(goal_velocity));
			param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(goal_velocity));
			param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(goal_velocity));

			// Add Dynamixel goal position value to the Syncwrite parameter storage
			uint8_t dxl_addparam_result = groupSyncWrite.addParam(get_id(i), param_goal_velocity);

			if(!dxl_addparam_result) {
				ROS_ERROR("[ID: %u] groupSyncRead addparam failed", get_id(i));
				//success = false;
			}
		}

		do_write = true;
	}
	else if(item_id == DCI_GOAL_POSITION) {
		for(int i=0; i<dxl_.size(); i++) {
			// Allocate goal position value into byte array
			uint8_t param_goal_position[4];
			int32_t goal_position = convert_radian_value(ref->at(i), i);

			param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
			param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
			param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
			param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

			// Add Dynamixel goal position value to the Syncwrite parameter storage
			uint8_t dxl_addparam_result = groupSyncWrite.addParam(get_id(i), param_goal_position);

			if(!dxl_addparam_result) {
				ROS_ERROR("[ID: %u] groupSyncRead addparam failed", get_id(i));
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

	if(do_write) {
		// Syncwrite goal position
		groupSyncWrite.txPacket();
		// Clear syncwrite parameter storage
		groupSyncWrite.clearParam();
	}
}
