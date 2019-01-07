#include <dynamixel_sdk/dynamixel_sdk.h>
#include <interface_dynamixel/interface_dynamixel.h>

bool InterfaceDynamixel::readMotorState( dynamixel_control_items_t item_id,
	int motor_number, int64_t* read_value ) {

	return readDynamixelRegister(
		get_id( motor_number ), dynamixel_control_items_[item_id].address,
		dynamixel_control_items_[item_id].data_length, read_value );
}

bool InterfaceDynamixel::readDynamixelRegister( uint8_t id, uint16_t addr,
	uint8_t length, int64_t* value ) {
	uint8_t dynamixel_error = 0;
	int8_t comm_result;

	int8_t value_8_bit = 0;
	int16_t value_16_bit = 0;
	int32_t value_32_bit = 0;

	if ( length == 1 ) {
		comm_result = packetHandler_->read1ByteTxRx(
			portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error );
	} else if ( length == 2 ) {
		comm_result = packetHandler_->read2ByteTxRx(
			portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error );
	} else if ( length == 4 ) {
		comm_result = packetHandler_->read4ByteTxRx(
			portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error );
	}

	if ( comm_result == COMM_SUCCESS ) {
		if ( dynamixel_error != 0 ) {
			ROS_ERROR( "Dynamixel Rx packet error: %s",
				packetHandler_->getRxPacketError( dynamixel_error ) );
		} else {
			if ( length == 1 ) {
				*value = value_8_bit;
			} else if ( length == 2 ) {
				*value = value_16_bit;
			} else if ( length == 4 ) {
				*value = value_32_bit;
			}

			return true;
		}
	} else {
		ROS_ERROR( "Dynamixel Tx/Rx result error: %s",
			packetHandler_->getTxRxResult( comm_result ) );

		ROS_ERROR( "[ID: %u] Fail to read!", id );
	}

	return false;
}

void InterfaceDynamixel::initSyncRead() {
	for ( int i = 0; i < dxl_.size(); i++ ) {
		// dynamixel_[i].item_ = dynamixel_[i].ctrl_table_["indirect_address"];

		uint8_t id = get_id( i );
		// uint16_t addr = dynamixel_[i].item_->address;
		uint16_t addr = 168; // XXX: XM430-W350 indirect_address_1
		uint8_t length = 2;

		ControlTableItem item;
		item = dynamixel_control_items_[DCI_TORQUE_ENABLE];
		writeDynamixelRegister( id, addr + 0, length, item.address + 0 );
		item = dynamixel_control_items_[DCI_PRESENT_POSITION];
		writeDynamixelRegister( id, addr + 2, length, item.address + 0 );
		writeDynamixelRegister( id, addr + 4, length, item.address + 1 );
		writeDynamixelRegister( id, addr + 6, length, item.address + 2 );
		writeDynamixelRegister( id, addr + 8, length, item.address + 3 );
		item = dynamixel_control_items_[DCI_PRESENT_VELOCITY];
		writeDynamixelRegister( id, addr + 10, length, item.address + 0 );
		writeDynamixelRegister( id, addr + 12, length, item.address + 1 );
		writeDynamixelRegister( id, addr + 14, length, item.address + 2 );
		writeDynamixelRegister( id, addr + 16, length, item.address + 3 );
		item = dynamixel_control_items_[DCI_PRESENT_CURRENT];
		writeDynamixelRegister( id, addr + 18, length, item.address + 0 );
		writeDynamixelRegister( id, addr + 20, length, item.address + 1 );
	}
}

// TODO: This needs to be switched to a groupBulkRead so that multiple different
// models can be supported
bool InterfaceDynamixel::doSyncRead(
	std::vector<std::vector<std::int32_t>>* states ) {
	bool success = true;
	// dynamixel_[0].item_ = dynamixel_[0].ctrl_table_["indirect_data"];
	// uint8_t read_addr = dynamixel_[0].item_->address;
	uint8_t read_addr = 224; // XXX: XM430-W350 indirect_data_1
	dynamixel::GroupSyncRead groupSyncRead(
		portHandler_, packetHandler_, read_addr,
		13 ); // XXX: 13 from length of data stored in indirect

	for ( int j = 0; j < dxl_.size(); j++ ) {
		uint8_t dxl_addparam_result = groupSyncRead.addParam( get_id( j ) );

		if ( !dxl_addparam_result ) {
			ROS_ERROR( "[ID: %u] groupSyncRead addparam failed", get_id( j ) );
			success = false;
		}
	}

	if ( success ) {
		uint8_t comm_result = groupSyncRead.txRxPacket();

		if ( comm_result == COMM_SUCCESS ) {
			for ( int i = 0; i < dxl_.size(); i++ ) {
				// Get Dynamixel present position value
				uint8_t id = get_id( i );

				ControlTableItem item;
				item = dynamixel_control_items_[DCI_PRESENT_POSITION];
				item = dynamixel_control_items_[DCI_TORQUE_ENABLE];
				uint8_t len_te = item.data_length;
				item = dynamixel_control_items_[DCI_PRESENT_POSITION];
				uint8_t len_pos = item.data_length;
				item = dynamixel_control_items_[DCI_PRESENT_VELOCITY];
				uint8_t len_vel = item.data_length;
				item = dynamixel_control_items_[DCI_PRESENT_CURRENT];
				uint8_t len_cur = item.data_length;

				uint32_t offset = 0;
				int32_t torque_enable = groupSyncRead.getData( id, read_addr + offset, len_te );
				offset += len_te;
				int32_t present_position = groupSyncRead.getData( id, read_addr + offset, len_pos );
				offset += len_pos;
				int32_t present_velocity = groupSyncRead.getData( id, read_addr + offset, len_vel );
				offset += len_vel;
				int32_t present_current = (int16_t)groupSyncRead.getData( id, read_addr + offset, len_cur );

				ROS_DEBUG( "SYNC_READ: [%d, %d, %d, %d]", torque_enable,
					present_position, present_velocity, present_current );

				std::vector<int32_t> vals;
				vals.push_back( torque_enable );
				vals.push_back( present_position );
				vals.push_back( present_velocity );
				vals.push_back( present_current );

				states->push_back( vals );
			}
		} else {
			ROS_ERROR( "Dynamixel Tx/Rx result error: %s",
				packetHandler_->getTxRxResult( comm_result ) );
			success = false;
		}
	}

	return success;
}
