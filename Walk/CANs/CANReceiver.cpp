/*
 * CANReceiver.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#include "CANReceiver.h"


CANReceiver::CANReceiver(CAN *can) : CANProtocol(can)
{
	reset();
}


/*
 * example:
 * CANReceiver canrcv(&can);
 * void can_receive(){
 * 		if(can.read(rcvMsg)){
 * 			if((rcvMsg.id&0xff0)==(CANID::From::Master | CANID::To::SlaveAll)){
 * 				canrcv.recerive(rcvMsg.id, rcvMsg.data);
 * 			}
 * 		}
 * }
 */
void CANReceiver::receive(CANMessage msg)
{
	unsigned int type = msg.id & 0x00f;

	switch(type){
	case CANID::Direction:
		copy_data(msg, data.direction.byte, BYTE_DIRECTION);
		break;
	case CANID::Speed:
		copy_data(msg, data.speed.byte, BYTE_SPEED);
		break;
	case CANID::VelocityVector:
		copy_data(msg, data.velocity_vector.byte, BYTE_VELOCITYVECTOR);
		data.direction.value = data.velocity_vector.direction.value;
		data.speed.value = data.velocity_vector.speed.value;
		break;
	case CANID::Area:
		copy_data(msg, data.area.byte, BYTE_AREA);
		break;
	case CANID::AreaChange:
		copy_data(msg, data.area_change.byte, BYTE_AREA);
		break;
	case CANID::LegUp:
		copy_data(msg, data.leg_up.byte, BYTE_LEGUP);
		break;
	case CANID::MovePositionFront:
		copy_data(msg, data.move_position_front.byte, BYTE_MOVEPOSITION);
		break;
	case CANID::MovePositionRear:
		copy_data(msg, data.move_position_rear.byte, BYTE_MOVEPOSITION);
		break;
	default:
		break;
	}
}


void CANReceiver::copy_data(CANMessage msg, unsigned char data[], short byte)
{
	for(int i=0; i<byte; i++){
		data[i] = msg.data[i];
	}
}
