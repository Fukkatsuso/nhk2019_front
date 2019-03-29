/*
 * CANSender.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#include "CANSender.h"


CANSender::CANSender(CAN *can) : CANProtocol(can)
{}


void CANSender::send_direction(unsigned int can_id, float direction)
{
	data.direction.value = direction;
	send(can_id, data.direction.byte, BYTE_DIRECTION);
}


void CANSender::send_speed(unsigned int can_id, float speed)
{
	data.speed.value = speed;
	send(can_id, data.speed.byte, BYTE_SPEED);
}

void CANSender::send_velocity_vector(unsigned int can_id, float direction, float speed)
{
	data.velocity_vector.direction.value = direction;
	data.velocity_vector.speed.value = speed;
	data.direction.value = direction;
	data.speed.value = speed;
	send(can_id, data.velocity_vector.byte, BYTE_VELOCITYVECTOR);
}


void CANSender::send_timer_reset(unsigned int can_id)
{
	data.timer_reset.value = 1;
	send(can_id, data.timer_reset.byte, BYTE_TIMERRESET);
}


void CANSender::send_area(unsigned int can_id, unsigned char area)
{
	data.area.value = area;
	send(can_id, data.area.byte, BYTE_AREA);
}


void CANSender::send_area_change(unsigned int can_id, unsigned char area)
{
	data.area_change.value = area;
	send(can_id, data.area_change.byte, BYTE_AREA);
}


void CANSender::send_leg_up(unsigned int can_id, unsigned char leg_up)
{
	data.leg_up.byte[0] = leg_up;
	send(can_id, data.leg_up.byte, BYTE_LEGUP);
}


void CANSender::send_move_dist(unsigned int can_id, float dist)
{
	unsigned int type = can_id & 0x00f;
	switch(type){
	case CANID::MoveDistFront:
		data.move_dist_front.value = (signed short)dist;
		send(can_id, data.move_dist_front.byte, BYTE_DIST);
		break;
	case CANID::MoveDistRear:
		data.move_dist_rear.value = (signed short)dist;
		send(can_id, data.move_dist_rear.byte, BYTE_DIST);
		break;
	default:
		break;
	}
}


void CANSender::send(unsigned int can_id, unsigned char *data, unsigned short byte)
{
	CANMessage msg(can_id, (const char*)data, byte);
	if(can->write(msg));
	else if(can->write(msg));
	else (can->write(msg));
}
