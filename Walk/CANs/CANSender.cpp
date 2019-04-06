/*
 * CANSender.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#include "CANSender.h"


CANSender::CANSender(CAN *can) : CANProtocol(can)
{}


void CANSender::send_walk_command(unsigned int can_id,
		unsigned char area, short speed, short direction, short pitch, unsigned char leg_up)
{
	union can_WalkCommand cmd = {};
	cmd.area = area;
	cmd.speed = speed;
	cmd.direction = direction;
	cmd.pitch = pitch;
	cmd.leg_up.byte[0] = leg_up;
	send(can_id, cmd.byte, BYTE_WALK_COMMAND);
}


void CANSender::send_timer_reset(unsigned int can_id)
{
	union can_TimerReset timer_reset = {};
	timer_reset.value = 1;
	send(can_id, timer_reset.byte, BYTE_TIMER_RESET);
}


void CANSender::send_leg_info(unsigned int can_id, short dist, unsigned char state, unsigned char kouden)
{
	union can_LegInfo leg_info = {};
	leg_info.dist = dist;
	leg_info.state.byte[0] = state;
	leg_info.kouden.byte[0] = kouden;
	send(can_id, leg_info.byte, BYTE_LEG_INFO);
}


void CANSender::send_area_change(unsigned int can_id, unsigned char area)
{
	union can_Area area_change = {};
	area_change.area = area;
	send(can_id, area_change.byte, BYTE_AREA);
}


/*************
 * protected
 *************/
void CANSender::send(unsigned int can_id, unsigned char *data, unsigned short byte)
{
	CANMessage msg(can_id, (const char*)data, byte);
	if(can->write(msg));
	else if(can->write(msg));
	else (can->write(msg));
}
