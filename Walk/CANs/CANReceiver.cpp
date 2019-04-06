/*
 * CANReceiver.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#include "CANReceiver.h"


CANReceiver::CANReceiver(CAN *can) : CANProtocol(can)
{
//	this->can = can;
	reset();
}


void CANReceiver::receive(CANMessage msg)
{
	unsigned int type = msg.id & 0x00f;

	switch(type){
	case CANID::WalkCommand:
		walk_command.semaphore = false;
		copy_data(msg, walk_command.data.byte, BYTE_WALK_COMMAND);
		walk_command.semaphore = true;
		return;

	case CANID::LegInfo:
		if(CANID::is_from(msg.id, CANID::FromFront)){
			leg_info_front.semaphore = false;
			copy_data(msg, leg_info_front.data.byte, BYTE_LEG_INFO);
			leg_info_front.semaphore = true;
		}
		else if(CANID::is_from(msg.id, CANID::FromRear)){
			leg_info_rear.semaphore = false;
			copy_data(msg, leg_info_rear.data.byte, BYTE_LEG_INFO);
			leg_info_rear.semaphore = true;
		}
		return;

	case CANID::AreaChange:
		area_change.semaphore = false;
		copy_data(msg, area_change.data.byte, BYTE_AREA);
		area_change.semaphore = true;
		return;

	default:
		return;
	}
}


void CANReceiver::copy_data(CANMessage msg, unsigned char data[], short byte)
{
	for(int i=0; i<byte; i++){
		data[i] = msg.data[i];
	}
}


/****************
 * walk_command
 ****************/
unsigned char CANReceiver::get_area()
{
	while(!walk_command.semaphore);
	return walk_command.data.area;
}

signed short CANReceiver::get_speed()
{
	while(!walk_command.semaphore);
	return walk_command.data.speed;
}

signed short CANReceiver::get_direction()
{
	while(!walk_command.semaphore);
	return walk_command.data.direction;
}

signed short CANReceiver::get_pitch()
{
	while(!walk_command.semaphore);
	return walk_command.data.pitch;
}

unsigned char CANReceiver::get_leg_up_FR()
{
	while(!walk_command.semaphore);
	return walk_command.data.leg_up.FR;
}

unsigned char CANReceiver::get_leg_up_FL()
{
	while(!walk_command.semaphore);
	return walk_command.data.leg_up.FL;
}

unsigned char CANReceiver::get_leg_up_RR()
{
	while(!walk_command.semaphore);
	return walk_command.data.leg_up.RR;
}

unsigned char CANReceiver::get_leg_up_RL()
{
	while(!walk_command.semaphore);
	return walk_command.data.leg_up.RL;
}


/******************
 * leg_info_front
 ******************/
signed short CANReceiver::get_dist_front()
{
	while(!leg_info_front.semaphore);
	return leg_info_front.data.dist;
}

unsigned char CANReceiver::get_state_fr()
{
	while(!leg_info_front.semaphore);
	return leg_info_front.data.state.right;
}

unsigned char CANReceiver::get_state_fl()
{
	while(!leg_info_front.semaphore);
	return leg_info_front.data.state.left;
}

unsigned char CANReceiver::get_kouden_sanddune_front()
{
	while(!leg_info_front.semaphore);
	return leg_info_front.data.kouden.sand_dune;
}

//leg_info_rear
signed short CANReceiver::get_dist_rear()
{
	while(!leg_info_rear.semaphore);
	return leg_info_rear.data.dist;
}

unsigned char CANReceiver::get_state_rr()
{
	while(!leg_info_rear.semaphore);
	return leg_info_rear.data.state.right;
}

unsigned char CANReceiver::get_state_rl()
{
	while(!leg_info_rear.semaphore);
	return leg_info_rear.data.state.left;
}

unsigned char CANReceiver::get_kouden_sanddune_rear()
{
	while(!leg_info_rear.semaphore);
	return leg_info_rear.data.kouden.sand_dune;
}


/***************
 * area_change
 ***************/
unsigned short CANReceiver::get_area_change()
{
	while(!area_change.semaphore);
	return area_change.data.area;
}


/*************
 * protected
 *************/
void CANReceiver::reset()
{
	for(int i=0; i<BYTE_WALK_COMMAND; i++){
		walk_command.data.byte[i] = 0;;
	}
	walk_command.semaphore = true;

	for(int i=0; i<BYTE_LEG_INFO; i++){
		leg_info_front.data.byte[i] = 0;
	}
	leg_info_front.semaphore = true;

	for(int i=0; i<BYTE_LEG_INFO; i++){
		leg_info_rear.data.byte[i] = 0;
	}
	leg_info_rear.semaphore = true;

	for(int i=0; i<BYTE_AREA; i++){
		area_change.data.byte[i] = 0;
	}
	area_change.semaphore = true;
}
