/*
 * CANReceiver.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANRECEIVER_H_
#define WALK_CANS_CANRECEIVER_H_

#include "CANProtocol.h"


class CANReceiver : public CANProtocol
{
public:
	CANReceiver(CAN *can);
	void receive(CANMessage msg);

	//WalkCommand
	unsigned char get_area();
	signed short get_speed();
	signed short get_direction();
	signed short get_pitch();
	unsigned char get_leg_up_FR();
	unsigned char get_leg_up_FL();
	unsigned char get_leg_up_RR();
	unsigned char get_leg_up_RL();

	//LegInfo-Front
	signed short get_dist_front();
	unsigned char get_state_fr();
	unsigned char get_state_fl();
	unsigned char get_kouden_sanddune_front();
	//LegInfo-Rear
	signed short get_dist_rear();
	unsigned char get_state_rr();
	unsigned char get_state_rl();
	unsigned char get_kouden_sanddune_rear();

	//Area
	unsigned short get_area_change();

protected:
	void reset();

	struct{
		union can_WalkCommand data;		//ControllerからSlaveへの指示
		bool semaphore;
	}walk_command;

	struct{
		union can_LegInfo data;		//前脚動作情報
		bool semaphore;
	}leg_info_front;

	struct{
		union can_LegInfo data;		//後脚動作情報
		bool semaphore;
	}leg_info_rear;

	struct{
		union can_Area data; 		//エリア変更要請
		bool semaphore;
	}area_change;

	void copy_data(CANMessage msg, unsigned char data[], short byte);
};


#endif /* WALK_CANS_CANRECEIVER_H_ */
