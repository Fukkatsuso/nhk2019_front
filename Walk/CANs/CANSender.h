/*
 * CANSender.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANSENDER_H_
#define WALK_CANS_CANSENDER_H_

#include "mbed.h"
#include "CANProtocol.h"


class CANSender : public CANProtocol
{
public:
	CANSender(CAN *can);

	void send_walk_command(unsigned int can_id,
			unsigned char area, short speed, short direction, short pitch, unsigned char leg_up);
	void send_timer_reset(unsigned int can_id);
	void send_leg_info(unsigned int can_id, short dist, unsigned char state, unsigned char kouden);
	void send_area_change(unsigned int can_id, unsigned char area);

protected:
	void send(unsigned int can_id, unsigned char *data, unsigned short byte);
};


#endif /* WALK_CANS_CANSENDER_H_ */
