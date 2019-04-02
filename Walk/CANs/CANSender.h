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

	void send_direction(unsigned int can_id, float direction);
	void send_speed(unsigned int can_id, float speed);
	void send_velocity_vector(unsigned int can_id, float direction, float speed);
	void send_timer_reset(unsigned int can_id);
	void send_area(unsigned int can_id, unsigned char area);
	void send_area_change(unsigned int can_id, unsigned char area);
	void send_leg_up(unsigned int can_id, unsigned char leg_up);
	void send_move_position(unsigned int can_id, float dist, unsigned char kouden_sanddune);

protected:
	void send(unsigned int can_id, unsigned char *data, unsigned short byte);
};


#endif /* WALK_CANS_CANSENDER_H_ */
