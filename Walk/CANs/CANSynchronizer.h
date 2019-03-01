/*
 * CANSynchronizer.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANSYNCHRONIZER_H_
#define WALK_CANS_CANSYNCHRONIZER_H_

#include "mbed.h"
#include "CANProtocol.h"
#include "CANSender.h"


class CANSynchronizer : public CANProtocol
{
public:
	CANSynchronizer(CAN *can, CANSender *can_sender);
	void set_period(float period, bool send);
	void set_duty(float duty, bool send);

private:
	CANSender *can_sender;
	float period;
	float duty;
};


#endif /* WALK_CANS_CANSYNCHRONIZER_H_ */
