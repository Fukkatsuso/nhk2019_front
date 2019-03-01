/*
 * CANSynchronizer.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANSYNCHRONIZER_H_
#define WALK_CANS_CANSYNCHRONIZER_H_

#include "mbed.h"
#include "CANSender.h"


//Master
class CANSynchronizer : public CANSender
{
public:
	CANSynchronizer(CAN *can);
	void set_period(float period, bool send);
	void set_duty(float duty, bool send);

private:
	float period;
	float duty;
};


#endif /* WALK_CANS_CANSYNCHRONIZER_H_ */
