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
	CANSynchronizer(CAN *can, void (*fptr_timerreset)(void));
	void set_period(float period);
	void set_duty(float duty);
	void timer_reset(bool allReset);

	float get_period();
	float get_duty();

private:
	void (*fptr_timerreset)();
	Ticker ticker;
	float period;
	float duty;
};


#endif /* WALK_CANS_CANSYNCHRONIZER_H_ */
