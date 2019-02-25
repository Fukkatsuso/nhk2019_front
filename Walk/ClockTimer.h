/*
 * ClockTimer.h
 *
 *  Created on: 2019/02/25
 *      Author: mutsuro
 */

#ifndef WALK_CLOCKTIMER_H_
#define WALK_CLOCKTIMER_H_

#include "mbed.h"


//各脚で使う
class ClockTimer : public Timer{
public:
	ClockTimer();
	void reset();
	void calc_dt();
	float get_dt();
private:
	struct{
		float prev;
		float dif;
	}timer;
};



#endif /* WALK_CLOCKTIMER_H_ */
