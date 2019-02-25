/*
 * ClockTimer.cpp
 *
 *  Created on: 2019/02/25
 *      Author: mutsuro
 */

#include "ClockTimer.h"


ClockTimer::ClockTimer()
{
	reset();start();
	timer.prev = read();
}


void ClockTimer::reset()
{
	Timer::reset();
	timer.prev = read();
}


void ClockTimer::calc_dt()
{
	float now = read();
	timer.dif = now - timer.prev;
	if(timer.dif<0)timer.dif = now;
	timer.prev = now;
}


float ClockTimer::get_dt()
{
	return timer.dif;
}
