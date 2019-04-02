/*
 * PhotoelectricSensor.cpp
 *
 *  Created on: 2019/01/15
 *      Author: mutsuro
 */

#include "PhotoelectricSensor.h"


PhotoelectricSensor::PhotoelectricSensor(PinName input)
{
	Input = new DigitalIn(input);
	Input->mode(PullUp);
	now = prev = 0;
	counter = 0;
}


//毎ループ1回ずつ実行
void PhotoelectricSensor::sensing()
{
	prev = now;
	now = 1-(Input->read());
	if(now)counter++;
	else if(counter>0)counter--;
}


/*
 * 検知:1
 * 非検知:0
 */
//
int PhotoelectricSensor::read()
{
	return now;
}


bool PhotoelectricSensor::is_rising()
{
	return (prev==0 && now==1);
}


unsigned int PhotoelectricSensor::get_counter(unsigned int filter)
{
	if(counter < filter)return 0;
	return counter;
}


void PhotoelectricSensor::reset_counter()
{
	counter = 0;
}
