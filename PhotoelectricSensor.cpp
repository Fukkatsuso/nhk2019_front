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
}


void PhotoelectricSensor::sensing()
{
	prev = now;
	now = 1-(Input->read());
	if(is_rising()){
		tm_kouden.reset();
		tm_kouden.start();
	}
	else if(now==0){
		tm_kouden.stop();
		tm_kouden.reset();
	}
}

/*
 * 検知:1
 * 非検知:0
 */
int PhotoelectricSensor::read()
{
	sensing();
	return now;
}

bool PhotoelectricSensor::is_rising()
{
	return (prev==0 && now==1);
}

float PhotoelectricSensor::get_ontime()
{
	read();
	return tm_kouden.read();
}
