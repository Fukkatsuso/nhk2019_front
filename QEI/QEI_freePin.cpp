/*
 * QEI_freePin.cpp
 *
 *  Created on: 2018/12/21
 *      Author: mutsuro
 */

#include "QEI_freePin.h"


//エンコーダーの値を角度に変換
float pulse_to_rads(float pulse)
{
	return pulse * (RADS_PER_PULSE);
}

float pulse_to_degree(float pulse)
{
	return pulse * (DEGREE_PER_PULSE);
}


QEI_freePin::QEI_freePin(PinName channelA, PinName channelB, const float offset=0):
	QEI(channelA, channelB, NC, 624, QEI::X4_ENCODING),
	offset(offset)
{
	PinA = new DigitalIn(channelA);
	PinB = new DigitalIn(channelB);

	PinA->mode(PullUp);
	PinB->mode(PullUp);
}


float QEI_freePin::getPulses()
{
	return (float)QEI::getPulses() + offset;
}
