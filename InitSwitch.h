/*
 * InitSwitch.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef INITSWITCH_H_
#define INITSWITCH_H_

#include "mbed.h"


class InitSwitch : public DigitalIn
{
public:
	InitSwitch(PinName pin, const bool onSignal=1):
		DigitalIn(pin), onSignal(onSignal){}

	int read(){
		return ((DigitalIn::read()==onSignal)? 1:0);
	}

private:
	const bool onSignal;
};


#endif /* INITSWITCH_H_ */
