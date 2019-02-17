/*
 * QEI_freePin.h
 *
 *  Created on: 2018/12/21
 *      Author: mutsuro
 */

#ifndef QEI_FREEPIN_H_
#define QEI_FREEPIN_H_

#include "QEI.h"

#define RADS_PER_PULSE (2.0*M_PI/800.0)
//2π[rad]=800[pulse]
#define DEGREE_PER_PULSE (180.0/400.0)
//180[°]=400[pulse]


//エンコーダーの値を角度に変換
float pulse_to_rads(float pulse);
float pulse_to_degree(float pulse);


class QEI_freePin : public QEI
{
public:
	QEI_freePin(PinName channelA, PinName channelB, const float offset);
	float getPulses();

private:
	DigitalIn *PinA;
	DigitalIn *PinB;
	const float offset;
};



#endif /* QEI_FREEPIN_H_ */
