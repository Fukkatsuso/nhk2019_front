/*
 * SingleLegQEI.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef QEI_SINGLELEGQEI_H_
#define QEI_SINGLELEGQEI_H_

#include "mbed.h"
#include "QEI_freePin.h"


class SingleLegQEI : public QEI_freePin
{
public:
	SingleLegQEI(PinName channelA, PinName channelB);
	float getAngle();
	float getRadian();
};


#endif /* QEI_SINGLELEGQEI_H_ */
