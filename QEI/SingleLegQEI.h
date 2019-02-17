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

#define OFFSET_ENC (85.0*400.0/180.0)//鉛直方向とのオフセット角度をパルスに変換//180°=400pulses


class SingleLegQEI : public QEI_freePin
{
public:
	SingleLegQEI(PinName channelA, PinName channelB);
	float getAngle();
	float getRadian();
};


#endif /* QEI_SINGLELEGQEI_H_ */
