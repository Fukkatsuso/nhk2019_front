/*
 * SingleLegQEI.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "SingleLegQEI.h"
#include "Walk/Protected/LegConst.h"


SingleLegQEI::SingleLegQEI(PinName channelA, PinName channelB):
	QEI_freePin(channelA, channelB, OFFSET_ENC)
{}


//SingleLeg専用角度[degree]取得
float SingleLegQEI::getAngle()
{
	return pulse_to_degree(QEI_freePin::getPulses());
}


float SingleLegQEI::getRadian()
{
	return pulse_to_rads(QEI_freePin::getPulses());
}
