/*
 * LegPID.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_LEGPID_H_
#define WALK_LEGPID_H_

#include "mbed.h"
#include "PID.h"
#include "SingleLegQEI.h"


class LegPID : public PID
{
public:
	LegPID();

	void set_PID(SingleLegQEI *enc, float Kp, float Ki, float Kd);

	float calc_duty(float tgt_angle);

	float get_enc();
	void reset_duty();

private:
	SingleLegQEI *enc;

	float tgt_prv;
};


#endif /* WALK_LEGPID_H_ */
