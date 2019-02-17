/*
 * ForwardKinematics.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_FORWARDKINEMATICS_H_
#define WALK_FORWARDKINEMATICS_H_

#include "mbed.h"
#include "QEI/SingleLegQEI.h"


struct LegInfo{
	float upper;
	float fore;
	SingleLegQEI *enc;
	float angle;
	struct{
		float base;
		float elbow;
	}x, y;
};

class ForwardKinematics{
public:
	ForwardKinematics();
	void set_f(float base_x, float base_y, float upper, float fore, SingleLegQEI *enc);
	void set_r(float base_x, float base_y, float upper, float fore, SingleLegQEI *enc);
	void estimate();
	void calc_elbow(LegInfo *leg);
	float get_x();
	float get_y();
private:
	LegInfo front, rear;
	struct{
		float hand;
	}x, y;
};


#endif /* WALK_FORWARDKINEMATICS_H_ */
