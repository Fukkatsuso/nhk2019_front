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


class ForwardKinematics{
public:
	ForwardKinematics();
	ForwardKinematics(float f_bx, float f_by, SingleLegQEI *f_enc,
					  float r_bx, float r_by, SingleLegQEI *r_enc);
	void set_f(float base_x, float base_y, SingleLegQEI *enc);
	void set_r(float base_x, float base_y, SingleLegQEI *enc);
	void estimate();
	float get_x();
	float get_y();

private:
	struct LegInfo{
		static const float upper;
		static const float fore;
		SingleLegQEI *enc;
		float angle;
		struct{
			float base;
			float elbow;
		}x, y;
	}front, rear;

	struct{
		float hand;
	}x, y;

protected:
	void calc_elbow(LegInfo *leg);
};


#endif /* WALK_FORWARDKINEMATICS_H_ */
