/*
 * ForwardKinematics.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "ForwardKinematics.h"
#include "functions.h"
#include "Protected/LegConst.h"


const float ForwardKinematics::LegInfo::upper = LEG_UPPER;
const float ForwardKinematics::LegInfo::fore = LEG_FORE;


ForwardKinematics::ForwardKinematics(){}


void ForwardKinematics::set_f(float base_x, float base_y, SingleLegQEI *enc)
{
	front.x.base = base_x;
	front.y.base = base_y;
	front.enc = enc;
}


void ForwardKinematics::set_r(float base_x, float base_y, SingleLegQEI *enc)
{
	rear.x.base = base_x;
	rear.y.base = base_y;
	rear.enc = enc;
}


void ForwardKinematics::estimate()
{
	calc_elbow(&front);
	front.x.elbow = front.x.base + front.upper*sin(front.angle);
	calc_elbow(&rear);
	rear.x.elbow = rear.x.base - rear.upper*sin(rear.angle);
	float l = sqrt2(front.x.elbow-rear.x.elbow, front.y.elbow-rear.y.elbow);
	float cosine = limit(cos_formula(l, front.fore, rear.fore), 1.0, 0.0);
	float elevation = atan2(front.y.elbow-rear.y.elbow, front.x.elbow-rear.x.elbow);
	x.hand = front.x.elbow - front.fore * cos(acos(cosine)-elevation);
	y.hand = front.y.elbow + front.fore * sin(acos(cosine)-elevation);
}

void ForwardKinematics::calc_elbow(LegInfo *leg)
{
	leg->angle = leg->enc->getAngle();
	leg->y.elbow = leg->y.base + leg->upper*cos(leg->angle);
}

float ForwardKinematics::get_x()
{
	return x.hand;
}

float ForwardKinematics::get_y()
{
	return y.hand;
}
