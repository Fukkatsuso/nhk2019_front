/*
 * InverseKinematics.cpp
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#include "InverseKinematics.h"
#include "functions.h"
#include "LegConst.h"


const float InverseKinematics::l_upper = LEG_UPPER;
const float InverseKinematics::l_fore = LEG_FORE;



/************
 * 			*
 *  public  *
 *  		*
 ************/
InverseKinematics::InverseKinematics(float x_base, float y_base)
{
	x.base = x_base;
	y.base = y_base;
}


//static void InverseKinematics::lengths(float upper, float fore)
//{
//	l_upper = upper;
//	l_fore = fore;
//}


void InverseKinematics::set_angle_limit(float angle_max, float angle_min)
{
	angle_base_max = angle_max;
	angle_base_min = angle_min;
}


float InverseKinematics::move_to(float arg_x, float arg_y)
{
	x.hand = arg_x;
	y.hand = arg_y;

	calc_angle_base();

	return angle_base;
}


float InverseKinematics::get_angle()
{
	return angle_base;
}


void InverseKinematics::calc_angle_base()
{
	calc_l_hand();
	calc_angle_fore();
	calc_angle_hand();
	calc_x_elbow();
	calc_y_elbow();

	angle_base = limit(atan2(y.elbow-y.base, x.elbow-x.base), angle_base_max, angle_base_min);
}


void InverseKinematics::calc_l_hand()
{
	l_hand = sqrt2(x.hand-x.base, y.hand-y.base);
}


void InverseKinematics::calc_angle_fore()
{
	angle_fore = atan2(x.hand-x.base, -(y.hand-y.base));
}


void InverseKinematics::calc_angle_hand()
{
	float cos = (float)limit(cos_formula(l_hand, l_fore, l_upper), 1.0, -1.0);
	angle_hand = acos(cos);
}


void InverseKinematics::calc_x_elbow()
{
	x.elbow = x.hand - l_fore * sin(angle_fore - angle_hand);
}


void InverseKinematics::calc_y_elbow()
{
	y.elbow = y.hand + l_fore * cos(angle_fore - angle_hand);
}


