/*
 * functions.cpp
 *
 *  Created on: 2018/11/16
 *      Author: mutsuro
 */

#include "functions.h"

int limit(int value, int max, int min){
	if(value > max)return max;
	else if(value < min)return min;
	else return value;
}

double limit(double value, double max, double min){
	if(value > max)return max;
	else if(value < min)return min;
	else return value;
}

float sqrt2(float a, float b){
	return sqrt(a*a + b*b);
}

float cos_formula(float A1, float A2, float B){
	return ((A1)*(A1) + (A2)*(A2) - B*B) / (2 * (A1) * (A2));
}
