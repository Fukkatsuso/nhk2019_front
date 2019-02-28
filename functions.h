/*
 * functions.h
 *
 *  Created on: 2018/11/09
 *      Author: mutsuro
 *
 *  汎用的な関数を定義
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "mbed.h"


//リミット:int
extern int limit(int value, int max, int min);

//リミット:double
extern double limit(double value, double max, double min);

//二乗和平方根
extern float sqrt2(float a, float b);

//余弦定理
extern float cos_formula(float A1, float A2, float B);


#endif /* FUNCTIONS_H_ */
