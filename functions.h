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

//リミット:unsigned int
extern unsigned int limit_uint(unsigned int value, unsigned int max);

//二乗和平方根
extern float sqrt2(float a, float b);

//余弦定理
extern float cos_formula(float A1, float A2, float B);

//毎ループごとに一定速度で増加させる
extern float trapezoidal_control(float now, float initial, float target, float roop_period, float time_required);

//符号なし整数カウンタ更新
unsigned int counter_update(unsigned int counter, unsigned char flag);

//2数を比較して大きい方を返す
unsigned int max_uint(unsigned int a, unsigned int b);

//valueからcutだけ減らした数を返す
unsigned int uint_cut(unsigned int value, unsigned int cut);


#endif /* FUNCTIONS_H_ */
