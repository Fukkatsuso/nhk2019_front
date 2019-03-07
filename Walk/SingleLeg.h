/*
 * SingleLeg.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */


#ifndef WALK_SINGLELEG_H_
#define WALK_SINGLELEG_H_

#include "mbed.h"
#include "Protected/LegConst.h"
#include "Protected/InverseKinematics.h"
#include "QEI/SingleLegQEI.h"
#include "InitSwitch.h"
#include "LegPID.h"
#include "MRMode.h"


#define ANGLE_MAX 118.6f//degree
#define ANGLE_MIN -13.05f//degree


//rad -> degree
float rad_to_degree(float rad);


class SingleLeg : public InverseKinematics
{
public:
	//引数：前後, 左右, リンクの根元の水平位置座標, 鉛直位置座標 (リンクの根元が原点)
	SingleLeg(LegPosition arg_fr, LegPosition arg_rl, float hrz_base, float vrt_base);
	//~SingleLeg();
	void unitize(PwmOut *motor, SingleLegQEI *enc, InitSwitch *sw, Timer *timer);
	void set_dependencies(MRMode *mode);

	//x反転する
	void move_to(float arg_x, float arg_y);
	void move_to(float arg_x, float arg_y, float duty_max, float duty_min);
	void move_to_angle(float target_degree);
	void state_update();
	//void lim_angle(float);

	//後の計算に使用(しない)
	short get_rl();
	//デバッグ用
	float get_duty();
	float get_x();
	float get_y();
	float get_angle();//目標角度[degree]
	float get_P();
	float get_I();
	float get_D();
	//センサーから
	float get_enc();
	int get_sw();
	void reset_enc();

	void set_PID_from_file(const char *fileName);
	void set_PID(float Kp, float Ki, float Kd);
	void set_limits();
	void set_duty_limit(float d_max, float d_min);
	void reset_duty();
	void reset_duty(float reset);

private:
	const short fr;
	const short rl;

	struct{
		int sw;
		float enc;
		float duty;
	}status;

	PwmOut *motor;
	SingleLegQEI *enc;
	InitSwitch *sw;
	MRMode *MRmode;
	LegPID legPID;
};


#endif /* WALK_SINGLELEG_H_ */
