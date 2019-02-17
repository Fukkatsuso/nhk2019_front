/*
 * PID.h
 *
 *  Created on: 2018/10/31
 *      Author: mutsuro
 *
 * ・ゲインを設定してインスタンス生成
 * ・
 */

#ifndef PID_H_
#define PID_H_

#include "mbed.h"

#define MARGIN_I 5
#define MARGIN_D 30
#define MARGIN_BRAKE 0.3f

class PID
{
public:
	//PID(float Kp, float Ki, float Kd);
	PID();
	~PID();

	void set_PID(float Kp, float Ki, float Kd);

	void start(float obs_init=0, float opr_init=0);

	void param_update(float obs_now, float obs_tgt);
	void calc_P();
	void calc_I();
	void calc_D();
	void calc(float obs_now, float obs_tgt);

	void param_set_limit(float max, float min);
	void param_limit();

	float get_opr_nxt();
	float get_obs_dif();

	float get_P();
	float get_I();
	float get_D();

	void reset_opr_nxt(float opr_init);

private:
	float Kp;
	float Ki;
	float Kd;

	Timer timer;

	struct{
		float now;
		float prv;
		float dif;
		float min;
	}time;

	struct{
		float nxt;
		float prv;
		float dif;
		float min;
		float max;
		float tgt;
	}opr;//操作量

	struct{
		float now;
		float prv;
		float prv_prv;
		float prv_prv_prv;
		float dif;
		float dif_prv;
		float dif_prv_prv;
		float tgt;
		float tgt_prv;
	}obs;//センサー観測

	double P;
	double I;
	double D;
};


#endif /* PID_H_ */
