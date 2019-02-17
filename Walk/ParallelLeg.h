/*
 * ParallelLeg.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_PARALLELLEG_H_
#define WALK_PARALLELLEG_H_

#include "mbed.h"

//絶対固定
#define BASE_X 20
#define LEG_UPPER 135
#define LEG_FORE 160


class ParallelLeg
{
public:
	ParallelLeg(float pos_x, float pos_y);

	void set_x_lim(float xmax, float xmin);
	void set_y_lim(float ymax, float ymin);
	void set_x_initial(float x_initial);
	void set_y_initial(float y_initial);
	void set_initial(float x_initial, float y_initial);
	void set_high(float high);
	void set_gradient(float grad);
	void set_period(float p);
	void set_duty(float d);

	//速度, 方向 -> 次の着地点（歩幅）
	void walk(float spd, float dir, float tm);

	//足先座標を返す
	float get_x();
	float get_y();
	float get_x_initial();
	float get_y_initial();
	float get_x_vel();
	float get_y_vel();
	int get_mode();
	bool is_recovery();
	bool is_stay();
	bool is_climb();

protected:
	float curve_adjust(float value);
	void calc_dt(float tm);
	void set_timing();
	void walk_mode();
	void check_flag();
	void calc_velocity();
		void calc_step();
		void calc_vel_recovery();
	void calc_position();

private:
	short fr;
	short rl;

	float gradient;
	float period;
	float duty;
	float speed;
	float direction;

	float step;	//着地点
	float high;	//振り上げ高さ

	struct{
		float vel;
		struct{
			float init;
			float now;
			float dif;
			float next;
			float min;
			float max;
			float recover_start;
		}pos;
	}x, y;

	float timing[4];//時刻0, 復帰開始時刻, 復帰完了時刻, 1周期時刻
	short mode;
	short mode_prv;

	struct{
		bool recovery;//復帰完了
		bool stay_command;//静止コマンド
		bool stay;//静止状態
		bool first_cycle;//最初の歩行サイクル
		bool climb;//登山
	}flag;

	struct{
		float prv;
		float now;
		float dif;
	}time;
};


#endif /* WALK_PARALLELLEG_H_ */
