/*
 * ParallelLeg.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_PARALLELLEG_H_
#define WALK_PARALLELLEG_H_

#include "mbed.h"
#include "MRMode.h"
#include "ClockTimer.h"
#include "CANs/CANReceiver.h"
#include "CANs/CANSynchronizer.h"

//脚持ち上げ:脚下げ = LEGUP:LEGDOWN
#define LEGUP_MOVE 4.0f
#define LEGDOWN_MOVE (1.0f)
#define LEGUP_TIME 1.0f
#define LEGDOWN_TIME 1.0f//4.0f

//障害物用
//脚上げ:脚復帰スライド:脚下げ = LEGUP_STABLE_TIME:LEGSLIDE_STABLE_TIME:LEGDOWN_STABLE_TIME
#define LEGUP_STABLE_TIME 1.0f
#define LEGSLIDE_STABLE_TIME 1.0f
#define LEGDOWN_STABLE_TIME 1.0f

#define X_STAY_MARGIN 2.0f
#define Y_STAY_MARGIN 2.0f


class ParallelLeg
{
public:
	ParallelLeg(int fr, int rl, float pos_x, float pos_y);
	void set_dependencies(ClockTimer *tm_period, MRMode *mode, CANReceiver *can_rcv, CANSynchronizer *can_syn);
	void mrmode_update(); //MRModeの更新を反映させる

	void set_x_lim(float xmax, float xmin);
	void set_y_lim(float ymax, float ymin);
	void set_limits();

	void set_x_initial(float x_initial);
	void set_y_initial(float y_initial);
	void set_initial(float x_initial, float y_initial);
	void set_height(float height);
	void set_gradient(float grad);
	void set_orbits();

	void set_period(float period);
	void set_duty(float duty);

	//速度, 方向 -> 次の着地点（歩幅）
	void walk(float spd, float dir);
	void walk();

	void walk_stable(float spd, float dir, float rate_stablemove_time);

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
//	void calc_dt(float tm);
	void timer_update();
	void set_timing();
	void walk_mode();
	void check_flag();
	void calc_velocity();
		void calc_step();
		void calc_vel_recovery();
	void calc_position();

	void set_stable_timing();
	void walk_stable_mode();
	void check_stable_flag();
	void calc_stable_velocity();
	void calc_stable_step();
	void calc_stable_vel_recovery();
	void calc_stable_position();

private:
	const short fr;
	const short rl;

	ClockTimer *timer_period;
	MRMode *MRmode;
	CANReceiver *can_receiver;
	CANSynchronizer *can_synchronizer;
	float gradient; //フィールド勾配
	float height;	//振り上げ高さ

	float period;
	float duty;
	float speed;
	float direction;
	float time_stablemove; //StableMoveする時間

	float step;	//着地点

	struct{
		float vel;
		struct{
			float init; //MRModeで設定しておく
			float now;
			float dif;
			float next;
			float min;
			float max;
			float recover_start;
		}pos;
	}x, y;

	float timing[4];//時刻0, 復帰開始時刻, 復帰完了時刻, 1周期時刻
	//時刻0, 復帰開始時刻, 復帰完了時刻, 胴体移動開始時刻1, 胴体移動完了時刻1, 胴体移動開始時刻2, 胴体移動完了時刻2, 1周期時刻
	float timing_stable[8];
	LegMode mode;
	LegMode mode_prv;
	MRMode::Area area;
	MRMode::Area area_prv;

	struct{
		bool timer_reset;//タイマーリセットの指令があるか//使うか未定
		bool recovery;//復帰完了
		bool stay_command;//静止コマンド
		bool stay;//静止状態	CANで送信するプログラムを実装せねば
		bool first_cycle;//最初の歩行サイクル
		bool climb;//登山
	}flag;

//	struct{
//		float prv;
//		float now;
//		float dif;
//	}time;
};


#endif /* WALK_PARALLELLEG_H_ */
