/*
 * InverseKinematics.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_INVERSEKINEMATICS_H_
#define WALK_INVERSEKINEMATICS_H_

#include "mbed.h"


class InverseKinematics
{
public:
	//引数：アームの根元のxy座標。固定。
	InverseKinematics(float x_base, float y_base);

	//角度制限
	void set_angle_limit(float angle_max, float angle_min);

	//アームの長さ：根元から順に2つ
	void lengths(float upper, float fore);

	//指定の座標xyに動かすための、アームの角度(x軸とのなす角)を計算して返す
	float move_to(float arg_x, float arg_y);

	//返り値：アームの角度。角度見る用
	float get_angle();

protected:
	struct{
		float base;		//アームの根元位置
		float elbow;	//リンクのジョイント位置
		float hand;		//アーム先端の位置
	}x, y;

private:
	float l_upper;		//アームの根元側の長さ
	float l_fore;		//アームの先端側の長さ

	float angle_base;	//根元の角度。x軸とのなす角
	float angle_base_max;
	float angle_base_min;

	//角度計算専用
	float l_hand;
	float angle_hand;
	float angle_fore;

	//一連の計算
	void calc_angle_base();

	void calc_l_hand();
	void calc_angle_fore();
	void calc_angle_hand();
	void calc_x_elbow();
	void calc_y_elbow();
};


#endif /* WALK_INVERSEKINEMATICS_H_ */
