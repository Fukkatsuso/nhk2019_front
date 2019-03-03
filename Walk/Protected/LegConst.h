/*
 * LegConst.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_LEGCONST_H_
#define WALK_LEGCONST_H_


//絶対固定
#define LEG_UPPER 135
#define LEG_FORE 160
#define BASE_X 20

//28.6+90 = 118.6
#define OFFSET_ENC (118.6*400.0/180.0)//鉛直方向とのオフセット角度をパルスに変換//180°=400pulses


#endif /* WALK_LEGCONST_H_ */
