/*
 * MRMode.cpp
 *
 *  Created on: 2019/02/18
 *      Author: mutsuro
 */


#include "MRMode.h"


Limits limits[MRMode::Area_end] =
		{		//--------ParallelLeg--------     --------SingleLeg--------
				//{x.max, x.min}, {y.max, y.min}, {angle.max, angle.min}, {duty.max, duty.min}
				{{}, {}, {}, {}},	//WaitGobiUrtuu
				{{}, {}, {}, {}},	//GetGerege
				{{}, {}, {}, {}},	//PrepareWalking
				{{}, {}, {}, {}},	//Start1
				{{}, {}, {}, {}},	//GobiArea
				{{}, {}, {}, {}},	//SandDune
				{{}, {}, {}, {}},	//ReadyForTussock
				{{}, {}, {}, {}},	//Tussock1
				{{}, {}, {}, {}},	//Tussock2
				{{}, {}, {}, {}},	//Finish1
				{{}, {}, {}, {}},	//WaitMountainUrtuu
				{{}, {}, {}, {}},	//GetSign
				{{}, {}, {}, {}},	//Start2
				{{}, {}, {}, {}},	//MountainArea
				{{}, {}, {}, {}},	//UukhaiZone
				{{}, {}, {}, {}},	//Uukhai
				{{}, {}, {}, {}}	//Finish2
		};

Orbits orbits[MRMode::Area_end] =
		{		//gradient, init, high
				{0,		280, 	10},	//WaitGobiUrtuu
				{0, 	280, 	10},	//GetGerege
				{0, 	250, 	10},	//PrepareWalking
				{0, 	250, 	100},	//Start1
				{0, 	250, 	100},	//GobiArea
				{0, 	280, 	150},	//SandDune
				{0, 	250, 	100},	//ReadyForTussock
				{0, 	280, 	150},	//Tussock1
				{0, 	280, 	150},	//Tussock2
				{0, 	250, 	100},	//Finish1
				{0, 	250, 	100},	//WaitMountainUrtuu
				{0, 	250, 	100},	//GetSign
				{0, 	250, 	100},	//Start2
				{14.9, 	150, 	80},	//MountainArea
				{0, 	250, 	80},	//UukhaiZone
				{0, 	250, 	100},	//Uukhai
				{0, 	250, 	100}	//Finish2
		};


MRMode::MRMode(CANReceiver *rcv, enum Area init_area, bool operate=false)
{
	this->can_receiver = rcv;
	area[Now] = area[Initial] = init_area;
	flag.operate = operate;
}


//getする前に必ず実行すること
void MRMode::update()
{
	area[Now] = (Area)(can_receiver->get_area());//今のArea
	roop_prev = roop_now;	roop_now = area[Now];
	flag.switched = (roop_now!=roop_prev);//Area切り替わりの判断
	area[Prev] = (Area)((int)area[Now] - (((int)area[Now]>(int)area[Initial])? 1:0));//1つ前のArea
	area[Next] = (Area)((int)area[Now] + (((int)area[Now]<(int)Finish2)? 1:0));//次のArea（予定）
}

bool MRMode::is_switched()
{
	return flag.switched;
}


MRMode::Area MRMode::get_area(enum Reference ref){
	return area[ref];
}

//float MRMode::get_lim_x_max(enum Reference ref){
//	return limit->x.max;
//}
//
//float MRMode::get_lim_x_min(enum Reference ref){
//	return limit->y.min;
//}
//
//float MRMode::get_lim_y_max(enum Reference ref){
//	return limit->y.max;
//}
//
//float MRMode::get_lim_y_min(enum Reference ref){
//	return limit->y.min;
//}
//
//float MRMode::get_lim_angle_max(enum Reference ref){
//	return limit->angle.max;
//}
//
//float MRMode::get_lim_angle_min(enum Reference ref){
//	return limit->angle.min;
//}
//
//float MRMode::get_lim_duty_max(enum Reference ref){
//	return limit->duty.max;
//}
//
//float MRMode::get_lim_duty_min(enum Reference ref){
//	return limit->duty.min;
//}

Limits* MRMode::get_limits(enum Area area){
	return &limits[area];
}

//float MRMode::get_gradient(enum Reference ref){
//	return toe->gradient;
//}
//
//float MRMode::get_init(enum Reference ref){
//	return
//}
//
//float MRMode::get_high(enum Reference ref){
//
//}

Orbits* MRMode::get_orbits(enum Area area){
	return &orbits[area];
}
