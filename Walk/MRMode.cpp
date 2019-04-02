/*
 * MRMode.cpp
 *
 *  Created on: 2019/02/18
 *      Author: mutsuro
 */


#include "MRMode.h"

#define X_NORMAL_WIDTH 60
#define X_CLIMB_MAX 130
#define X_CLIMB_MIN -180

Limits limits[MRMode::Area_end] =
		{	//----------ParallelLeg----------   ----------SingleLeg----------
			//{{x.max, 			x.min}, 			{y.max, y.min}, {angle.max, angle.min}, {duty.max, duty.min}}
			  {{X_NORMAL_WIDTH,	-X_NORMAL_WIDTH}, 	{280, 	200}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//WaitGobiUrtuu
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, 	{280, 	200}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//GetGerege
			  {{X_NORMAL_WIDTH,	-X_NORMAL_WIDTH}, 	{280, 	200}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//PrepareWalking
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, 	{280, 	200}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//Start1
			  {{90, -90}, 							{280, 	200}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//GobiArea
			  {{100, -100}, 						{260, 	120}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//SandDuneFront
			  {{100, -100}, 						{260, 	120}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//SandDuneRear
			  {{90, -90}, 							{280, 	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//ReadyForTussock
			  {{90, -90}, 							{280, 	80}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//Tussock
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, 	{280, 	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//Finish1
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, 	{260, 	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//WaitMountainUrtuu
			  {{X_NORMAL_WIDTH, -X_NORMAL_WIDTH}, 	{260, 	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//GetSign
			  {{90, -90},						 	{260, 	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//Start2
			  {{X_CLIMB_MAX,  	X_CLIMB_MIN}, 		{260, 	100}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//StartClimb1//Front開始
			  {{X_CLIMB_MAX,  	X_CLIMB_MIN}, 		{260, 	100}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//StartClimb2
			  {{X_CLIMB_MAX,	X_CLIMB_MIN},		{260, 	100}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//MountainArea
			  {{100, 			-100}, 				{260, 	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//UukhaiZone
			  {{100, 			-100}, 				{260,	150}, 	{110, 		-20}, 		{0.8, 	   0.2}},	//Uukhai
			  {{100,			-100}, 				{260,	150}, 	{110, 		-20}, 		{0.8, 	   0.2}}	//Finish2
		};

#define HEIGHT_NORMAL 40
#define GRAD_SLOPE 14.9
#define X_CLIMB_INIT -50//-53 // = init_y * tan(gradient)
#define X_NORMAL_INIT -20

Orbits orbits[MRMode::Area_end] =
		{		//gradient, 	init_x,			init_y, height,				time_change
				{0,				0, 				240, 	HEIGHT_NORMAL,		1},		//WaitGobiUrtuu
				{0, 			0,				240, 	HEIGHT_NORMAL,		1},		//GetGerege
				{0, 			X_NORMAL_INIT,	260, 	HEIGHT_NORMAL,		0.5},		//PrepareWalking
				{0, 			X_NORMAL_INIT,	260, 	HEIGHT_NORMAL,		1},		//Start1
				{0, 			X_NORMAL_INIT,	260, 	HEIGHT_NORMAL,		1},		//GobiArea
				{0, 			0,				260,	140,				1},		//SandDuneFront
				{0,				0,				260,	30,					0},		//SandDuneRear
				{0, 			0,				260, 	HEIGHT_NORMAL,		1},		//ReadyForTussock
				{0, 			X_NORMAL_INIT,	260, 	HEIGHT_NORMAL,		1},		//Tussock
				{0, 			X_NORMAL_INIT,	260, 	HEIGHT_NORMAL,		1},		//Finish1
				{0, 			0,				260, 	HEIGHT_NORMAL,		1},		//WaitMountainUrtuu
				{0, 			0,				200, 	HEIGHT_NORMAL,		0.5},		//GetSign //ここで姿勢変更
				{0, 			0,				200, 	HEIGHT_NORMAL,		1},		//Start2
				{GRAD_SLOPE,	0, 				200, 	50,					1},		//StartClimb1//Front開始
				{0,				X_CLIMB_INIT,	200, 	40,					1},		//StartClimb2
				{0, 			X_CLIMB_INIT,	200, 	40,					1},		//MountainArea
				{0, 			0,				260, 	50,					0.5},		//UukhaiZone
				{0, 			0,				260, 	50,					0.5},		//Uukhai
				{0, 			0,				260, 	50,					0.5}		//Finish2
		};


MRMode::MRMode(CANReceiver *rcv, CANSender *snd, enum Area init_area, bool operate=false)
{
	this->can_receiver = rcv;
	this->can_sender = snd;
	area[Now] = area[Initial] = init_area;
	flag.operate = operate;
	roop_prev = init_area;
}


//getする前に必ず実行すること
void MRMode::update()
{
	area[Now] = (Area)(can_receiver->get_area());//今のArea
	roop_prev = roop_now;	roop_now = area[Now];
	flag.switched = (roop_now!=roop_prev);//Area指令切り替わりの判断
	//エリア切り替わりに関して
	if(flag.switched){
		area[Prev] = roop_prev;//1つ前のArea
		area[Next] = (Area)((int)area[Now] + (((int)area[Now]<(int)Finish2)? 1:0));//次のArea（予定）
		flag.changing_area = true;
		x_vel_change_init = (orbits[area[Now]].init_x - orbits[area[Prev]].init_x) / orbits[area[Now]].time_change;
		y_vel_change_init = (orbits[area[Now]].init_y - orbits[area[Prev]].init_y) / orbits[area[Now]].time_change;
		timer_changing_area.reset();
		timer_changing_area.start();
		time_changing_area_prev = timer_changing_area.read();
	}
	if(flag.changing_area){
		timeslice_changing_area = timer_changing_area.read() - time_changing_area_prev;
		time_changing_area_prev = timer_changing_area.read();
	}
	if(timer_changing_area.read() > orbits[area[Now]].time_change){
		timer_changing_area.stop();
		timer_changing_area.reset();
		flag.changing_area = false;
	}
}


bool MRMode::is_switched()
{
	return flag.switched;
}


bool MRMode::is_changing_area()
{
	return flag.changing_area;
}


void MRMode::request_to_change_area(enum Area area_req, CANID::From can_from)
{
	can_sender->send_area_change(CANID::generate(can_from, CANID::ToController, CANID::AreaChange), area_req);
}


MRMode::Area MRMode::get_area(enum Reference ref)
{
	return area[ref];
}


MRMode::Area MRMode::get_now()
{
	return area[Now];
}


Limits* MRMode::get_limits(enum Area area)
{
	return &limits[area];
}


Orbits* MRMode::get_orbits(enum Area area)
{
	return &orbits[area];
}

float MRMode::get_x_dif_change_init()
{
	return x_vel_change_init * timeslice_changing_area;
}

float MRMode::get_y_dif_change_init()
{
	return y_vel_change_init * timeslice_changing_area;
}
