/*
 * CANCommand.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_CANCOMMAND_H_
#define WALK_CANCOMMAND_H_

#include "mbed.h"


/*
 * 中央制御用マイコンとCAN通信
 */
class CANCommand
{
public:
	CANCommand(CAN *can);
	int get_area();

private:
	CAN *can;
};


#endif /* WALK_CANCOMMAND_H_ */
