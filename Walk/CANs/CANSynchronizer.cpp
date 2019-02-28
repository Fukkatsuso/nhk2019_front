/*
 * CANSynchronizer.cpp
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */


#include "CANSynchronizer.h"


CANSynchronizer::CANSynchronizer(CAN *can)
{
	this->can = can;
}
