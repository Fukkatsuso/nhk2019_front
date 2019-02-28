/*
 * CANReceiver.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANRECEIVER_H_
#define WALK_CANS_CANRECEIVER_H_

#include "CANProtocol.h"


class CANReceiver : public CANProtocol
{
public:
	CANReceiver(CAN *can);
private:
};


#endif /* WALK_CANS_CANRECEIVER_H_ */
