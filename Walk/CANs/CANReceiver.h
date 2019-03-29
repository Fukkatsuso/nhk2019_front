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
	void receive(CANMessage msg);

protected:
	void copy_data(CANMessage msg, unsigned char data[], short byte);
};


#endif /* WALK_CANS_CANRECEIVER_H_ */
