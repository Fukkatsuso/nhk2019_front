/*
 * CANSender.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANSENDER_H_
#define WALK_CANS_CANSENDER_H_

#include "CANProtocol.h"


class CANSender : public CANProtocol
{
public:
	CANSender(CAN *can);
private:
};


#endif /* WALK_CANS_CANSENDER_H_ */
