/*
 * CANSynchronizer.h
 *
 *  Created on: 2019/02/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANSYNCHRONIZER_H_
#define WALK_CANS_CANSYNCHRONIZER_H_

#include "CANProtocol.h"

class CANSynchronizer : public CANProtocol
{
public:
	CANSynchronizer(CAN *can);
private:
};


#endif /* WALK_CANS_CANSYNCHRONIZER_H_ */
