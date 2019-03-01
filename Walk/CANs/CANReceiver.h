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
	void receive(unsigned int id_can, unsigned char data_can[]);
	float get_data(enum CANID::DataType type);

	int get_area();

protected:
	float decode_from_candata(unsigned char data_can[], int len_i, int len_f);

private:
	float data[CANID::DataType_end];//DataTypeごとの値
};


#endif /* WALK_CANS_CANRECEIVER_H_ */
