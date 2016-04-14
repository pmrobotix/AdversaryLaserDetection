#ifndef _LedPannelsManager_H_
#define _LedPannelsManager_H_

#include "BeaconDetectionModel.h"
#include <Arduino.h>


class LedPannelsManager {
public:
	LedPannelsManager();
	void sendDetectedBeacons(BeaconDetectionModel beaconDetectionModel);

private:
	void write(const uint8_t *data, size_t quantity);
};


#endif //_LedPannelsManager_H_
