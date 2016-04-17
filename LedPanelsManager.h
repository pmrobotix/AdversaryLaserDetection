#ifndef _LedPanelsManager_H_
#define _LedPanelsManager_H_

#include "BeaconDetectionModel.h"
#include <Arduino.h>


class LedPanelsManager {
public:
	LedPanelsManager();
	void sendDetectedBeacons(BeaconDetectionModel beaconDetectionModel);

private:
	void write(const uint8_t *data, size_t quantity);
};


#endif //_LedPanelsManager_H_
