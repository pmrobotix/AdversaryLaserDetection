#ifndef _LedPanelsManager_H_
#define _LedPanelsManager_H_

#include "BeaconDetectionModel.h"
#include <Arduino.h>


class LedPanelsManager {
public:
	LedPanelsManager(unsigned long timeBetweenSendToLedPanelsMs);
	void sendDetectedBeaconsIfRequired(BeaconDetectionModel beaconDetectionModel);

private:
	void write(const uint8_t *data, size_t quantity);
	bool isTimeToSend();

	unsigned long timeBetweenSendToLedPanelsMs;
	unsigned long lastSentToLedPanelsMs;
};


#endif //_LedPanelsManager_H_
