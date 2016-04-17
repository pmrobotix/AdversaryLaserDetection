
#include "BeaconDetectionStableHolder.h"

//#include <Arduino.h>

BeaconDetectionStableHolder::BeaconDetectionStableHolder() {
	stableBufIdx = 0;
}

BeaconDetectionModel& BeaconDetectionStableHolder::startWrite() {
	return buf[getWriteBufIdx()];
}

void BeaconDetectionStableHolder::commit() {
	stableBufIdx = getWriteBufIdx();
}

BeaconDetectionModel& BeaconDetectionStableHolder::getStable() {
	return buf[stableBufIdx];
}


// private methods

int BeaconDetectionStableHolder::getWriteBufIdx() {
	if(stableBufIdx==0) return 1;
	else return 0;
}

/*
void BeaconDetectionStableHolder::log() {
	Serial.print("stableBufIdx=");
	Serial.print(stableBufIdx);

	Serial.print(" (");
	Serial.print(buf[stableBufIdx].detectedBeaconCount);
	Serial.print(" beacons)");

	Serial.print(" / UNstableBufIdx=");
	Serial.print(getWriteBufIdx());

	Serial.print(" (");
	Serial.print(buf[getWriteBufIdx()].detectedBeaconCount);
	Serial.println(" beacons)");
}
*/
