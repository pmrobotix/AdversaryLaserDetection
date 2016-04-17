
#include "BeaconDetectionModel.h"

#include <string.h>
#include <Arduino.h>

void BeaconDetectionModel::serialize() {
	nextInBuf = serializedBuffer;

	appendToBuf(&detectedBeaconCount, sizeof(detectedBeaconCount));
	appendToBuf(distToBeaconCenterInCm, sizeof(distToBeaconCenterInCm));
	appendToBuf(angleInDeg, sizeof(angleInDeg));
	appendToBuf(flags, sizeof(flags));
}

void BeaconDetectionModel::deserialize() {
	nextInBuf = serializedBuffer;

	readFromBuf(&detectedBeaconCount, sizeof(detectedBeaconCount));
	readFromBuf(distToBeaconCenterInCm, sizeof(distToBeaconCenterInCm));
	readFromBuf(angleInDeg, sizeof(angleInDeg));
	readFromBuf(flags, sizeof(flags));
}


void BeaconDetectionModel::logToConsole() {
	Serial.println(F("** BeaconDetectionModel::logToConsole **"));

	Serial.print(F("detectedBeaconCount="));
	Serial.println(detectedBeaconCount);

	for(int i=0; i<detectedBeaconCount; i++) {
		Serial.print("beacon ");
		Serial.print(i);

		Serial.print(": angleInDeg=");
		Serial.print(angleInDeg[i]);

		Serial.print(" distToBeaconCenterInCm=");
		Serial.print(distToBeaconCenterInCm[i]);

		Serial.print(" blindSpot=");
		Serial.print((flags[i] & FLAG_BLIND_SPOT)==FLAG_BLIND_SPOT);

		Serial.println();
	}
}


//private methods

void BeaconDetectionModel::appendToBuf(void *data, size_t size) {
	memcpy(nextInBuf, data, size);
	nextInBuf += size;
}

void BeaconDetectionModel::readFromBuf(void *dest, size_t size) {
	memcpy(dest, nextInBuf, size);
	nextInBuf += size;
}
