
#include <Arduino.h>
#include <Wire.h>
#include "BeaconDetectionModel.h"

const uint8_t BEACON_DETECTOR_I2C_ADDRESS = 99;


BeaconDetectionModel beaconDetectionData;

bool getBeaconDetectionData(BeaconDetectionModel& target) {
	uint8_t requestedSize = sizeof(target.serializedBuffer);
	Wire.requestFrom(BEACON_DETECTOR_I2C_ADDRESS, requestedSize);

	uint8_t i=0;
	while (Wire.available()) { // slave may send less than requested
		target.serializedBuffer[i] =  Wire.read();
		i++;
	}
	if(i==requestedSize) {
		target.deserialize();
		return true;
	} else {
		if(Serial) {
			Serial.print(F("WARN: getBeaconDetectionData: receivedSize="));
			Serial.print(i);
			Serial.print(F(" requestedSize="));
			Serial.println(requestedSize);
		}
		return false;
	}
}

void setup() {
	Wire.begin();     // address not required for master

	Serial.begin(9600);
	Serial.println(F("AdversaryLaserDetectionMasterSimulator console!"));
}

void loop() {
	bool ok = getBeaconDetectionData(beaconDetectionData);
	if(ok) {
		Serial.print(F("\n **** beaconDetectionData: "));
		beaconDetectionData.logToConsole();
	}
	delay(1000);
}

