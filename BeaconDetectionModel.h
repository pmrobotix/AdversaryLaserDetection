
#ifndef _BeaconDetectionModel_H_
#define _BeaconDetectionModel_H_

#include <stddef.h>
#include <stdint.h>

class BeaconDetectionModel {

public:
	static const size_t MAX_NBR_OF_BEACONS = 5;

	// ** data to be serialized
	uint8_t detectedBeaconCount;
	uint8_t distToBeaconCenterInCm[MAX_NBR_OF_BEACONS];
	int16_t angleInDeg[MAX_NBR_OF_BEACONS];
	uint8_t flags[MAX_NBR_OF_BEACONS]; // future use

	uint8_t serializedBuffer[ sizeof(detectedBeaconCount)
						  + sizeof(distToBeaconCenterInCm)
						  + sizeof(angleInDeg)
						  + sizeof(flags) ];

	void serialize();
	void deserialize();
	void logToConsole();

private:
	void* nextInBuf;
	void appendToBuf(void *data, size_t size);
	void readFromBuf(void *dest, size_t size);
};


#endif //_BeaconDetectionModel_H_
