
#ifndef _DetectionDataProcessor_H_
#define _DetectionDataProcessor_H_

#include "RawDetectionData.h"
#include "BeaconDetectionModel.h"


class DetectionDataProcessor {

public:
	void processRawData(RawDetectionData &inTimings, BeaconDetectionModel &outModel);

private:
	/** Dist = from the center of the detector to the center of the beacon  (in meters) */
	float getDistInM(unsigned long t1, unsigned long t2, unsigned long tB_tA);

	float getAngleInDeg(unsigned long t1, unsigned long t2, unsigned long tB_tA);

	bool isInBlindSpot(unsigned long t1, unsigned long t2, unsigned long tB_tA);
	bool isInBlindSpot(float a);


	unsigned long tD[MAX_NBR_OF_BEACONS];

	/** time in microseconds of laser signal rise (end of beacon detection) */
	unsigned long tU[MAX_NBR_OF_BEACONS];

	int detectedBeaconCount;
};


#endif //_DetectionDataProcessor_H_
