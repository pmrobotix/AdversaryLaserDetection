
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
};


#endif //_DetectionDataProcessor_H_
