
#ifndef _BeaconDetectionStableHolder_H_
#define _BeaconDetectionStableHolder_H_

#include "BeaconDetectionModel.h"

/**
 * Keep 2 versions of detection data so interruptions (I2C reads can get a stable version)
 */
class BeaconDetectionStableHolder {

public:
	BeaconDetectionStableHolder();

	/** returns where to write */
	BeaconDetectionModel& startWrite();

	/** End of writing -> commit data as stable */
	void commit();

	/** Return the last stable version */
	BeaconDetectionModel& getStable();

private:
	BeaconDetectionModel buf[2];
	int stableBufIdx; // 0 or 1

	int getWriteBufIdx();

	//void log();
};


#endif //_BeaconDetectionStableHolder_H_
