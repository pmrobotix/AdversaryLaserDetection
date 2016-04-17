
#include "RawDetectionData.h"

RawDetectionData::RawDetectionData() {
	clear();
}

void RawDetectionData::clear() volatile {
	tA = 0;
	tB = 0;
	for(int i=0; i<MAX_NBR_OF_BEACONS; i++) {
		tD[i] = 0;
		tU[i] = 0;
	}
	detectedBeaconCount = 0;

	for(int i=0; i<_DEBUG_INFO_COUNT; i++) {
		dedugInfoCountByType[i] = 0;
	}
}

void RawDetectionData::copyFrom(const volatile RawDetectionData &src) volatile {
	this->tA = src.tA;
	this->tB = src.tB;
	for(int i=0; i<MAX_NBR_OF_BEACONS; i++) {
		this->tD[i] = src.tD[i];
		this->tU[i] = src.tU[i];
	}
	this->detectedBeaconCount = src.detectedBeaconCount;
	for(int i=0; i<_DEBUG_INFO_COUNT; i++) {
		this->dedugInfoCountByType[i] = src.dedugInfoCountByType[i];
	}
}

/** return false in case of beacon overflow */
bool RawDetectionData::setTD(int beacon, unsigned long value) volatile {
	if(beacon<MAX_NBR_OF_BEACONS) {
		tD[beacon] = value;
		return true;
	} else {
		// overflow
		dedugInfoCountByType[DEBUG_INFO_BEACON_OVERFLOW]++;
		return false;
	}
}

/** return false in case of beacon overflow */
bool RawDetectionData::setTU(int beacon, unsigned long value) volatile {
	if(beacon<MAX_NBR_OF_BEACONS) {
		tU[beacon] = value;
		return true;
	} else {
		// overflow
		dedugInfoCountByType[DEBUG_INFO_BEACON_OVERFLOW]++;
		return false;
	}
}

