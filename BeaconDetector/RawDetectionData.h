
#ifndef _RawDetectionData_H_
#define _RawDetectionData_H_

//TODO rename/move
enum DEBUG_INFO {
	DEBUG_INFO_BEACON_OVERFLOW,
	DEBUG_INFO_LAST_ONE_SEQUENCE,
	DEBUG_INFO_UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE,

	_DEBUG_INFO_COUNT // count of business elements in the enum
};

/** Much greater than actual beacon count in order to avoid buffer overflow */
const unsigned int MAX_NBR_OF_BEACONS = 10;

class RawDetectionData {
	public:
	RawDetectionData();

	// volatile required in order be be accessed from interruptions

	void clear() volatile;

	void copyFrom(const volatile RawDetectionData &src) volatile;

	/** return false in case of beacon overflow */
	bool setTD(int beacon, unsigned long value) volatile;

	/** return false in case of beacon overflow */
	bool setTU(int beacon, unsigned long value) volatile;


	// rotation
	/** time in microseconds of previous rotation */
	unsigned long tA;
	/** time in microseconds of last rotation */
	unsigned long tB;

	// laser
	/** time in microseconds of laser signal fall (start of beacon detection) */
	unsigned long tD[MAX_NBR_OF_BEACONS];

	/** time in microseconds of laser signal rise (end of beacon detection) */
	unsigned long tU[MAX_NBR_OF_BEACONS];

	int detectedBeaconCount;

	int dedugInfoCountByType[_DEBUG_INFO_COUNT];
};


#endif //_RawDetectionData_H_
