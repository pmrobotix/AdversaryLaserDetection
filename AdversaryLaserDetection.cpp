// Do not remove the include below
#include "AdversaryLaserDetection.h"

#include <AFMotor.h>

AF_DCMotor motor(4);

/** Acquisition mode (internal) */
enum ACQ_MODE {
	ACQ_MODE_STD,
	ACQ_MODE_DISABLED,
	ACQ_MODE_LAST_ONE,

	/** end of the last_one/disabled rotation */
	ACQ_MODE_IGNORE_NEXT_LASER_UP
};

/** Much greater than actual beacon count in order to avoid buffer overflow */
const unsigned int MAX_NBR_OF_BEACONS = 10;

const uint8_t ROTATION_PIN = 2;
const uint8_t LASER_PIN = 3;

enum DEBUG_INFO {
	DEBUG_INFO_BEACON_OVERFLOW,
	DEBUG_INFO_LAST_ONE_SEQUENCE,
	DEBUG_INFO_UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE,

	_DEBUG_INFO_COUNT // count of business elements in the enum
};


class LaserDetectionBuffer {
	public:
	//LaserDetectionBuffer();

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

	LaserDetectionBuffer() {
		clear();
	}

	void clear() volatile {
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

	void copyFrom(const volatile LaserDetectionBuffer &src) volatile {
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
	boolean setTD(int beacon, unsigned long value) volatile {
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
	boolean setTU(int beacon, unsigned long value) volatile {
		if(beacon<MAX_NBR_OF_BEACONS) {
			tU[beacon] = value;
			return true;
		} else {
			// overflow
			dedugInfoCountByType[DEBUG_INFO_BEACON_OVERFLOW]++;
			return false;
		}
	}
};


// Detection buffers
/** Current acquisition, do not read */
volatile LaserDetectionBuffer workBuffer;
volatile LaserDetectionBuffer lastStableBuffer;
volatile boolean stableBufReadInProgress = false;
volatile ACQ_MODE acqMode = ACQ_MODE_STD; // init, do not change

void setup() {
	Serial.begin(9600);           // set up Serial library at 9600 bps
	Serial.println("AdversaryLaserDetection console!");

	// pins mode
	pinMode(ROTATION_PIN, INPUT_PULLUP);	// rotation 'tick' (0 = detection)
	pinMode(LASER_PIN, INPUT_PULLUP);		// laser (0 = detection)
	pinMode(13, OUTPUT); 					// led

	// turn on motor
	motor.setSpeed(200);
	motor.run(RELEASE);

	motor.run(FORWARD);
	motor.setSpeed(120);

	// interrupts
	attachInterrupt(digitalPinToInterrupt(ROTATION_PIN), interruptRotationTick, FALLING);
	attachInterrupt(digitalPinToInterrupt(LASER_PIN), interruptLaser, CHANGE);
}

void loop() {
	logStableBuffer();
	delay(1000);
}

void interruptRotationTick() {
	int laser = digitalRead(LASER_PIN);
	unsigned long t = micros();

	workBuffer.tB = t;

	if(laser==0 && acqMode==ACQ_MODE_STD) {
		// a beacon is currently detected, overlap on next rotation
		acqMode = ACQ_MODE_LAST_ONE;
		workBuffer.dedugInfoCountByType[DEBUG_INFO_LAST_ONE_SEQUENCE]++;
		return;
	}

	writeToStableBuffer();

	workBuffer.clear();
	workBuffer.tA = t;
	if(acqMode==ACQ_MODE_DISABLED) {
		acqMode = ACQ_MODE_IGNORE_NEXT_LASER_UP;
	} else {
		acqMode = ACQ_MODE_STD;
	}

}


void interruptLaser() {
	int laser = digitalRead(LASER_PIN);
	unsigned long t = micros();

	if(acqMode==ACQ_MODE_DISABLED) return;

	if(acqMode==ACQ_MODE_LAST_ONE) {
		acqMode = ACQ_MODE_DISABLED;
		if(laser == 0) {
			// should not happen
			workBuffer.dedugInfoCountByType[DEBUG_INFO_UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE]++;
		}
	}

	if(laser==1) {
		if(acqMode != ACQ_MODE_IGNORE_NEXT_LASER_UP) {
			boolean noOverflow = workBuffer.setTU(workBuffer.detectedBeaconCount, t);
			if(noOverflow) workBuffer.detectedBeaconCount++;
		} else {
			acqMode = ACQ_MODE_STD;
		}
	} else {
		workBuffer.setTD(workBuffer.detectedBeaconCount, t);
	}
}


void writeToStableBuffer() {
	if( ! stableBufReadInProgress) {
		lastStableBuffer.copyFrom(workBuffer);
	}
}

/** copy latest stable buffer to dest */
void readFromStableBuffer(LaserDetectionBuffer &dest) {
	stableBufReadInProgress = true;
	dest.copyFrom(lastStableBuffer);
	stableBufReadInProgress = false;
}

void logStableBuffer() {
	LaserDetectionBuffer buf;
	readFromStableBuffer(buf);

	Serial.println("\n*****************");
	Serial.print("DEBUG_INFO counts => BEACON_OVERFLOW:");
	Serial.print(buf.dedugInfoCountByType[DEBUG_INFO_BEACON_OVERFLOW]);

	Serial.print("  DEBUG_INFO_LAST_ONE_SEQUENCE:");
	Serial.print(buf.dedugInfoCountByType[DEBUG_INFO_LAST_ONE_SEQUENCE]);

	Serial.print("  UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE:");
	Serial.println(buf.dedugInfoCountByType[DEBUG_INFO_UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE]);


	Serial.print("tA=");
	Serial.print(buf.tA);
	Serial.print("   tB=");
	Serial.print(buf.tB);
	Serial.print("   =>   tB-tA=");
	Serial.println(buf.tB - buf.tA);

	Serial.print("detectedBeaconCount=");
	Serial.println(buf.detectedBeaconCount);

	for(int i=0; i<buf.detectedBeaconCount; i++) {
		Serial.print("beacon ");
		Serial.print(i);

		Serial.print(": tD=");
		Serial.print(buf.tD[i]);
		Serial.print(" tD-tA=");
		Serial.print(buf.tD[i]-buf.tA);

		Serial.print("   tU=");
		Serial.print(buf.tU[i]);
		Serial.print(" tU-tA=");
		Serial.print(buf.tU[i]-buf.tA);

		Serial.print("   =>   tU-tD=");
		unsigned long deltaTBeacon = buf.tU[i] - buf.tD[i];
		Serial.print(deltaTBeacon);

		unsigned long t1 = buf.tD[i]-buf.tA;
		unsigned long t2 = buf.tU[i]-buf.tA;
		unsigned long t2_t1 = t2-t1;
		unsigned long tB_tA = buf.tB-buf.tA;

		float alpha = 180.0f *((float) t2+t1) / ((float) tB_tA);
		Serial.print("   alpha (without calibration)=");
		Serial.print(alpha);

		float beta = 180.0f * ((float) t2_t1) / ((float) tB_tA);

		// add calibration
		const float alphaRef = -193.0f;
		float alphaB = alpha + alphaRef;

		// TODO angle mort

		Serial.print(" alphaB=");
		Serial.print(alphaB);

		Serial.print(" beta=");
		Serial.print(beta);

		Serial.println();
	}
}
