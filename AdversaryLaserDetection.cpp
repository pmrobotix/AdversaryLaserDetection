
#include "AdversaryLaserDetection.h"
#include "MeanVariance.h"

#include <AFMotor.h>
#include <Wire.h>

// I2C Soft, see http://playground.arduino.cc/Main/SoftwareI2CLibrary

// *** ARDUINO MEGA ***
#define SCL_PIN 5 // pin 5 of port B = digital pin 11 of Arduino Mega 2560
#define SCL_PORT PORTB
#define SDA_PIN 7 // pin 7 of port B = digital pin 13 of Arduino Mega 2560
#define SDA_PORT PORTB
#define I2C_TIMEOUT	1000 // ms
#include <SoftI2CMaster.h>

// *** ARDUINO DEUMILANOVE ***
/*
//#define SCL_PIN 3 // pin 3 of port B = digital pin 11 of duemilanove
#define SCL_PIN 1 // pin 1 of port B = digital pin 9 of duemilanove
#define SCL_PORT PORTB
//#define SDA_PIN 5 // pin 5 of port B = digital pin 13 of duemilanove
//#define SDA_PIN 2 // pin 2 of port B = digital pin 10 of duemilanove

// * avec lib AFMotor
// (scl/sda) 9/10, 9/11 :OK
// (scl/sda) 9/12 :HS
// 11 : OK
// 12,13 : HS

// * sans lib AFMotor
// (scl/sda) 9/12 :OK
// (scl/sda) 9/13 :HS

// * sans lib AFMotor, slow mode
// (scl/sda) 9/12 :OK
// (scl/sda) 9/13 :HS

#define SDA_PIN 5 //13
#define SDA_PORT PORTB
#define I2C_TIMEOUT	1000 // ms
//#define I2C_SLOWMODE 1
#include <SoftI2CMaster.h>
*/


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


// calibration
float betaHist[10];
float beatMoy10;
float betaHistIndex;

void setup() {
	Serial.begin(9600);           // set up Serial library at 9600 bps
	Serial.println(F("AdversaryLaserDetection console!"));

	//Soft i2c
	if(!i2c_init()) {
		Serial.println(F("WARN: I2C Soft bus is locked."));
	}

	// standard I2C
	Wire.begin(); // join i2c bus (address optional for master)


	// pins mode
	pinMode(ROTATION_PIN, INPUT_PULLUP);	// rotation 'tick' (0 = detection)
	pinMode(LASER_PIN, INPUT_PULLUP);		// laser (0 = detection)
	//pinMode(13, OUTPUT); 					// led => soft i2c

	// turn on motor
	motor.setSpeed(200);
	motor.run(RELEASE);

	motor.run(FORWARD);
	// start motor (will not always start if start speed < 120)
	// don't even turn properly if < 75 rpm
	motor.setSpeed(120);

	delay(1000);
	// 75 rpm allows to detect up to 2.5 meters
	// but at height, a beacon is seen only id 38cm < dist < 140cm
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


void sendToI2CSlave(int beaconCount) {
	  Wire.beginTransmission(0); // transmit to device x (0 for broadcast)
	  Wire.write(beaconCount);              // sends one byte
	  Wire.endTransmission();    // stop transmitting
}

// return false in case of error
boolean sendToI2CSoftSlave(int beaconCount) {
	uint8_t salveAddr = 0;
	if (!i2c_start(salveAddr | I2C_WRITE)) {
		return false;
	}
	//i2c_start_wait(salveAddr*2 | I2C_WRITE); //convert addr to 8bit
	if (!i2c_write((uint8_t) beaconCount)) {
		return false;
	}
	i2c_stop();
	return true;
}


MeanVariance calibrationMeanVariance;
void calibration(float newBeta) {
	calibrationMeanVariance.addValue(newBeta);
	Serial.print(F("** (calibration) Beta: "));

	Serial.print(F(" last="));
	Serial.print(newBeta, 4);

	Serial.print(F(" histCount="));
	Serial.print(calibrationMeanVariance.getHistCount());

	Serial.print(F(" mean="));
	Serial.print(calibrationMeanVariance.getMean(), 4);

	Serial.print(F(" variance="));
	Serial.print(calibrationMeanVariance.getVariance(), 6);

	Serial.println();
}

/** Dist = from the center of the detector to the center of the beacon  (in meters) */
float getDist(unsigned long t1, unsigned long t2, unsigned long tA, unsigned long tB) {
	unsigned long start = micros();

	//from calibration & external data processing
	const float a=-0.018011;
	const float b=0.075850;

	unsigned long t2_t1 = t2-t1;
	unsigned long tB_tA = tB-tA;

	float betaRad = (((float) t2_t1) / ((float) tB_tA)) * PI;

	// Dist = r - x + b ./ (2.*tan(betaRad)-a)
	// x goes from 0.005 to 0.02 m => set to 0.01m / r = 0.04m

	float dist = 0.03f + b / (2*tan(betaRad)-a);

	unsigned long end = micros();

	Serial.print(F("dist computation took "));
	Serial.print(end-start);
	Serial.println(F(" MICRO seconds."));
	// => less than 0.2 ms on ariduno mega

	return dist;
}

void logStableBuffer() {
	LaserDetectionBuffer buf;
	readFromStableBuffer(buf);

	Serial.println(F("\n*****************"));
	Serial.print(F("DEBUG_INFO counts => BEACON_OVERFLOW:"));
	Serial.print(buf.dedugInfoCountByType[DEBUG_INFO_BEACON_OVERFLOW]);

	Serial.print(F("  DEBUG_INFO_LAST_ONE_SEQUENCE:"));
	Serial.print(buf.dedugInfoCountByType[DEBUG_INFO_LAST_ONE_SEQUENCE]);

	Serial.print(F("  UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE:"));
	Serial.println(buf.dedugInfoCountByType[DEBUG_INFO_UNEXPECTED_DETECTION_IN_LAST_ONE_SEQUENCE]);


	Serial.print(F("tA="));
	Serial.print(buf.tA);
	Serial.print(F("   tB="));
	Serial.print(buf.tB);
	Serial.print(F("   =>   tB-tA="));
	Serial.println(buf.tB - buf.tA);

	Serial.print(F("detectedBeaconCount="));
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
		Serial.print(beta, 4); // 4 digits

		Serial.println();


		//calibration(beta);

		float dist = getDist(t1, t2, buf.tA, buf.tB);
		Serial.print(" dist=");
		Serial.print(dist, 4);
	}

	//standard I2C
	sendToI2CSlave(buf.detectedBeaconCount);

	// soft 2ic (led)
	sendToI2CSoftSlave(buf.detectedBeaconCount);

}
