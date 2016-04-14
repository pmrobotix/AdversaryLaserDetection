
#include "AdversaryLaserDetection.h"


LedPannelsManager ledPannelsManager;
DetectionDataProcessor detectionDataProcessor;

AF_DCMotor motor(4);


/** Acquisition mode (internal) */
enum ACQ_MODE {
	ACQ_MODE_STD,
	ACQ_MODE_DISABLED,
	ACQ_MODE_LAST_ONE,

	/** end of the last_one/disabled rotation */
	ACQ_MODE_IGNORE_NEXT_LASER_UP
};


const uint8_t ROTATION_PIN = 2;
const uint8_t LASER_PIN = 3;


// Detection buffers
/** Current acquisition, do not read */
volatile RawDetectionData workBuffer;
volatile RawDetectionData lastStableBuffer;
volatile boolean stableBufReadInProgress = false;
volatile ACQ_MODE acqMode = ACQ_MODE_STD; // init, do not change


void setup() {
	Serial.begin(9600);           // set up Serial library at 9600 bps
	Serial.println(F("AdversaryLaserDetection console!"));

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

void processDetectionData() {
	RawDetectionData rawDetectionData;
	readFromStableBuffer(rawDetectionData);
	logRawDetectionData(rawDetectionData);

	BeaconDetectionModel beaconDetectionModel;
	detectionDataProcessor.processRawData(rawDetectionData, beaconDetectionModel);
	beaconDetectionModel.logToConsole();

	ledPannelsManager.sendDetectedBeacons(beaconDetectionModel);
}


void loop() {
	processDetectionData();
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
void readFromStableBuffer(RawDetectionData &dest) {
	stableBufReadInProgress = true;
	dest.copyFrom(lastStableBuffer);
	stableBufReadInProgress = false;
}


void sendToI2CSlave(int beaconCount) {
	  Wire.beginTransmission(0); // transmit to device x (0 for broadcast)
	  Wire.write(beaconCount);              // sends one byte
	  Wire.endTransmission();    // stop transmitting
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


void logRawDetectionData(RawDetectionData buf) {
	Serial.println(F("\n** logRawDetectionData **"));
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

		Serial.print(" alphaB=");
		Serial.print(alphaB);

		Serial.print(" beta=");
		Serial.print(beta, 4); // 4 digits

		Serial.println();


		//calibration(beta);
	}
}

