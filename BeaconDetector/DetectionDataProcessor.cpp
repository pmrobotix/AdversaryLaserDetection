#include "DetectionDataProcessor.h"

#include <Arduino.h>

void DetectionDataProcessor::processRawData(RawDetectionData &inTimings, BeaconDetectionModel &outModel) {

	outModel.detectedBeaconCount = inTimings.detectedBeaconCount;
	unsigned long tB_tA = inTimings.tB-inTimings.tA;

	for(int i=0; i<inTimings.detectedBeaconCount; i++) {
		unsigned long t1 = inTimings.tD[i]-inTimings.tA;
		unsigned long t2 = inTimings.tU[i]-inTimings.tA;

		outModel.angleInDeg[i] = getAngleInDeg(t1, t2, tB_tA); // type conversion!

		float distInCm = getDistInM(t1, t2, tB_tA) * 100.0;
		if(distInCm > 255.0) distInCm = 255.0;
		outModel.distToBeaconCenterInCm[i] = distInCm; // type conversion!

		if(isInBlindSpot(t1, t2, tB_tA)) {
			outModel.flags[i] = outModel.FLAG_BLIND_SPOT;
		} else {
			outModel.flags[i] = 0;
		}
	}
}


// private methods

float DetectionDataProcessor::getDistInM(unsigned long t1, unsigned long t2, unsigned long tB_tA) {

	//unsigned long start = micros();

	//from calibration & external data processing
	const float a=-0.018011;
	const float b=0.075850;

	unsigned long t2_t1 = t2-t1;

	float betaRad = (((float) t2_t1) / ((float) tB_tA)) * PI;

	// Dist = r - x + b ./ (2.*tan(betaRad)-a)
	// x goes from 0.005 to 0.02 m => set to 0.01m / r = 0.04m

	float dist = 0.03f + b / (2*tan(betaRad)-a);

	//unsigned long end = micros();

	/*
	if(DEBUG) Serial.print(F("dist computation took "));
	if(DEBUG) Serial.print(end-start);
	if(DEBUG) Serial.println(F(" MICRO seconds."));
	// => less than 0.2 ms on ariduno mega
	 */

	return dist;
}

float DetectionDataProcessor::getAngleInDeg(unsigned long t1, unsigned long t2, unsigned long tB_tA) {
	unsigned long t2_t1 = t2-t1;

	float alpha = 180.0f *((float) t2+t1) / ((float) tB_tA);

	// add calibration
	const float alphaRef = -193.0f;
	float alphaB = alpha + alphaRef;

	return alphaB;
}

bool DetectionDataProcessor::isInBlindSpot(unsigned long t1, unsigned long t2, unsigned long tB_tA) {
	float alpha1 = 360.0 * ((float)t1) / ((float)tB_tA); // angle associated to t1/tD (start of detection) (without calibration)
	float alpha2 = 360.0 * ((float)t2) / ((float)tB_tA); // angle associated to t2/tU (end of detection) (without calibration)

	return isInBlindSpot(alpha1) || isInBlindSpot(alpha2);
}

// a is angleWithoutCalibration
bool DetectionDataProcessor::isInBlindSpot(float a) {
	return a>8.0 && a<22.5;
}
