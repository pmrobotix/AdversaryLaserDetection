// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _AdversaryLaserDetection_H_
#define _AdversaryLaserDetection_H_
#include <Arduino.h>
#include "MeanVariance.h"
#include "LedPanelsManager.h"
#include "RawDetectionData.h"
#include "DetectionDataProcessor.h"
#include "BeaconDetectionStableHolder.h"

#include <AFMotor.h>
#include <Wire.h>

//end of add your includes here


//add your function definitions for the project AdversaryLaserDetection here

void interruptRotationTick();
void interruptLaser();

void readFromStableBuffer(RawDetectionData &dest);
void writeToStableBuffer();
void logRawDetectionData(RawDetectionData buf);
void onBeaconDetectionDataRequested();

//Do not add code below this line
#endif /* _AdversaryLaserDetection_H_ */
