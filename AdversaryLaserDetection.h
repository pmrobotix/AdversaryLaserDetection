// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _AdversaryLaserDetection_H_
#define _AdversaryLaserDetection_H_
#include "Arduino.h"
//add your includes for the project AdversaryLaserDetection here


//end of add your includes here


//add your function definitions for the project AdversaryLaserDetection here

void interruptRotationTick();
void interruptLaser();

void writeToStableBuffer();
void logStableBuffer();

//Do not add code below this line
#endif /* _AdversaryLaserDetection_H_ */
