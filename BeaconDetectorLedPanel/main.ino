#include <Rainbowduino.h>
#include <Wire.h>

#include "BeaconDetectionModel.h"
#include "BeaconVisualusation.h"

//__________________________
// panels: 1=right / 2=back / 3=left
const byte PANEL_NUMBER = 1;
//__________________________


const byte MY_I2C_ADDRESS = PANEL_NUMBER;

const bool DEBUG = false;

bool showEye = true;
int eyeLoops = 0;

BeaconDetectionModel beaconDetectionModel;
BeaconVisualusation beaconVisualusation(PANEL_NUMBER); // panel number

void setup() {
	//Rb.init(); -> in BeaconVisualusation

	Wire.begin(MY_I2C_ADDRESS);                // join i2c bus with address...

	TWAR = (MY_I2C_ADDRESS << 1) | 1;  // enable broadcasts to be received

	Wire.onReceive(receiveEvent); // register event
	Serial.begin(9600);           // start serial for output

	Serial.print(F("BeaconDetectorLedPannel Console! - "));
	Serial.print(F("PANEL_NUMBER="));
	Serial.print(PANEL_NUMBER);
	Serial.print(F(" MY_I2C_ADDRESS="));
	Serial.print(MY_I2C_ADDRESS);
	Serial.println();

	initEye();
}

void loop() {
	// log last beacon received (console unavailable in interruption)
	if(DEBUG) beaconDetectionModel.logToConsole();

	if(showEye) {
		if(eyeLoops>10) {
			initEye();
			eyeLoops=0;
		}
		eyeLoop();
		eyeLoops++;
	}

	delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
	// !! serial not available (in interrupt) !!

	int i = 0;
	while (Wire.available()) {
		beaconDetectionModel.serializedBuffer[i] =  Wire.read();
		i++;
	}

	beaconDetectionModel.deserialize();
	onBeaconDetectionModelReceived();
}

void onBeaconDetectionModelReceived() {
	bool forThisPanel = false;
	bool firstBeaconForThisPanel = true;

	for(int i=0; i<beaconDetectionModel.detectedBeaconCount; i++) {
		if(beaconVisualusation.drawBeacon(
				beaconDetectionModel.angleInDeg[i],
				beaconDetectionModel.distToBeaconCenterInCm[i],
				beaconDetectionModel.flags[i],
				firstBeaconForThisPanel)) {

			forThisPanel = true;
			firstBeaconForThisPanel = false;
		}
	}
	
	showEye = ! forThisPanel;
}
