#include <Rainbowduino.h>
#include <Wire.h>

#include "BeaconDetectionModel.h"
#include "BeaconVisualusation.h"

const byte PANEL_NUMBER = 3;
const byte MY_I2C_ADDRESS = PANEL_NUMBER;

//unsigned int loopsWithoutBeaconForThisPanel = 0;
bool showEye = true;
int eyeLoops = 0;

BeaconDetectionModel beaconDetectionModel;
BeaconVisualusation beaconVisualusation(PANEL_NUMBER); // panel number

/*
void drawWait() {
	Rb.blankDisplay();
	Rb.drawChar('W', 0, 1, random(0xFFFFFF)); // W for waiting
}
*/

void setup() {
	//Rb.init(); -> in BeaconVisualusation

	Wire.begin(MY_I2C_ADDRESS);                // join i2c bus with address...

	TWAR = (MY_I2C_ADDRESS << 1) | 1;  // enable broadcasts to be received

	Wire.onReceive(receiveEvent); // register event
	Serial.begin(9600);           // start serial for output

	//drawWait();
	initEye();
}

void loop() {
	Serial.println("loop");

	// log last beacon received (console unavailable in interruption)
	beaconDetectionModel.logToConsole();

	if(showEye) {
		if(eyeLoops>10) {
			initEye();
			eyeLoops=0;
		}
		eyeLoop();
		eyeLoops++;
	}
	//if(loopsWithoutBeaconForThisPanel > 0) eyeLoop();
	//loopsWithoutBeaconForThisPanel++;

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
				firstBeaconForThisPanel)) {

			forThisPanel = true;
			firstBeaconForThisPanel = false;
		}
	}
	
	showEye = ! forThisPanel;
}
