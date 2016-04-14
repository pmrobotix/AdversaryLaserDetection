
#include "LedPannelsManager.h"

// I2C Soft, see http://playground.arduino.cc/Main/SoftwareI2CLibrary
// put this in .ccp and not in .h
#define SCL_PIN 5 // pin 5 of port B = digital pin 11 of Arduino Mega 2560
#define SCL_PORT PORTB
#define SDA_PIN 7 // pin 7 of port B = digital pin 13 of Arduino Mega 2560
#define SDA_PORT PORTB
#define I2C_TIMEOUT	1000 // ms
#include <SoftI2CMaster.h>


LedPannelsManager::LedPannelsManager() {
	if(!i2c_init()) {
		if(Serial) Serial.println(F("WARN: I2C Soft bus is locked."));
	}
}

void LedPannelsManager::sendDetectedBeacons(BeaconDetectionModel bdm) {
	//debug
	if(Serial) {
		Serial.print(F("sizeof(bdm.angleInDeg)="));
		Serial.print(sizeof(bdm.angleInDeg));
		Serial.println(F(" (should be 10)"));
	}

	bdm.serialize();

	uint8_t salveAddr = 0; // broadcast

	i2c_start(salveAddr*2 | I2C_WRITE); //convert addr to 8bit
	write(bdm.serializedBuffer, sizeof(bdm.serializedBuffer));
	i2c_stop();
}

//private methods

void LedPannelsManager::write(const uint8_t *data, size_t quantity) {
	for(size_t i = 0; i < quantity; ++i){
    	i2c_write(data[i]);
    }
}
