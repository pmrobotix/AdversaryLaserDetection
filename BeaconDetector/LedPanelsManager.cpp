
#include "LedPanelsManager.h"

// *** I2C Soft, see http://playground.arduino.cc/Main/SoftwareI2CLibrary
// put this in .ccp and not in .h
/*
#define SCL_PIN 5 // pin 5 of port B = digital pin 11 of Arduino Mega 2560
#define SCL_PORT PORTB
#define SDA_PIN 7 // pin 7 of port B = digital pin 13 of Arduino Mega 2560
#define SDA_PORT PORTB
*/

// Arduino Pro Micro avec ATMega32U4
// see https://www.arduino.cc/en/Hacking/PinMapping32u4
#define SCL_PIN 4 // pin 4 of port B = digital pin 8 of ATMega32U4
#define SCL_PORT PORTB
#define SDA_PIN 5 // pin 5 of port B = digital pin 9 of ATMega32U4
#define SDA_PORT PORTB

#define I2C_TIMEOUT	100 // ms
#include <SoftI2CMaster.h>


LedPanelsManager::LedPanelsManager(unsigned long timeBetweenSendToLedPanelsMs) {
	this->timeBetweenSendToLedPanelsMs = timeBetweenSendToLedPanelsMs;
	this->lastSentToLedPanelsMs = 0;

	if(!i2c_init()) {
		Serial.println(F("WARN: I2C Soft bus is locked."));
	}
}

void LedPanelsManager::sendDetectedBeaconsIfRequired(BeaconDetectionModel bdm) {
	if( ! isTimeToSend() ) return;

	bdm.serialize();

	uint8_t salveAddr = 0; // broadcast

	i2c_start(salveAddr*2 | I2C_WRITE); //convert addr to 8bit
	write(bdm.serializedBuffer, sizeof(bdm.serializedBuffer));
	i2c_stop();

	if(DEBUG) Serial.println(F("Sent to I2C soft."));
}

//private methods

void LedPanelsManager::write(const uint8_t *data, size_t quantity) {
	for(size_t i = 0; i < quantity; ++i){
    	i2c_write(data[i]);
    }
}

bool LedPanelsManager::isTimeToSend() {
	unsigned long now = millis();
	if(now > lastSentToLedPanelsMs + timeBetweenSendToLedPanelsMs) {
		lastSentToLedPanelsMs = now;
		return true;
	} else {
		return false;
	}
}
