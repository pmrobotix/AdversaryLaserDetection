//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2016-04-17 16:15:24

#include "Arduino.h"
#include <Rainbowduino.h>
#include <Rainbowduino.h>
#include <Wire.h>
#include "BeaconDetectionModel.h"
#include "BeaconVisualusation.h"
void initEye() ;
int convert(float time, int base_value, int target_value);
void eyeLoop();
void setNewPosition(int newX, int newY, int newSevere) ;
void setEyeBrow(int xx[8]) ;
void setPxl(int x, int y, uint32_t color) ;
void redraw() ;
void doBlink() ;
void lookAt(int x, int y, int newSevere) ;
void setup() ;
void loop() ;
void receiveEvent(int howMany) ;
void onBeaconDetectionModelReceived() ;


#include "eye.ino"
#include "main.ino"
