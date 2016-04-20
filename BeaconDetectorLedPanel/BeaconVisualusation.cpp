
#include "BeaconVisualusation.h"

#include <Rainbowduino.h>

const uint32_t GREEN = 0x00FF00;
const uint32_t ORANGE = 0xFF8C00;

BeaconVisualusation::BeaconVisualusation(int pannelNumber) {
	this->pannelNumber = pannelNumber;
	Rb.init();
}

void BeaconVisualusation::clearPannel() {
	Rb.blankDisplay();
}

bool BeaconVisualusation::drawBeacon(int angle, int distInCm, uint8_t flags,
		bool firstBeaconForThisPanel) {
			
	// display the beacon on the opposite side of the LED panels
	angle = angle + 180;

	// convert from -180/180 to 0/360
	int angle360 = (angle + 360) % 360;

	int column = getColumn(angle360);
	if(column == -1) return false;

	// on led panels:
	// 0,0 is at top left; X is vertical
	// 7,7 is bottom right
	int columnOfPanel = 7 - column;

	unsigned int heigth = getHeight(distInCm); // 0->7

	uint32_t color = getColor(flags);

	if(firstBeaconForThisPanel) clearPannel();
	//Rb.drawLine(7-heigth, columnOfPanel, 7, columnOfPanel, random(0xFFFFFF));
	Rb.drawLine(7-heigth, columnOfPanel, 7, columnOfPanel, color);

	return true;
}


// private methods

int BeaconVisualusation::getColumn(int angle360) {
	int panel;

	if(angle360 < 135) panel=1;
	else if(angle360 < 225) panel=2;
	else panel=3;

	if(panel != this->pannelNumber) return -1;

	//no front panel so show it on side panels
	if(angle360 < 45) angle360 = 45;
	if(angle360 > 314) angle360 = 314;

	float angleInPanel = angle360 + 45 - panel * 90;
	float colIndex = angleInPanel/11.25; // ready to truncate (0 to 7)
	int colIndexRounded = (int) colIndex;

	return colIndexRounded;
}

unsigned int BeaconVisualusation::getHeight(uint8_t dist) {
	float d; // 0 to 99

	if(dist<40) dist=40;
	else if(dist>139) dist=139;

	dist=dist-40;
	d=(float) dist;

	float height = d / 12.5; // ready to truncate (0 to 7)
	int heightInt = height;
	if(heightInt<0) heightInt = 0;
	return heightInt;
}

uint32_t BeaconVisualusation::getColor(uint8_t flags) {
	if((flags & BeaconDetectionModel::FLAG_BLIND_SPOT)
			== BeaconDetectionModel::FLAG_BLIND_SPOT) {
		return ORANGE;
	} else {
		return GREEN;
	}
}
