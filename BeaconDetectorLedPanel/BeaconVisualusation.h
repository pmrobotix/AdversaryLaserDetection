
#ifndef _BeaconVisualusation_H_
#define _BeaconVisualusation_H_

#include "BeaconDetectionModel.h"

class BeaconVisualusation {

public:
	/** panels: 0=right / 1=back / 2=left */
	BeaconVisualusation(int pannelNumber);

	void clearPannel();

	/** return true if the beacon was for this panel */
	bool drawBeacon(int angle, int distInCm, bool firstBeaconForThisPanel);

private:
	int pannelNumber;

	/**
	 * return the number of the led column (0 to 7) associated to the angle (0 for small angles)
	 * or -1 if not in this panel
	 */
	int getColumn(int angle360);

	/**
	 * convert 40cm->140cm to 0->7
	 * >53cm => 1 / > 128cm => 7
	 */
	unsigned int getHeight(uint8_t distInCm);
};


#endif //_BeaconVisualusation_H_