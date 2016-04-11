#include "MeanVariance.h"

void MeanVariance::addValue(float newValue) {
	hist[nextHistIndex] = newValue;
	if(histCount < MAX_HIST_COUNT) histCount++;
	nextHistIndex = (nextHistIndex+1) % MAX_HIST_COUNT;
}

float MeanVariance::getMean() {
	float sum = 0;
	for(int i=0; i<histCount; i++) {
		sum += hist[i];
	}
	return sum / histCount;
}

float MeanVariance::getVariance() {
	float mean = getMean();
	float tmp = 0;
	for(int i=0; i<histCount; i++) {
		tmp += (hist[i] - mean) * (hist[i] - mean);
	}
	return tmp / histCount;
}

int MeanVariance::getHistCount() {
	return histCount;
}
