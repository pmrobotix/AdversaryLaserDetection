
#ifndef _MeanVariance_H_
#define _MeanVariance_H_

class MeanVariance {

private:
	static const int MAX_HIST_COUNT = 100;
	float hist[MAX_HIST_COUNT];
	float mean;
	int nextHistIndex=0;
	int histCount=0;

public:
	float getMean();
	float getVariance();
	int getHistCount();
	void addValue(float newValue);
};

#endif
