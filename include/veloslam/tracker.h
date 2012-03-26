#pragma once
#include "veloslam/veloscan.h"
#include "veloslam/KalmanFilter.h"
#include <deque>

using namespace std;

class Tracker
{
public:
	Tracker(void);
	Tracker(clusterFeature &glu,double rollAngle);//added by yuanjun
	Tracker(const Tracker &copyTracker);//added by yuanjun
	~Tracker(void);
	void TrackerReset();//added by yuanjun

	deque<clusterFeature> statusList;
	deque<cluster> dataList;
	bool missMatch;
	bool Matched;
	int matchClusterID;
	int colorIdx;
	int frameNO;
	int missedTime;//added by yuanjun
	KalmanFilter kalmanFilter;//added by yuanjun

};

