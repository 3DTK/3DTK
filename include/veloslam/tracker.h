#pragma once
#include "veloslam/veloscan.h"
#include "veloslam/kalmanfilter.h"
#include <deque>

using namespace std;

struct MoveState
{
	ObjectState targetState;
	int frameNo;
};

class Tracker
{
public:
	Tracker(void);
	Tracker(clusterFeature &glu,double rollAngle);//added by yuanjun
	Tracker(const Tracker &copyTracker);//added by yuanjun
	~Tracker(void);
	void TrackerReset();//added by yuanjun

    // for log all the cluster for tracking
	deque<clusterFeature> statusList;
	deque<cluster> dataList;
 	// log the tracker is or not Matched.

	//	bool Matched or not;
	bool missMatch;
	//for log Cluster ID in lastScan.
	int matchClusterID;
	//show different color.
	int colorIdx;
	// for log the lift length trackerID
    int trackerID;

	int missedTime;//added by yuanjun
	KalmanFilter kalmanFilter;//added by yuanjun
	vector<MoveState> moveStateList;//added by yuanjun

    //feature for movement
    float moving_distance;
};

