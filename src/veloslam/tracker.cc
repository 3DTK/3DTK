#include "veloslam/tracker.h"

Tracker::Tracker(void)
{
	missMatch=false;
	matchClusterID=-1;
	colorIdx=-1;

	missedTime=0;
    trackerID=0;
}

Tracker::~Tracker(void)
{
    TrackerReset();
}

Tracker::Tracker(clusterFeature &glu,double rollAngle):kalmanFilter(glu,rollAngle)  //added by yuanjun
{
	missMatch=false;
	matchClusterID=-1;
	colorIdx=-1;
	missedTime=0;
    trackerID=0;
}

Tracker::Tracker(const Tracker &copyTracker)  //added by yuanjun
{
	missMatch=copyTracker.missMatch;
	matchClusterID=copyTracker.matchClusterID;
	colorIdx=copyTracker.colorIdx;
	statusList=copyTracker.statusList;
	dataList=copyTracker.dataList;
	kalmanFilter=copyTracker.kalmanFilter;
	missedTime=copyTracker.missedTime;
	moveStateList=copyTracker.moveStateList;
    trackerID=copyTracker.trackerID;
}

void Tracker::TrackerReset()
{
	statusList.clear();
	dataList.clear();
	moveStateList.clear();
	return;
}