#pragma once

#include <list>
#include <vector>
#include "slam6d/scan.h"
#include "veloslam/tracker.h"
 
using namespace std;

struct ClusterStatus
{
	bool FilterRet;
	bool Matched;
};


class TrackerManager
{
public:
	TrackerManager(void);

	~TrackerManager(void);

	int Init(void);

	int getNumberofTracker(void);

	int HandleScan(VeloScan& scanRef);

	int DrawScanCluster(VeloScan& scanRef);

	int DrawTrackers(VeloScan& scanRefs);

	int DrawTrackersMovtion(VeloScan& scanRef1,  VeloScan& scanRef2);

	int DrawTrackersMovtion_Long(vector <VeloScan *> allScans);

   int DrawTrackersMovtion_Long_Number(vector <Scan *> allScans, int n);

	int DrawTrackersMovtion_Long_Number_All(vector <Scan *> allScans, int n);

	int DrawTrackersContrailAfterFilted(vector <Scan *> allScans);

	int ClassifiyTrackersObjects(vector <Scan *> allScans, int currentNO ,int windowsize);

	static bool TrackerFilter(clusterFeature &glu, cluster &gluData);

	vector<ClusterStatus> clusterStatus;

	void TrackerManagerReset(); //added by yuanjun

	void GetTwoScanRoll(Scan *CurrentScan,Scan *preScan);//added by yuanjun

private:
	int UpdateTrackers(VeloScan& scanRef);

	int AddTrackers(VeloScan& scanRef);

	int MatchTrackers(VeloScan& scanRef,Tracker& tracker,float kg);

	int FilterObject(VeloScan& scanRef);
	
	list<Tracker> tracks;

	int colorIdx;

	double delta_Theta[3],delta_Pos[3],rollAngle;//added by yuanjun

	VeloScan *preScan; //added by yuanjun
};

