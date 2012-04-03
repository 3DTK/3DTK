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

int GetScanID_in_SlidingWindow(int absNO, int current_pos, int  window_size);


class TrackerManager
{
public:
	TrackerManager(void);

	~TrackerManager(void);

	int Init(void);

	int getNumberofTracker(void);

	int HandleScan(VeloScan& scanRef);

  	int ClassifiyTrackersObjects(vector <Scan *> allScans, int currentNO ,int windowsize);

	static bool TrackerFilter(clusterFeature &glu, cluster &gluData);

	void TrackerManagerReset(); //added by yuanjun

//	void GetTwoScanRoll(Scan *CurrentScan, Scan *preScan);//added by yuanjun
	 /////////////////////////////////////////////////////////////////
    int DrawEgoTrajectory();

    int DrawScanCluster(VeloScan& scanRef);

    int DrawTrackers(VeloScan& scanRefs);

    int DrawTrackersMovtion(VeloScan& scanRef1,  VeloScan& scanRef2);

    int DrawTrackersMovtion_Long(vector <VeloScan *> allScans);

    int DrawTrackersMovtion_Long_Number(vector <Scan *> allScans, int n);

    int DrawTrackersMovtion_Long_Number_All(vector <Scan *> allScans, int n);

    int DrawTrackersContrailAfterFilted(vector <Scan *> allScans);
    //////////////////////////////////////////////////////////////////
public:
	int UpdateTrackers(VeloScan& scanRef);

	int AddTrackers(VeloScan& scanRef);

	int MatchTrackers(VeloScan& scanRef,Tracker& tracker,float kg);

    int RemoveNoUsedTracker(VeloScan& scanRef);

	int FilterObject(VeloScan& scanRef);

	list<Tracker> tracks;
	//log the new trackerNo should begin
    int trackerStartID;
    //only for draw the color 
	int colorIdx;
	//log the cluster status such as  	bool FilterRet;	bool Matched;
	vector<ClusterStatus> clusterStatus;  //should move in cluster struct

	double delta_Theta[3],  delta_Pos[3],  rollAngle;//added by yuanjun
//	VeloScan *preScan; //added by yuanjun
};

