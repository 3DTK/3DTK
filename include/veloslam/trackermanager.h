#pragma once

#include <list>
#include <vector>
#include "slam6d/scan.h"
#include "veloslam/tracker.h"

//#define NO_SLIDING_WINDOW

using namespace std;
extern float  constant_static_or_moving;
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

	int HandleScan(VeloScan& scanRef,int trackingAlgo);

  	int ClassifiyTrackersObjects(vector <Scan *> allScans, int currentNO ,int windowsize);

	static bool TrackerFilter(clusterFeature &glu, cluster &gluData);

	void TrackerManagerReset(); //added by yuanjun

	void GetTwoScanRoll(Scan *CurrentScan, Scan *preScan);//added by yuanjun
	 /////////////////////////////////////////////////////////////////
    int DrawEgoTrajectory();

    int DrawTrackersMovtion_Long_Number_All(vector <Scan *> allScans, int n);

    int DrawTrackersContrailAfterFilted(vector <Scan *> allScans);
    //////////////////////////////////////////////////////////////////
public:
    int ListTrackers();

	int UpdateTrackers(VeloScan& scanRef);

    int UpdateClustersPoistioninTrackers();

	int AddTrackers(VeloScan& scanRef);

	int MatchTrackers(VeloScan& scanRef,Tracker& tracker,float kg);

    int CalculateTrackersFeature(vector <Scan *> allScans, int currentNO ,int windowsize);

    int MarkClassifiyTrackersResult(vector <Scan *> allScans, int currentNO ,int windowsize);

    int RemoveNoUsedTracker(VeloScan& scanRef);

	int FilterObject(VeloScan& scanRef);

	CMatrix ConstructCostMatrix(VeloScan &scanRef,int *clusterIndex);

	int MatchTracksWithClusters(VeloScan &scanRef);

	list<Tracker> tracks;
	//log the new trackerNo should begin
    int trackerStartID;
    //only for draw the color
	int colorIdx;
	//log the cluster status such as  	bool FilterRet;	bool Matched;
	vector<ClusterStatus> clusterStatus;  //should move in cluster struct

	double delta_Theta[3],  delta_Pos[3],  rollAngle;//added by yuanjun

	//VeloScan *preScan; //added by yuanjun
};

