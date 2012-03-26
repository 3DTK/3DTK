#pragma once
#include "veloslam/veloscan.h"
#include <deque>

using namespace std;

class Tracker
{
public:
	Tracker(void);
	~Tracker(void);
	deque<clusterFeature> statusList;
	deque<cluster> dataList;
	bool missMatch;
	bool Matched;
	int matchClusterID;
	int colorIdx;
	int frameNO;

};

