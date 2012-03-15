#pragma once
#include "veloslam/veloscan.h"
#include <deque>
 
using namespace std;
/** 
跟踪器，每一个跟踪器代表一个被跟踪对象，并记录了被跟踪对象的信息
*/
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

