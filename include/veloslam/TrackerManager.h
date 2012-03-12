#pragma once

#include <list>
#include <vector>
#include "slam6d/scan.h"
#include "Veloslam/Tracker.h"

using namespace std;

struct ClusterStatus
{
	bool FilterRet;
	bool Matched;
};


class TrackerManager
{
public:
	/** @brief CTor */
	TrackerManager(void);

	/** @brief DTor*/
	~TrackerManager(void);

	/** @brief Initialize function */
	int Init();

	/** @brief 处理一帧数据，！！注意的是，这一帧必须已经被处理（投影和集群）过！！*/
	int HandleScan(VeloScan& scanRef);

	/** @brief 将这一帧的cluster画出来*/
	int DrawScanCluster(VeloScan& scanRef);

	/** @brief 将所有的跟踪器画出来*/
	int DrawTrackers(VeloScan& scanRefs);

	/** @brief 将所有的跟踪器运动情况画出来*/
	int DrawTrackersMovtion(VeloScan& scanRef1,  VeloScan& scanRef2);

	/** @brief 将所有的跟踪器运动情况画出来, 完整的连续轨迹*/
	int DrawTrackersMovtion_Long(vector <VeloScan *> allScans);

	/** @brief 将所有的跟踪器运动情况画出来, 完整的连续轨迹*/
	int DrawTrackersMovtion_Long_Number(vector <VeloScan *> allScans, int n);

	/** @brief 跟踪过滤器，把一些不符合跟踪的cluster去掉*/
	static bool TrackerFilter(clusterFeature &glu);

	/** @brief cluster状态数据容器*/
	vector<ClusterStatus> clusterStatus;

private:
	/** @brief 更新跟踪器*/
	int UpdateTrackers(VeloScan& scanRef);

	/** @brief 添加跟踪器*/
	int AddTrackers(VeloScan& scanRef);

	/** @brief 匹配跟踪器*/
	int MatchTrackers(VeloScan& scanRef,Tracker& tracker);

	/** @brief 过滤特定物体*/
	int FilterObject(VeloScan& scanRef);
	
	/** @brief 跟踪器列表容器*/
	list<Tracker> tracks;

	int colorIdx;
};

