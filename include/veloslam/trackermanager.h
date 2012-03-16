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
	/** @brief CTor */
	TrackerManager(void);

	/** @brief DTor*/
	~TrackerManager(void);

	/** @brief Initialize function */
	int Init();

	/** @brief ����һ֡���ݣ�����ע����ǣ���һ֡�����Ѿ�������ͶӰ�ͼ�Ⱥ��������*/
	int HandleScan(VeloScan& scanRef);

	/** @brief ����һ֡��cluster������*/
	int DrawScanCluster(VeloScan& scanRef);

	/** @brief �����еĸ�����������*/
	int DrawTrackers(VeloScan& scanRefs);

	/** @brief �����еĸ������˶����������*/
	int DrawTrackersMovtion(VeloScan& scanRef1,  VeloScan& scanRef2);

	/** @brief �����еĸ������˶����������, �����������켣*/
	int DrawTrackersMovtion_Long(vector <VeloScan *> allScans);

	/** @brief �����еĸ������˶����������, �����������켣*/
	int DrawTrackersMovtion_Long_Number(vector <Scan *> allScans, int n);

	/** @brief �����еĸ������˶����������, what every size or lenght, �����������켣*/
	int TrackerManager::DrawTrackersMovtion_Long_Number_All(vector <Scan *> allScans, int n);

	/** @brief ���ٹ���������һЩ�����ϸ��ٵ�clusterȥ��*/
	static bool TrackerFilter(clusterFeature &glu);

	/** @brief cluster״̬��������*/
	vector<ClusterStatus> clusterStatus;

private:
	/** @brief ���¸�����*/
	int UpdateTrackers(VeloScan& scanRef);

	/** @brief ��Ӹ�����*/
	int AddTrackers(VeloScan& scanRef);

	/** @brief ƥ�������*/
	int MatchTrackers(VeloScan& scanRef,Tracker& tracker);

	/** @brief �����ض�����*/
	int FilterObject(VeloScan& scanRef);
	
	/** @brief �������б�����*/
	list<Tracker> tracks;

	int colorIdx;
};

