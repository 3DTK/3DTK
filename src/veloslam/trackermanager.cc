/*
 * trackermanager implementation
 *
 * Copyright (C) ZhangLiang, YuanJun, Li Wei, Li Ming, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Main programm for dynamic Velodyne SLAM
 *
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 * @author YuanJun, Wuhan University, China
 * @author ZhangLiang, Wuhan University, China
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include<cmath>

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include "slam6d/scan.h"
#include "slam6d/globals.icc"

#include "veloslam/color_util.h"
#include "veloslam/veloscan.h"
#include "veloslam/trackermanager.h"
#include "veloslam/debugview.h"
#include "veloslam/kalmanfilter.h"
#include "veloslam/lap.h"



#define KG  35
#define BIGNUM 100000

int sliding_window_size = 6;
int current_sliding_window_pos =0;
Trajectory VelodyneTrajectory;
VeloScan * g_pfirstScan;
float  constant_static_or_moving  = 20.0;

int GetScanID_in_SlidingWindow(int absNO, int current_pos, int  window_size)
{
        int firstNO=-1;
             // relative scanID   absolute scanID
             // absNO (6,7,8,9)  current_pos(6,7,8,9)  window_size =6
#ifdef  NO_SLIDING_WINDOW
		 firstNO = absNO;
#else
		if(   (absNO  >=(current_pos - window_size) )
           && (absNO  <= current_pos) )
         {
			 firstNO =absNO - (current_pos - window_size );
        }
		else
		{
			  //in first scans
              firstNO = -1;
		}
       if(current_pos < window_size)
       {
            //in first scans
             firstNO = absNO;
       }
#endif
//	   cout << " absNO  " << absNO  << " firstNO  "<<firstNO
//		      << " current_pos  " << current_pos << " window_size  "<< window_size <<endl ;
       return firstNO;
}

TrackerManager::TrackerManager(void)
{
	colorIdx=0;
	rollAngle=0;
    trackerStartID=0;
	delta_Pos[0]=delta_Pos[1]=delta_Pos[2]=0;
}

TrackerManager::~TrackerManager(void)
{
}

int TrackerManager::Init(void)
{
    trackerStartID=0;
	return 0;
}

/** @brief Initialize function */
int TrackerManager::getNumberofTracker(void)
{
	return  tracks.size();
}

int TrackerManager::HandleScan(VeloScan& scanRef,int trackingAlgo)
{
    //each scan only process once.

	if(scanRef.isTrackerHandled==true)
		return 0;
	else
		scanRef.isTrackerHandled=true;

	//cout<<"HandleScan is running!"<<endl;

	int i;
	int size=scanRef.scanClusterArray.size();
    //how many cluster need process.

	if(size==0)
		return 0;
    // maybe the problem for the scans allway changed.
	clusterStatus.resize(size);

    // all the objects in currect scan should be no matched and filtered.
	for(i=0; i<size;++i)
	{
		clusterStatus[i].Matched=false;
		clusterStatus[i].FilterRet=false;
	}

    //filter not good target object
	FilterObject(scanRef);

    //update same tracker this is need matching
	switch (trackingAlgo)
	{
	case 0:
		UpdateTrackers(scanRef);
		break;
	case 1:
		MatchTracksWithClusters(scanRef);
		break;
	}

    //add no tracked objects in to new tracker.
	AddTrackers(scanRef);

    //Remove No used trackers
    RemoveNoUsedTracker(scanRef);

	//preScan=&scanRef;

	return 0;
}

int TrackerManager::FilterObject(VeloScan& scanRef)
{
	int i;
	int size=scanRef.scanClusterArray.size();
	scanRef.clusterNum=0;

	for(i=0; i<size;++i)
	{
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];
		clusterStatus[i].FilterRet = TrackerManager::TrackerFilter(glu, gluData);
		if (clusterStatus[i].FilterRet)
		{
			scanRef.clusterNum++;
		}
	}

	cout<<"scanID: "<<scanRef.scanid<<" clusters to be tracked: "<<scanRef.clusterNum<<endl;
	return 0;
}


bool TrackerManager::TrackerFilter(clusterFeature &glu, cluster &gluData)
{
   if(FilterNOMovingObjcets(glu, gluData))
	   return true;
   else
	   return false;
}

//MatchTrackers with Kalman filters.
int TrackerManager::MatchTrackers(VeloScan& scanRef,Tracker& tracker,float kg)
{
	int i;
	int minID=-1;
	float value,minValue=360.0;
	float radiusDiff;
	float thetaDiff;
	float sizeDiff;
	float positionDiff;
    float shapeDiff;
	CMatrix standardDeviation(2,2),measurementErro(2,1);
	Measurement predictMeasurement;
	bool IsSmaller1,   IsSmaller2,   flag=false;
	clusterFeature temp;
	ObjectState movestate;

	predictMeasurement=tracker.kalmanFilter.GetPredictMeasurement(rollAngle,delta_Pos);
	standardDeviation=tracker.kalmanFilter.CalMeasureDeviation();
	for(i=0; i<scanRef.scanClusterArray.size();++i)
	{
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];

        // filter out other objects.
		if( clusterStatus.size()!=0  &&  clusterStatus[i].FilterRet==false)
    //	if( clusterStatus[i].FilterRet==false)
			continue;

        // allways get the moving for current scans
		measurementErro.m_pTMatrix[0][0]= glu.avg_x - predictMeasurement.x_measurement;
		measurementErro.m_pTMatrix[1][0]= glu.avg_z - predictMeasurement.z_measurement;

		IsSmaller1=(fabs(measurementErro.m_pTMatrix[0][0]) < kg*standardDeviation.m_pTMatrix[0][0]);
		IsSmaller2=(fabs(measurementErro.m_pTMatrix[1][0]) < kg*standardDeviation.m_pTMatrix[1][1]);

		if (IsSmaller1 && IsSmaller2)
		{
		//	cout<<"The "<<i<<"th object is matching with tracker "<<tracker.trackerID<<endl
		//		<<"with predictErro_x:"<<measurementErro.m_pTMatrix[0][0]<<"with predictErro_z:"<<measurementErro.m_pTMatrix[1][0]<<endl;
			if(tracker.statusList.size() >0)
			{
					radiusDiff=fabs(tracker.statusList.back().radius-glu.radius);
					thetaDiff=fabs(tracker.statusList.back().theta-glu.theta);
					sizeDiff =abs(tracker.statusList.back().size-glu.size);
					shapeDiff =fabs( fabs(tracker.statusList.back().size_x-glu.size_x) +
									 fabs(tracker.statusList.back().size_y-glu.size_y) +
									 fabs(tracker.statusList.back().size_z-glu.size_z) );
					positionDiff = sqrt(   (tracker.statusList.back().avg_x -glu.avg_x)*(tracker.statusList.back().avg_x -glu.avg_x) + 
						                            (tracker.statusList.back().avg_z -glu.avg_z)*(tracker.statusList.back().avg_z -glu.avg_z)
											  	) ;
			}
			value= radiusDiff*1.0 + thetaDiff*1.0  +  sizeDiff*0.8 + shapeDiff*0.8 + positionDiff * 0.3 ;
			if(value < minValue)
			{
					minValue=value;
					minID=i;
					temp=glu;
					flag=true;
			}

		}

	}

    // find matched tracker.
	if (flag)
	{
		tracker.kalmanFilter.stateUpdate(temp,rollAngle,delta_Pos);
		tracker.kalmanFilter.timeUpdate();
	//	movestate=tracker.kalmanFilter.GetCurrentState();

    //  cout<<"The tracker with trackerID:"<<tracker.trackerID<<"is matched with the "<<minID<<"th object!"<<endl;
	//	cout<<"with x_positon:"<<movestate.x_position<<", z_position:"<<movestate.z_positon<<endl;
	//	cout<<"with x_speed:"<<movestate.x_speed<<", z_speed:"<<movestate.z_speed<<endl;
	}
	return minID;
}


int TrackerManager::UpdateTrackers(VeloScan& scanRef)
{
	int i,j;
	int matchID;

	list<Tracker>::iterator it;
	int trackNO =0;
	for(it=tracks.begin() ; it!=tracks.end();it++ )
	{
		Tracker &tracker=*it;
		trackNO ++;

		if (!(tracker.missMatch))
		{
			matchID=MatchTrackers(scanRef,tracker,KG);
		}
		else
		{
			matchID=MatchTrackers(scanRef,tracker,1.5*KG);
		}

		if(matchID==-1)
		{
            // log miss times of tracker
			tracker.missMatch=true;
			tracker.missedTime++;
          //   cout << "missing  tracker" << tracker.trackerID
          //       <<" times " <<tracker.missedTime<<endl;
		}

        // find match tracker
		if(matchID >=0 )
		{
			scanRef.scanClusterFeatureArray[matchID].trackNO = trackNO;

			clusterFeature &glu=scanRef.scanClusterFeatureArray[matchID];
            cluster &gluData=scanRef.scanClusterArray[matchID];
			glu.frameNO= scanRef.scanid;
            glu.selfID = matchID;

			// save feature
			tracker.statusList.push_back(glu);
			tracker.dataList.push_back(gluData);
			tracker.matchClusterID=matchID;
			tracker.missMatch=false;

			clusterStatus[matchID].Matched=true;
		//	cout << "Match  tracker" << tracker.trackerID
         //        <<" size of " <<tracker.statusList.size()<<endl;
			tracker.missedTime=0;

			MoveState targetMove;
			targetMove.targetState=tracker.kalmanFilter.GetCurrentState();
			targetMove.frameNo=scanRef.scanid;
			tracker.moveStateList.push_back(targetMove);

			//FILE *fp=fopen("F:\\VeloSlamTemp\\bin\\Release\\Update.txt","a");
			//fprintf(fp,"TrackerID:%d,ScanID:%d,matchID:%d,clusterID:%d\n",tracker.trackerID,scanRef.scanid,matchID,glu.selfID);
			//fclose(fp);
		}

	}

	return 0;
}


int TrackerManager::AddTrackers(VeloScan& scanRef)
{
//	cout<<"AddTrackers is running!"<<endl;
	int i;
	int size=scanRef.scanClusterArray.size();
    // i is glu id in scan
	for(i=0;  i<size; ++i)
	{
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];

        // what cluster should added new tracker
		if(clusterStatus[i].FilterRet == true &&
           clusterStatus[i].Matched == false)
		{
			Tracker newTracker(glu,rollAngle);

			MoveState targetMove;
			targetMove.targetState	=newTracker.kalmanFilter.GetCurrentState();
			targetMove.frameNo=scanRef.scanid;
			newTracker.moveStateList.push_back(targetMove);

			newTracker.kalmanFilter.timeUpdate();
            // for find scan id for each cluster.
			glu.frameNO= scanRef.scanid;
            glu.selfID = i;

			newTracker.statusList.push_back(glu);
			newTracker.dataList.push_back(gluData);

			/*cout << "Add track" << TrackerManager::trackerStartID <<" glu.selfID "<< glu.selfID
                  <<" scanid" <<scanRef.scanid << endl;*/

			newTracker.colorIdx=colorIdx;
			newTracker.matchClusterID=i;
            newTracker.trackerID = TrackerManager::trackerStartID;
			tracks.push_back(newTracker);
            TrackerManager::trackerStartID++;
            colorIdx++;

		}
	}
	return 0;
}

int TrackerManager::ListTrackers()
{
    list<Tracker>::iterator it;
    cout<<" tracker list : "<<endl;
    for(it=tracks.begin() ; it!=tracks.end(); it++)
	{
		Tracker &tracker=*it;
        cout<<" ID "<<tracker.trackerID
            <<" matchClusterID "<<tracker.matchClusterID
            <<" missMatch "   <<tracker.missMatch
            <<" glusize "   <<tracker.statusList.size()
            <<" moving_distance "   <<tracker.moving_distance
            <<endl;
    }
	return 0;
}

// removednoUsedTrackers and out of sliding window's scans
int TrackerManager::RemoveNoUsedTracker(VeloScan& scanRef)
{
//	cout<<"RemoveNoUsedTracker is running!"<<endl;
//	cout << " current_sliding_window_pos: " << current_sliding_window_pos << " current id " << scanRef.scanid<< endl;

	list<Tracker>::iterator it;
	int trackNO =0;
	for(it=tracks.begin() ; it!=tracks.end();)
	{

		Tracker &tracker=*it;

#ifndef NO_SLIDING_WINDOW
		if (!tracker.statusList.empty())
		{
			deque<clusterFeature>::iterator Iter1;
			deque<cluster>::iterator Iter2;
			vector<MoveState>::iterator Iter3;
			Iter1=tracker.statusList.begin();
			Iter2=tracker.dataList.begin();
			Iter3=tracker.moveStateList.begin();

			do
			{
				clusterFeature &tempClu=*Iter1;
				if(tempClu.frameNO <current_sliding_window_pos - sliding_window_size
					|| tempClu.frameNO >current_sliding_window_pos)
				{
					//cout<<"trackerID: "<<tracker.trackerID<<" cluster "<<tempClu.selfID<<" in "<< tempClu.frameNO<<" scan is deleted!"<<endl;
					Iter1=tracker.statusList.erase(Iter1);
					Iter2=tracker.dataList.erase(Iter2);
					Iter3=tracker.moveStateList.erase(Iter3);
					continue;
				}
				Iter1++;
				Iter2++;
				Iter3++;
			}
			while(Iter1!=tracker.statusList.end());
		}
#endif

    	if (tracker.missedTime==4)
		{
			//cout<<" erase tracker "<<tracker.trackerID<<" is terminated"<<endl;
			it= tracks.erase(it);
			continue;
		}
		it++;
	}
	return 0;
}

void TrackerManager::TrackerManagerReset()
{
	clusterStatus.clear();
	list<Tracker>::iterator Iter;
	for (Iter=tracks.begin();Iter!=tracks.end();Iter++)
	{
		Tracker temp=*Iter;
		temp.TrackerReset();
	}
	tracks.clear();
	return;
}


int TrackerManager::CalculateTrackersFeature(vector <Scan *> allScans, int currentNO ,int windowsize)
{
    int i,j,k,colorIdx;
    float movement;
    list<Tracker>::iterator it;

	int n;

    for(it=tracks.begin(); it!=tracks.end(); it++)
    {
         Tracker &tracker=*it;
         tracker.moving_distance = 0.0;

         int size=tracker.statusList.size();
         if ( size <2 )
         {
             continue;
         }
		 movement =0;
         n=0;

         for ( i=0;i<size-2;i++)
         {
             clusterFeature &glu1=tracker.statusList[i];
             clusterFeature &glu2=tracker.statusList[i+1];

             Point p1(glu1.avg_x, glu1.avg_y, glu1.avg_z);
             Point p2(glu2.avg_x, glu2.avg_y, glu2.avg_z);

             int firstNO = GetScanID_in_SlidingWindow(glu1.frameNO,
                                 current_sliding_window_pos,
                                 sliding_window_size);
             int secondNO = GetScanID_in_SlidingWindow(glu2.frameNO,
                                 current_sliding_window_pos,
                                 sliding_window_size);

             if(firstNO <0 || secondNO< 0 )
                 continue;

             Scan *CurrentScan = allScans[firstNO];
             Scan *CurrentScanNext = allScans[secondNO];

             double  deltaMat[16];
             double  deltaMatNext[16];

             GetCurrecntdelteMat(*CurrentScan ,  deltaMat);
             GetCurrecntdelteMat(*CurrentScanNext ,  deltaMatNext);

           //  cout << " pose  no" << tracker.matchClusterID <<"  "
                //   <<  p1.x  <<"  " <<  p2.x <<"  "
                   //<<  p1.y  <<"  " <<  p2.y <<"  "
                   //<<  p1.z  <<"  " <<  p2.z <<"  " <<  movement <<"  " << endl;

             p1.transform(deltaMat);
             p2.transform(deltaMatNext);

#ifdef NO_SLIDING_WINDOW
			 if(glu1.frameNO>=current_sliding_window_pos - sliding_window_size&&glu2.frameNO <=current_sliding_window_pos)
			 {
				 movement += sqrt( (p1.x -p2.x)*(p1.x -p2.x)+(p1.z -p2.z)*(p1.z -p2.z) );
				 n++;
			 }
#else
			  movement += sqrt( (p1.x -p2.x)*(p1.x -p2.x)+(p1.z -p2.z)*(p1.z -p2.z) );
#endif
            // cout << " tracker no" << tracker.matchClusterID <<"  "
                //   <<  p1.x  <<"  " <<  p2.x <<"  "
                   //<<  p1.y  <<"  " <<  p2.y <<"  "
                   //<<  p1.z  <<"  " <<  p2.z <<"  " <<  movement <<"  " << endl;

         }

#ifdef NO_SLIDING_WINDOW
		 if (n!=0)
		 {
			 tracker.moving_distance=movement/n;
		 }
		 else
		 {
			 tracker.moving_distance=0;
		 }

#else
         tracker.moving_distance = movement/(size-1);
#endif

     }
	return 0;
}

int TrackerManager::MarkClassifiyTrackersResult(vector <Scan *> allScans, int currentNO ,int windowsize)
{
	int i,j,k,colorIdx;
    float movement;
	list<Tracker>::iterator it;

    // mark all objets type such as moving or static
      for(it=tracks.begin();  it!=tracks.end();  it++)
      {
              int currentScanNo =0;
              Tracker &tracker=*it;
              int size=tracker.statusList.size();
              // no tracking all for slam
              // for tracking classifiation //not transfrom to scan
              if (size < 3)
              {
                  for(i=0;i<size;i++)
                  {
                      clusterFeature &glu1=tracker.statusList[i];

                      currentScanNo= GetScanID_in_SlidingWindow(glu1.frameNO,
                                                       current_sliding_window_pos,
                                                       sliding_window_size);

                      if(currentScanNo <0 )
                           continue;

                      VeloScan *CurrentScan = ( VeloScan *)( allScans[currentScanNo]);
                      clusterFeature &realglu1=CurrentScan->scanClusterFeatureArray[glu1.selfID];
                      cluster  &realgclu = CurrentScan->scanClusterArray[glu1.selfID];

                      realglu1.clusterType  = CLUSTER_TYPE_STATIC_OBJECT;
                      for(j =0; j< realgclu.size() ; ++j)
                      {
                              cellFeature &gcF = *(realgclu[j]);
                              gcF.cellType = CELL_TYPE_STATIC;
                      }
                  }
                  continue;
              }

              for(i=0;i<size;i++)
              {
                  clusterFeature &glu1=tracker.statusList[i];

                  currentScanNo= GetScanID_in_SlidingWindow(glu1.frameNO,
                                                    current_sliding_window_pos,
                                                    sliding_window_size);

                  if(currentScanNo <0 )
                           continue;

                  VeloScan *CurrentScan = ( VeloScan *)( allScans[currentScanNo]);
                  clusterFeature &realglu1=CurrentScan->scanClusterFeatureArray[glu1.selfID];
                  cluster  &realgclu = CurrentScan->scanClusterArray[glu1.selfID];

					if(tracker.moving_distance < constant_static_or_moving)
					{
						realglu1.clusterType = CLUSTER_TYPE_STATIC_OBJECT;
						for(j =0; j< realgclu.size() ; ++j)
						{
							cellFeature &gcF = *(realgclu[j]);
							gcF.cellType = CELL_TYPE_STATIC;
						}
					}
                   else
                   {
                      realglu1.clusterType  = CLUSTER_TYPE_MOVING_OBJECT;
                      for(j =0; j< realgclu.size() ; ++j)
                      {
                          cellFeature &gcF = *(realgclu[j]);
                          gcF.cellType = CELL_TYPE_MOVING;
                      }

                   }

              }
      }
	  return 0;
}


int TrackerManager::ClassifiyTrackersObjects(vector <Scan *> allScans, int currentNO ,int windowsize)
{
    CalculateTrackersFeature(allScans, currentNO, windowsize);
    MarkClassifiyTrackersResult(allScans, currentNO , windowsize);
	return 0;
}

int TrackerManager::UpdateClustersPoistioninTrackers()
{
	int i,j;

	list<Tracker>::iterator it;
	for(it=tracks.begin() ; it!=tracks.end();it++ )
	{
		Tracker &tracker=*it;
/*
        	   firstNO = GetScanID_in_SlidingWindow(glu1.frameNO,
                                 current_sliding_window_pos,
                                 sliding_window_size);
               secondNO = GetScanID_in_SlidingWindow(glu2.frameNO,
                                 current_sliding_window_pos,
                                 sliding_window_size);
 			   if(firstNO <0 || secondNO< 0 )
				    continue;

			    Scan *firstScan = (Scan *)g_pfirstScan;
                Scan *CurrentScan = allScans[firstNO];
				Scan *CurrentScanNext = allScans[secondNO];

				double  deltaMat[16];
				double  deltaMatNext[16];

				GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);
				GetCurrecntdelteMat(*CurrentScanNext , *firstScan,  deltaMatNext);

				p1.x = glu1.avg_x; p1.y= glu1.avg_y;p1.z=glu1.avg_z;
				p1text.x= glu1.avg_x+150; p1text.y= glu1.avg_y+80; p1text.z=glu1.avg_z+50;
				p2.x = glu2.avg_x; p2.y= glu2.avg_y;  p2.z=glu2.avg_z;
				p2text.x= glu2.avg_x+150; p2text.y= glu2.avg_y+80; p2text.z=glu2.avg_z+50;
*/
	}
	return 0;
}

void TrackerManager::GetTwoScanRoll(Scan *CurrentScan,Scan *preScan)//???
{
	double tempMat[16],deltaMat[16];
	M4inv(preScan->get_transMat(), tempMat);
	MMult(CurrentScan->get_transMat(), tempMat,deltaMat);
	Matrix4ToEuler(deltaMat,delta_Theta,delta_Pos);
	//cout<<"delta_Theta: "<<delta_Theta[0]<<" "<<delta_Theta[1]<<" "<<delta_Theta[2]<<endl;
	//cout<<"delta_pos: "<<delta_Pos[0]<<" "<<delta_Pos[1]<<" "<<delta_Pos[2]<<endl;
}

CMatrix TrackerManager::ConstructCostMatrix(VeloScan &scanRef,int *clusterIndex)
{
	int clusterSize=scanRef.clusterNum;
	int trackSize=getNumberofTracker();

	int maxSize;
	if (clusterSize!=trackSize)
	{
		maxSize=clusterSize>trackSize?clusterSize:trackSize;
	}
	else
	{
		maxSize=clusterSize;
	}

	CMatrix costMatrix(maxSize,maxSize);

	int i=0,j,k;
	double costValue;
	float radiusDiff;
	float thetaDiff;
	float sizeDiff;
	float positionDiff;
	CMatrix standardDeviation(2,2),measurementErro(2,1);
	Measurement predictMeasurement;
	bool IsSmaller1,IsSmaller2;
	float kg;

	list<Tracker>::iterator it;
	for(it=tracks.begin(); it!=tracks.end(); it++)
	{
		Tracker &tracker=*it;

		predictMeasurement=tracker.kalmanFilter.GetPredictMeasurement(rollAngle,delta_Pos);
		standardDeviation=tracker.kalmanFilter.CalMeasureDeviation();

		if (tracker.missMatch)
		{
			kg=1.5*KG;
		}
		else
		{
			kg=KG;
		}

		k=0;
		for(j=0;j<scanRef.scanClusterArray.size();j++)
		{

			clusterFeature &glu=scanRef.scanClusterFeatureArray[j];
			cluster &gluData=scanRef.scanClusterArray[j];

			if(clusterStatus.size()!=0&&clusterStatus[j].FilterRet==false)
				continue;

			measurementErro.m_pTMatrix[0][0]=glu.avg_x-predictMeasurement.x_measurement;
			measurementErro.m_pTMatrix[1][0]=glu.avg_z-predictMeasurement.z_measurement;

			IsSmaller1=(fabs(measurementErro.m_pTMatrix[0][0])<kg*standardDeviation.m_pTMatrix[0][0]);
			IsSmaller2=(fabs(measurementErro.m_pTMatrix[1][0])<kg*standardDeviation.m_pTMatrix[1][1]);

			if (IsSmaller1&&IsSmaller2)
			{
				radiusDiff=fabs(tracker.statusList.back().radius-glu.radius);
				thetaDiff=fabs(tracker.statusList.back().theta-glu.theta);
				sizeDiff =abs(tracker.statusList.back().size-glu.size);
				positionDiff = sqrt(sqr(tracker.statusList.back().avg_x -glu.avg_x) + sqr(tracker.statusList.back().avg_z -glu.avg_z) ) ;
				costValue= radiusDiff*1.0 + thetaDiff*1.0  +  sizeDiff*0.8 + positionDiff * 0.03 ;
				costMatrix.m_pTMatrix[i][k]=costValue;
			}
			else
			{
				costMatrix.m_pTMatrix[i][k]=BIGNUM;
			}

			if (i==0)
			{
				clusterIndex[k]=j;
			}
			k++;
		}
		i++;
	}

	if (clusterSize!=trackSize)
	{
		if (maxSize==clusterSize)
		{
			for (i=trackSize;i<maxSize;i++)
			{
				for (j=0;j<maxSize;j++)
				{
					costMatrix.m_pTMatrix[i][j]=BIGNUM;
				}
			}
		}
		else
		{
			for (i=0;i<maxSize;i++)
			{
				for (j=clusterSize;j<maxSize;j++)
				{
					costMatrix.m_pTMatrix[i][j]=BIGNUM;
				}
			}
		}
	}

	return costMatrix;
}

int TrackerManager::MatchTracksWithClusters(VeloScan &scanRef)
{
//	cout<<"MatchTracksWithClusters is running!"<<endl;
	if (tracks.empty())
	{
		return 0;
	}

	CMatrix costMatrix;
	int clusterSize=scanRef.clusterNum;
	int trackSize=getNumberofTracker();
	int *clusterIndex;
	clusterIndex=new int[clusterSize];
	costMatrix=ConstructCostMatrix(scanRef,clusterIndex);
	int dim=costMatrix.m_nCol;

	//FILE *fp=fopen("F:\\VeloSlamTemp\\bin\\Release\\CostMatix.txt","a");
	//fprintf(fp,"scanID:%d\n",scanRef.scanid);
	//for (int m=0;m<dim;m++)
	//{
	//	for (int l=0;l<dim;l++)
	//	{
	//		fprintf(fp,"%.1f\t",costMatrix.m_pTMatrix[m][l]);

	//	}
	//	fprintf(fp,"\n");
	//}
	//fclose(fp);

	int *colsol,*rowsol;
	colsol=new int[dim];
	rowsol=new int[dim];

	double *u,*v;
	u=new double[dim];
	v=new double[dim];

	int i,j;
	bool *missFlag;
	missFlag=new bool[trackSize];
	for (i=0;i<trackSize;i++)
	{
		missFlag[i]=false;
	}

	double **assignCost;
	assignCost = new double*[dim];
	for (i = 0; i < dim; i++)
		assignCost[i] = new double[dim];

	for (i=0;i<dim;i++)
	{
		for (j=0;j<dim;j++)
		{
			assignCost[i][j]=costMatrix.m_pTMatrix[i][j];
		}
	}

	lap(dim,assignCost,rowsol,colsol,u,v);

	for (i=0;i<trackSize;i++)
	{
		int col=rowsol[i];
		if (assignCost[i][col]==BIGNUM)
		{
			missFlag[i]=true;
		}
	}

	int trackerIndex=-1;
	int trackNO =0;
	list<Tracker>::iterator it;
	for(it=tracks.begin() ; it!=tracks.end();it++)
	{
		Tracker &tracker=*it;
		trackNO ++;
		trackerIndex++;
		if (missFlag[trackerIndex])
		{
			tracker.missMatch=true;
			tracker.missedTime++;

		}
		else
		{
			int clusterID=rowsol[trackerIndex];
			int matchID=clusterIndex[clusterID];
			//cout<<"TrackerID: "<<tracker.trackerID<<"scanID: "<<scanRef.scanid<<"MatchID: "<<matchID<<endl;
			scanRef.scanClusterFeatureArray[matchID].trackNO = trackNO;

			clusterFeature &glu=scanRef.scanClusterFeatureArray[matchID];
			cluster &gluData=scanRef.scanClusterArray[matchID];
			glu.frameNO= scanRef.scanid;
            glu.selfID = matchID;

			tracker.kalmanFilter.stateUpdate(glu,rollAngle,delta_Pos);
			tracker.kalmanFilter.timeUpdate();

			tracker.statusList.push_back(glu);
			tracker.dataList.push_back(gluData);
			tracker.matchClusterID=matchID;
			tracker.missMatch=false;
			tracker.missedTime=0;

			clusterStatus[matchID].Matched=true;

			MoveState targetMove;
			targetMove.targetState=tracker.kalmanFilter.GetCurrentState();
			targetMove.frameNo=scanRef.scanid;
			tracker.moveStateList.push_back(targetMove);

			//FILE *fp=fopen("F:\\VeloSlamTemp\\bin\\Release\\Lap.txt","a");
			//fprintf(fp,"TrackerID:%d,ScanID:%d,matchID:%d,clusterID:%d\n",tracker.trackerID,scanRef.scanid,matchID,glu.selfID);
			//fclose(fp);

		}

	}

	delete[] assignCost;
	delete[] rowsol;
	delete[] colsol;
	delete[] u;
	delete[] v;
	delete[] clusterIndex;
	delete[] missFlag;

	return 0;
}
