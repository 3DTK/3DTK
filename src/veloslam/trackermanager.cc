#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "veloslam/color_util.h"
#include "veloslam/veloscan.h"
#include "veloslam/trackermanager.h"
#include "veloslam/debugview.h"
#include "veloslam/kalmanfilter.h"

#include <GL/gl.h>		   	/* OpenGL header file */
#include <GL/glu.h>			/* OpenGL utilities header file */

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

#define KG 35

int sliding_window_size = 6;
int current_sliding_window_pos =0;
Trajectory VelodyneTrajectory;
VeloScan * g_pfirstScan;

// is OK
int GetScanID_in_SlidingWindow(int absNO, int current_pos, int  window_size)
{
        int firstNO=-1;
             // relative scanID   absolute scanID
             // absNO (6,7,8,9)  current_pos(6,7,8,9)  window_size =6
		if(current_pos > window_size)
         {
			 // in slidingwindow
			 if(absNO  >= current_pos - window_size )
                   firstNO =absNO - (current_pos - window_size );
			 // out of slidingwindow
			 else
				    firstNO = -1;
        }
		else
		{
			  //in first scans
              firstNO = absNO;
		}
	//   cout << " absNO  " << absNO  << " firstNO  "<<firstNO
	//	      << " current_pos  " << current_pos << " window_size  "<< window_size <<endl ;
       return firstNO;
}

TrackerManager::TrackerManager(void)
{
	colorIdx=0;
	rollAngle=0;
    trackerStartID=0;
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

int TrackerManager::HandleScan(VeloScan& scanRef)
{
    //each scan only process once.
	if(scanRef.isTrackerHandled==true)
		return 0;
	else
		scanRef.isTrackerHandled=true;

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

	rollAngle=0;
    //filter not good target object
	FilterObject(scanRef);

    //update same tracker this is need matching
	UpdateTrackers(scanRef);

    //add no trackerd objects in to new tracker.
	AddTrackers(scanRef);

    //Remove No used trackers
    RemoveNoUsedTracker(scanRef);

	/*preScan=&scanRef;*/
	return 0;
}

int TrackerManager::FilterObject(VeloScan& scanRef)
{
	int i;
	int size=scanRef.scanClusterArray.size();

	for(i=0; i<size;++i)
	{
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];
		clusterStatus[i].FilterRet = TrackerManager::TrackerFilter(glu, gluData);
	}
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
	CMatrix standardDeviation(2,2),measurementErro(2,1);
	Measurement predictMeasurement;
	bool IsSmaller1,   IsSmaller2,   flag=false;
	clusterFeature temp;
	ObjectState movestate;

	predictMeasurement=tracker.kalmanFilter.GetPredictMeasurement(rollAngle);
	standardDeviation=tracker.kalmanFilter.CalMeasureDeviation();
	for(i=0; i<scanRef.scanClusterArray.size();++i)
	{
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];

        // filter out other objects.
	//	if( clusterStatus.size()!=0 && clusterStatus[i].FilterRet==false)
    	if( clusterStatus[i].FilterRet==false)
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
				positionDiff = sqrt(sqr(tracker.statusList.back().avg_x -glu.avg_x) + sqr(tracker.statusList.back().avg_z -glu.avg_z) ) ;
			}
			value= radiusDiff*1.0 + thetaDiff*1.0  +  sizeDiff*0.8 + positionDiff * 0.03 ;
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
		tracker.kalmanFilter.stateUpdate(temp,rollAngle);
		tracker.kalmanFilter.timeUpdate();
		movestate=tracker.kalmanFilter.GetCurrentState();

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
		}

	}

	return 0;
}


int TrackerManager::AddTrackers(VeloScan& scanRef)
{
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

			newTracker.kalmanFilter.timeUpdate();
            // for find scan id for each cluster.
			glu.frameNO= scanRef.scanid;
            glu.selfID = i;

			newTracker.statusList.push_back(glu);
			newTracker.dataList.push_back(gluData);

	//		cout << "Add track" << TrackerManager::trackerStartID <<" glu.selfID "<< glu.selfID
    //              <<" scanid" <<scanRef.scanid << endl;

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

// removednoUsedTrackers and out of sliding window's scans
int TrackerManager::RemoveNoUsedTracker(VeloScan& scanRef)
{
    list<Tracker>::iterator it;
	int trackNO =0;
	for(it=tracks.begin() ; it!=tracks.end(); it++)
	{

		Tracker &tracker=*it;
        //removednoUsedTrackers
        if (tracker.missedTime==3)
        {
       //     cout<<" erase tracker "<<tracker.trackerID<<" is terminated"<<endl;
            it= tracks.erase(it);
            continue;
        }

         // remove the out of sliding window's scans
         for (int i =0; i<tracker.statusList.size()-1; i++)
         {
             clusterFeature &glu1= tracker.statusList[i];
             cluster &gludata=tracker.dataList[i];
             if(glu1.frameNO < current_sliding_window_pos - sliding_window_size
                 || glu1.frameNO > current_sliding_window_pos)
             {
            //    cout<<" erase glu in  "<<tracker.trackerID
           //        <<" glu1.frameNO "<< glu1.frameNO
            //        <<" glu1 ID "<< glu1.trackNO
            //        << endl;
				    tracker.statusList.pop_front();
                    tracker.dataList.pop_front();
                    i++;
             }
         }
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

    for(it=tracks.begin(); it!=tracks.end(); it++)
    {
         Tracker &tracker=*it;
         tracker.moving_distance = 0.0;

         int size=tracker.statusList.size();
         if (current_sliding_window_pos  < 3 ||  size <3 )  //不处理前三帧和跟踪不到三个的。
         {
             continue;
         }

         movement =0;
         for ( i=0 ;i<size-2;i++)
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
                                 sliding_window_size);;

             if(firstNO <0 || secondNO< 0 )
                 continue;

             Scan *firstScan = (Scan *)g_pfirstScan;  //the first scan.
             Scan *CurrentScan = allScans[firstNO];
             Scan *CurrentScanNext = allScans[secondNO];

             double  deltaMat[16];
             double  deltaMatNext[16];

             GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);
             GetCurrecntdelteMat(*CurrentScanNext , *firstScan,  deltaMatNext);

           //  cout << " pose  no" << tracker.matchClusterID <<"  "
                //   <<  p1.x  <<"  " <<  p2.x <<"  "
                   //<<  p1.y  <<"  " <<  p2.y <<"  "
                   //<<  p1.z  <<"  " <<  p2.z <<"  " <<  movement <<"  " << endl;

             p1.transform(deltaMat);
             p2.transform(deltaMatNext);

             movement += sqrt( (p1.x -p2.x)*(p1.x -p2.x)
                      //        +(p1.y -p2.y)*(p1.y -p2.y)
                               +(p1.z -p2.z)*(p1.z -p2.z) );

            // cout << " tracker no" << tracker.matchClusterID <<"  "
                //   <<  p1.x  <<"  " <<  p2.x <<"  "
                   //<<  p1.y  <<"  " <<  p2.y <<"  "
                   //<<  p1.z  <<"  " <<  p2.z <<"  " <<  movement <<"  " << endl;

         }
         tracker.moving_distance = movement/(size-1);
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

            //      if(tracker.moving_distance < 8.0)
 /*              if(0)
                   {
                      realglu1.clusterType = CLUSTER_TYPE_STATIC_OBJECT;
                      for(j =0; j< realgclu.size() ; ++j)
                      {
                          cellFeature &gcF = *(realgclu[j]);
                          gcF.cellType = CELL_TYPE_STATIC;
                      }
                   }
                   else
*/
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


