#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "veloslam/color_util.h"
#include "veloslam/veloscan.h"
#include "veloslam/trackermanager.h"
#include "veloslam/debugview.h"
#include "veloslam/kalmanfilter.h"

#include <GL/gl.h>		    	/* OpenGL header file */
#include <GL/glu.h>			/* OpenGL utilities header file */

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

#define KG 35

TrackerManager::TrackerManager(void)
{
	colorIdx=0;
	rollAngle=0;
}


TrackerManager::~TrackerManager(void)
{
}

int TrackerManager::Init(void)
{
	return 0;
}

	/** @brief Initialize function */
int TrackerManager::getNumberofTracker(void)
{
	return  tracks.size();
}

int TrackerManager::HandleScan(VeloScan& scanRef)
{

	if(scanRef.isTrackerHandled==true)
		return 0;
	else
		scanRef.isTrackerHandled=true;

	int i;
	int size=scanRef.scanClusterArray.size();

	if(size==0)
		return 0;

	clusterStatus.resize(size);

	for(i=0; i<size;++i)
		clusterStatus[i].Matched=false;

	rollAngle=0;

	FilterObject(scanRef);

	UpdateTrackers(scanRef);

	AddTrackers(scanRef);

	/*preScan=&scanRef;*/

	return 0;
}


int TrackerManager::AddTrackers(VeloScan& scanRef)
{
	int i;
	int size=scanRef.scanClusterArray.size();

	for(i=0; i<size;++i)
	{

		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];
		if(clusterStatus[i].FilterRet==true&&clusterStatus[i].Matched==false)
		{
			Tracker newTracker(glu,rollAngle);

			newTracker.kalmanFilter.timeUpdate();

			newTracker.frameNO =scanRef.scanid;

			glu.frameNO= scanRef.scanid;

			newTracker.statusList.push_back(glu);
			newTracker.dataList.push_back(gluData);
			//		TRACE("Add track %d %f %f\n",colorIdx,glu.theta,glu.radius);
			newTracker.colorIdx=colorIdx;
			colorIdx++;

			newTracker.matchClusterID=i;
			tracks.push_back(newTracker);

		}
	}
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
   if (FilterNOMovingObjcets(glu, gluData ))
	    return true;
   else 
	   return false;
}

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
	bool IsSmaller1,IsSmaller2,flag=false;
	clusterFeature temp;
	ObjectState movestate;

	predictMeasurement=tracker.kalmanFilter.GetPredictMeasurement(rollAngle);

	standardDeviation=tracker.kalmanFilter.CalMeasureDeviation();

	for(i=0; i<scanRef.scanClusterArray.size();++i)
	{

		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];

		if(clusterStatus.size()!=0&&clusterStatus[i].FilterRet==false)
			continue;

		measurementErro.m_pTMatrix[0][0]=glu.avg_x-predictMeasurement.x_measurement;
		measurementErro.m_pTMatrix[1][0]=glu.avg_z-predictMeasurement.z_measurement;

		IsSmaller1=(fabs(measurementErro.m_pTMatrix[0][0])<kg*standardDeviation.m_pTMatrix[0][0]);
		IsSmaller2=(fabs(measurementErro.m_pTMatrix[1][0])<kg*standardDeviation.m_pTMatrix[1][1]);

		if (IsSmaller1&&IsSmaller2)
		{
	/*		cout<<"The "<<i<<"th object is matching with tracker "<<tracker.trackerID<<endl
				<<"with predictErro_x:"<<measurementErro.m_pTMatrix[0][0]<<"with predictErro_z:"<<measurementErro.m_pTMatrix[1][0]<<endl;*/

			radiusDiff=fabs(tracker.statusList.back().radius-glu.radius);
			thetaDiff=fabs(tracker.statusList.back().theta-glu.theta);
			sizeDiff =abs(tracker.statusList.back().size-glu.size);
			positionDiff = sqrt(sqr(tracker.statusList.back().avg_x -glu.avg_x) + sqr(tracker.statusList.back().avg_z -glu.avg_z) ) ;
			value= radiusDiff*1.0 + thetaDiff*1.0  +  sizeDiff*0.8 + positionDiff * 0.03 ;
			if(value<minValue)
			{
				minValue=value;
				minID=i;
				temp=glu;
				flag=true;
			}

		}

	}
	if (flag)
	{
		tracker.kalmanFilter.stateUpdate(temp,rollAngle);
		tracker.kalmanFilter.timeUpdate();
		movestate=tracker.kalmanFilter.GetCurrentState();
		//cout<<"The tracker with trackerID:"<<tracker.trackerID<<"is matched with the "<<minID<<"th object!"<<endl;
		//cout	<<"with x_positon:"<<movestate.x_position<<", z_position:"<<movestate.z_positon<<endl;
		//cout	<<"with x_speed:"<<movestate.x_speed<<", z_speed:"<<movestate.z_speed<<endl;
	}
	return minID;
}


int TrackerManager::UpdateTrackers(VeloScan& scanRef)
{
	int i,j;
	int matchID;

	list<Tracker>::iterator it;
	int trackNO =0;
	for(it=tracks.begin() ; it!=tracks.end(); )
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
			tracker.missMatch=true;
			tracker.missedTime++;
			if (tracker.missedTime==3)
			{
				//cout<<"The tracker with trackerID:"<<tracker.trackerID<<" is terminated"<<endl;
				it=tracks.erase(it);
				continue;
			}
			it++;
		}
		else
		{
			scanRef.scanClusterFeatureArray[matchID].trackNO = trackNO;
			// save feature
			tracker.statusList.push_back(scanRef.scanClusterFeatureArray[matchID]);
			// save data
			tracker.dataList.push_back(scanRef.scanClusterArray[matchID]);
			tracker.matchClusterID=matchID;
			tracker.missMatch=false;
			tracker.Matched = true;

			clusterStatus[matchID].Matched=true;
			//	TRACE("Match  tracker %d,len:%d\n",tracker.colorIdx,tracker.statusList.size());
			tracker.missedTime=0;
			it++;
		}

	}

	return 0;
}


int TrackerManager::DrawScanCluster(VeloScan& scanRef)
{
	int i,j,k,colorIdx;
	GLdouble dVect1[3];
//	glPointSize(1);

//	glBegin(GL_POINTS);
	for(i=0; i<scanRef.scanClusterArray.size();++i)
	{

		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];
		if( clusterStatus.size()!=0  &&  clusterStatus[i].FilterRet==false)
			continue;

		colorIdx=i%8;

		if(scanRef.scanid ==0 )
				Draw_Cube_GL_RGB(glu, ColorTableShot[0][0], ColorTableShot[0][1], ColorTableShot[0][2]);
		if(scanRef.scanid ==1 )
			    Draw_Cube_GL_RGB(glu, ColorTableShot[1][0], ColorTableShot[1][1], ColorTableShot[1][2]);
		if(scanRef.scanid ==2 )
			    Draw_Cube_GL_RGB(glu, ColorTableShot[2][0], ColorTableShot[2][1], ColorTableShot[2][2]);
		if(scanRef.scanid ==3 )
				Draw_Cube_GL_RGB(glu, ColorTableShot[3][0], ColorTableShot[3][1], ColorTableShot[3][2]);
		if(scanRef.scanid ==4 )
			    Draw_Cube_GL_RGB(glu, ColorTableShot[4][0], ColorTableShot[4][1], ColorTableShot[4][2]);
		if(scanRef.scanid ==5 )
			   Draw_Cube_GL_RGB(glu, ColorTableShot[5][0], ColorTableShot[5][1], ColorTableShot[5][2]);

		/*
		glColor3d(ColorTableShot[colorIdx][0],  ColorTableShot[colorIdx][1],  ColorTableShot[colorIdx][2]);

		for(j=0; j<gluData.size();++j)
		{
			cell* pCell=gluData[j]->pCell;
			for(k=0; k<pCell->size();++k)
			{
				dVect1[0]=(*pCell)[k]->x;
				dVect1[1]=(*pCell)[k]->y;
				dVect1[2]=(*pCell)[k]->z;

				glVertex3dv(dVect1);
			}
		}
		*/

	}
//	glEnd();

	return 0;
}

int TrackerManager::DrawTrackers(VeloScan& scanRef)
{
	int i,j,k,colorIdx;
	GLdouble dVect1[3];
	glPointSize(3);
	glBegin(GL_POINTS);
	list<Tracker>::iterator it;

	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		Tracker &tracker=*it;
		if(tracker.missMatch == true)
			continue;

		//	if(tracker.Matched == false)
		///	continue;

		clusterFeature &glu=scanRef.scanClusterFeatureArray[tracker.matchClusterID];
		cluster &gluData=scanRef.scanClusterArray[tracker.matchClusterID];

		//  TRACE("Draw  tracker %d,len:%d\n",tracker.colorIdx,tracker.statusList.size());
		colorIdx=tracker.colorIdx%8;

//		glColor3d(ColorTableShot[colorIdx][0], ColorTableShot[colorIdx][1], ColorTableShot[colorIdx][2]);

		if(scanRef.scanid ==4 )
				glColor3d(ColorTableShot[0][0], ColorTableShot[0][1], ColorTableShot[0][2]);
		if(scanRef.scanid ==5 )
			    glColor3d(ColorTableShot[1][0], ColorTableShot[1][1], ColorTableShot[1][2]);
		if(scanRef.scanid ==6 )
			    glColor3d(ColorTableShot[2][0], ColorTableShot[2][1], ColorTableShot[2][2]);
		if(scanRef.scanid ==7 )
				glColor3d(ColorTableShot[3][0], ColorTableShot[3][1], ColorTableShot[3][2]);
		if(scanRef.scanid ==0 )
			    glColor3d(ColorTableShot[4][0], ColorTableShot[4][1], ColorTableShot[4][2]);
		if(scanRef.scanid ==1 )
			    glColor3d(ColorTableShot[5][0], ColorTableShot[5][1], ColorTableShot[5][2]);

		for(j=0; j<gluData.size();++j)
		{
			cell* pCell=gluData[j]->pCell;
			for(k=0; k<pCell->size();++k)
			{
				dVect1[0]=(*pCell)[k]->x;
				dVect1[1]=(*pCell)[k]->y;
				dVect1[2]=(*pCell)[k]->z;

				glVertex3dv(dVect1);
			}

		}


	}
	glEnd();
	return 0;
}

int TrackerManager::DrawTrackersMovtion(VeloScan& scanRef1, VeloScan& scanRef2)
{
	 int i,j,k,colorIdx;
	 Point p, q;
	double  deltaMat[16];
	GetCurrecntdelteMat(scanRef1 , scanRef2,  deltaMat);

	GLdouble dVect1[3];
	GLdouble dVect2[3];

	list<Tracker>::iterator it;

	for(it=tracks.begin();  it!=tracks.end();  it++)
	{

		Tracker &tracker=*it;
		if(tracker.missMatch == true)
			continue;

		//	if(tracker.Matched == false)
		///	continue;

		if(tracker.statusList.size() > 1)
		{
		//	cerr << " object number  " << tracker.statusList.size() <<endl;
			clusterFeature &glu1=tracker.statusList[0];
			clusterFeature &glu2=tracker.statusList[1];

		//	clusterFeature &glu=scanRef.scanClusterFeatureArray[tracker.matchClusterID];
		  //   cluster &gluData=scanRef.scanClusterArray[tracker.matchClusterID];

			cluster &gluData1=tracker.dataList[0];
			cluster &gluData2=tracker.dataList[1];
	//		clusterFeature &glu3=tracker.statusList[2];
			//  TRACE("Draw  tracker %d,len:%d\n",tracker.colorIdx,tracker.statusList.size());

			cell* pCell;
			glBegin(GL_POINTS);
			glPointSize(1);
			colorIdx=tracker.colorIdx%8;
		//	glColor3d(ColorTableShot[colorIdx][0], ColorTableShot[colorIdx][1], ColorTableShot[colorIdx][2]);
	    	glColor3d(0.3 , 0.3 , 0.3);
			for(j=0; j<gluData1.size();++j)
			{
				pCell=gluData1[j]->pCell;
				for(k=0; k<pCell->size();++k)
				{
					dVect1[0]=(*pCell)[k]->x;
					dVect1[1]=(*pCell)[k]->y;
					dVect1[2]=(*pCell)[k]->z;

					glVertex3dv(dVect1);
				}
			}

			glEnd();

			glBegin(GL_POINTS);
			glPointSize(1);
			colorIdx=tracker.colorIdx%8;
			glColor3d(ColorTableShot[colorIdx][0], ColorTableShot[colorIdx][1], ColorTableShot[colorIdx][2]);
        // 	glColor3d(0.3 , 0.3 , 0.3);
			for(j=0; j<gluData2.size();++j)
			{
				pCell=gluData2[j]->pCell;
				for(k=0; k<pCell->size();++k)
				{
					Point pp((*pCell)[k]->x,  (*pCell)[k]->y,  (*pCell)[k]->z);

					pp.transform(deltaMat);

					//dVect1[0]=(*pCell)[k]->x;
					//dVect1[1]=(*pCell)[k]->y;
					//dVect1[2]=(*pCell)[k]->z;

					dVect1[0]=pp.x;
			        dVect1[1]=pp.y;
			        dVect1[2]=pp.z;

					glVertex3dv(dVect1);
				}
			}
			glEnd();

            glLineWidth(3);
			//////////////////////
			glBegin(GL_LINES);

			glColor3d(1, 0, 0);

    		Point p1(glu1.avg_x, glu1.avg_y, glu1.avg_z);
			Point p2(glu2.avg_x, glu2.avg_y,glu2.avg_z);

			p2.transform(deltaMat);

			////////////////////////
			dVect1[0]=p1.x;
			dVect1[1]=p1.y;
			dVect1[2]=p1.z;
			glVertex3dv(dVect1);

			//dVect2[0]=p2.x;
			//dVect2[1]=p2.y;
			//dVect2[2]=p2.z;
			//glVertex3dv(dVect2);

		    glEnd();

		}
	}

	return 0;
}


int TrackerManager::DrawTrackersMovtion_Long(vector <VeloScan *> allScans)
{
	 int i,j,k,colorIdx;

	list<Tracker>::iterator it;
	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		Tracker &tracker=*it;
		if(tracker.missMatch == true)
			   continue;
		if(tracker.statusList.size() <2)
			   continue;
		if(tracker.dataList.size()  < 2)
			   continue;

		//	cerr << " object number  " << tracker.statusList.size() <<endl;
			for(int i =0; i <2 ;i ++)
			{
			    Scan *firstScan = allScans[0];
                Scan *CurrentScan = allScans[i];
				double  deltaMat[16];

				GetCurrecntdelteMat(*CurrentScan ,  *firstScan,  deltaMat);
				clusterFeature &glu1=tracker.statusList[i];
				cluster &gluData1=tracker.dataList[i];

				colorIdx=tracker.colorIdx%8;
		//		DrawObjectPoint(gluData1, 1,  0.3, 0.3, 0.3,deltaMat);
				DrawObjectPoint(gluData1, 1,
					ColorTableShot[colorIdx][0],
					ColorTableShot[colorIdx][1],
					ColorTableShot[colorIdx][2] ,
					deltaMat);

				if (glu1.size < 8) continue;
				Point p1(glu1.avg_x, glu1.avg_y, glu1.avg_z);
				p1.transform(deltaMat);
				DrawPoint(p1,8,1,0,0);

		    //cluster &gluData=scanRef.scanClusterArray[tracker.matchClusterID];
			//////////////////////
//		    glLineWidth(3);
//			glBegin(GL_LINES);
	//		if (glu2.size < 8) continue;
//			Point p2(glu2.avg_x, glu2.avg_y,glu2.avg_z);

//			DrawPoint(p2,8,1,1,0);
		   }
	}

	return 0;
}


int TrackerManager::DrawTrackersMovtion_Long_Number(vector <Scan *> allScans, int n)
{
	 int i,j,k,colorIdx;

	list<Tracker>::iterator it;
	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		Tracker &tracker=*it;
		if(tracker.missMatch == true)
			   continue;
		if(tracker.statusList.size() < n)
			   continue;
		if(tracker.dataList.size()  < n)
			   continue;

		//	cerr << " object number  " << tracker.statusList.size() <<endl;
			for(int i =0; i <n ;i ++)
			{
			    Scan *firstScan = allScans[0];
                Scan *CurrentScan = allScans[i];
				double  deltaMat[16];

				GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);
				clusterFeature &glu1=tracker.statusList[i];
				cluster &gluData1=tracker.dataList[i];

				colorIdx=tracker.colorIdx%8;
		//		DrawObjectPoint(gluData1, 1,  0.3, 0.3, 0.3,deltaMat);
				DrawObjectPoint(gluData1, 1,
					ColorTableShot[colorIdx][0],
					ColorTableShot[colorIdx][1],
					ColorTableShot[colorIdx][2] ,
					deltaMat);

				if (glu1.size < 8) continue;
				Point p1(glu1.avg_x, glu1.avg_y, glu1.avg_z);
				p1.transform(deltaMat);
				DrawPoint(p1,8,1,0,0);

		    //cluster &gluData=scanRef.scanClusterArray[tracker.matchClusterID];
			//////////////////////
//		    glLineWidth(3);
//			glBegin(GL_LINES);
	//		if (glu2.size < 8) continue;
//			Point p2(glu2.avg_x, glu2.avg_y,glu2.avg_z);

//			DrawPoint(p2,8,1,1,0);
		   }
	}

	return 0;
}


int TrackerManager::DrawTrackersMovtion_Long_Number_All(vector <Scan *> allScans, int n)
{
	 int i,j,k,colorIdx;

	list<Tracker>::iterator it;
	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		Tracker &tracker=*it;
	//	if(tracker.missMatch == true)
	//		   continue;
	//	if(tracker.statusList.size() < n)
	//		   continue;
	//	if(tracker.dataList.size()  < n)
	//		   continue;

		//	cerr << " object number  " << tracker.statusList.size() <<endl;
			for(int i =0;   i <tracker.statusList.size()-1;  i++ )
			{

			    Scan *firstScan = allScans[0];
                Scan *CurrentScan = allScans[i];
				Scan *CurrentScanNext = allScans[i+1];
				double  deltaMat[16];
				double  deltaMatNext[16];

				GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);
				GetCurrecntdelteMat(*CurrentScanNext , *firstScan,  deltaMatNext);

				clusterFeature &glu1=tracker.statusList[i];
				cluster &gluData1=tracker.dataList[i];
			    clusterFeature &glu2=tracker.statusList[i+1];
				cluster &gluData2=tracker.dataList[i+1];

				if (glu1.size < 8) continue;
				if (glu2.size < 8) continue;

				Point p1(glu1.avg_x, glu1.avg_y, glu1.avg_z);
				Point p2(glu2.avg_x, glu2.avg_y, glu2.avg_z);
				Point p1text(glu1.avg_x, glu1.avg_y+50, glu1.avg_z);
				Point p2text(glu2.avg_x, glu2.avg_y+50, glu2.avg_z);

				p1.transform(deltaMat);
				p2.transform(deltaMatNext);
				p1text.transform(deltaMat);
				p2text.transform(deltaMatNext);

				colorIdx=tracker.colorIdx%8;

				DrawObjectPoint(gluData1, 1,
					ColorTableShot[colorIdx][0],
					ColorTableShot[colorIdx][1],
					ColorTableShot[colorIdx][2] ,
					deltaMat);

				char objectID[256];
				sprintf(objectID, "%d", glu1.trackNO);
		//		DrawTextRGB(p1text, 0,1,0,objectID );

				//colorIdx=(tracker.colorIdx+1)%8;
				//DrawObjectPoint(gluData2, 1,
				//	ColorTableShot[colorIdx][0],
				//	ColorTableShot[colorIdx][1],
				//	ColorTableShot[colorIdx][2] ,
				//	deltaMatNext);

				sprintf(objectID, "%d", glu2.trackNO);
		//		DrawTextRGB(p2text, 0,1,0,objectID );

				DrawPoint(p1,8,1,0,0);
				DrawPoint(p2,8,1,0,0);

			   Draw_Line_GL_RGB(p1, p2, 3,	1, 0, 0, false);

		   }
	}

	return 0;
}


int TrackerManager::ClassifiyTrackersObjects(vector <Scan *> allScans, int currentNO ,int windowsize)
{
	 int i,j,k,colorIdx;

	list<Tracker>::iterator it;

 //    int clustersize=scanClusterArray.size();
	//	//Find moving Ojbects
	//for(i=0; i<clustersize; ++i)
	//{
 //    	clusterFeature &glu = scanClusterFeatureArray[i];
	//	glu.clusterType != CLUSTER_TYPE_OBJECT;
	//    if( FilterNOMovingObjcets(glu))
	//	{
	//			clusterFeature &gclu = 	scanClusterFeatureArray[i];
	//			gclu.clusterType  |=CLUSTER_TYPE_MOVING_OBJECT;
	//	}
	//}

	////Mark No Moving Ojbects CELLS
	//int k;
	//for(i=0; i<clustersize; ++i)
	//{
 //    	clusterFeature &glu = scanClusterFeatureArray[i];
	//	if( !(glu.clusterType & CLUSTER_TYPE_MOVING_OBJECT))
	//	{
	//			cluster  &gclu = 	scanClusterArray[i];
	//			for(j =0; j< gclu.size() ; ++j)
	//			{
	//				cellFeature &gcF = *(gclu[j]);
	//				gcF.cellType |= CELL_TYPE_FOR_SLAM6D;
	//			}
	//	}
	//
	//}

//	for(it=tracks.begin();  it!=tracks.end();  it++)
//	{
//		Tracker &tracker=*it;
//	//	if(tracker.missMatch == true)
//	//		   continue;
//	//	if(tracker.statusList.size() < n)
//	//		   continue;
//	//	if(tracker.dataList.size()  < n)
//	//		   continue;
//
//		//	cerr << " object number  " << tracker.statusList.size() <<endl;
//			for(int i =0;   i <tracker.statusList.size();  i++ )
//			{
//
//			    Scan *firstScan = allScans[0];
//                Scan *CurrentScan = allScans[i];
//				double  deltaMat[16];
//
//				GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);
//				clusterFeature &glu1=tracker.statusList[i];
//				cluster &gluData1=tracker.dataList[i];
//
//				colorIdx=tracker.colorIdx%8;
//		//		DrawObjectPoint(gluData1, 1,  0.3, 0.3, 0.3,deltaMat);
//				DrawObjectPoint(gluData1, 1,
//					ColorTableShot[colorIdx][0],
//					ColorTableShot[colorIdx][1],
//					ColorTableShot[colorIdx][2] ,
//					deltaMat);
//
//				if (glu1.size < 8) continue;
//				Point p1(glu1.avg_x, glu1.avg_y, glu1.avg_z);
//				p1.transform(deltaMat);
//				DrawPoint(p1,8,1,0,0);
//
//		    //cluster &gluData=scanRef.scanClusterArray[tracker.matchClusterID];
//			//////////////////////
////		    glLineWidth(3);
////			glBegin(GL_LINES);
//	//		if (glu2.size < 8) continue;
////			Point p2(glu2.avg_x, glu2.avg_y,glu2.avg_z);
//
////			DrawPoint(p2,8,1,1,0);
//		   }
//
//
//	}

	return 0;
}

void TrackerManager::TrackerManagerReset()
{
	clusterStatus.clear();
	//tracks.clear();
	list<Tracker>::iterator Iter;
	Iter=tracks.begin();
	for (;Iter!=tracks.end();Iter++)
	{
		Tracker temp=*Iter;
		temp.TrackerReset();
	}
	tracks.clear();
	preScan=0;
	return;
}

