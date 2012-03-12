#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "veloslam/color_util.h"
#include "veloslam/veloscan.h"
#include "veloslam/trackermanager.h"
#include "veloslam/debugview.h"

#include <GL/gl.h>		    	/* OpenGL header file */
#include <GL/glu.h>			/* OpenGL utilities header file */

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

TrackerManager::TrackerManager(void)
{
	colorIdx=0;
}


TrackerManager::~TrackerManager(void)
{
}

int TrackerManager::Init(void)
{
	return 0;
}

/** ����һ֡���ݣ�����ע����ǣ���һ֡�����Ѿ�������ͶӰ�ͼ�Ⱥ��������*/
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

	/**
	0.��ʼ����һ֡��״̬
	*/

	clusterStatus.resize(size);


	for(i=0; i<size;++i)
		clusterStatus[i].Matched=false;

	/**
	1.���������ǲ�����Ҫ���cluster
	*/
	FilterObject(scanRef);

	/**
	2.�������е�tracker
	*/
	UpdateTrackers(scanRef);

	/**
	3.��ʣ�µ�û��ƥ���ϵ�cluster����tracker �б�
	*/
	AddTrackers(scanRef);

	return 0;
}


int TrackerManager::AddTrackers(VeloScan& scanRef)
{
	int i;
	int size=scanRef.scanClusterArray.size();

	for(i=0; i<size;++i)
	{
		//�˶�Ŀ��������ṹ
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];
		if(clusterStatus[i].FilterRet==true&&clusterStatus[i].Matched==false)
		{
			Tracker newTracker;

			//��Ҫ��¼����һ֡��ʼ����
			newTracker.frameNO =scanRef.scanid;
			//Ŀ��������һ֡�е�
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
		//�˶�Ŀ��������ṹ
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];
		clusterStatus[i].FilterRet = TrackerManager::TrackerFilter(glu);
	}
	return 0;

}

/** ���ٹ���������һЩ�����ϸ��ٵ�clusterȥ��*/
bool TrackerManager::TrackerFilter(clusterFeature &glu)
{
	if(glu.size < 8)
		return false; // small object do not use it!

	return true; // no filter

	/**�����֮��**/
	if( glu.size_z > 200 && ((glu.size_x>glu.size_y?glu.size_x:glu.size_y))<360)
	{
		return false;
	}
	/**����3.5�׳�������С��1.4m�����Բ�����Ϊ��**/
	else if((glu.size_y>350 && glu.size_x<140)|| (glu.size_x>350 && glu.size_y<140))
	{
		return false;

	}
	else if(glu.size_z > 250 )
	{
		return false;
	}
	else if((glu.size_x>glu.size_y?glu.size_x:glu.size_y)>420 && glu.size_z<130)
	{
		return false;
	}

	else if((glu.size_x>glu.size_y?glu.size_x:glu.size_y)>3.5 && ((glu.size_x>glu.size_y?glu.size_x:glu.size_y)/(glu.size_x<glu.size_y?glu.size_x:glu.size_y)>4))
	{
		return false;
	}

	else if(glu.size_x<700 && glu.size_y<700 &&  glu.size_z > 100  )
	{

		return true;
	}

	// ���˵�����15�׵� �����������϶��ǽ�����
	if(glu.size_x>1500 || glu.size_y>1500 || glu.size_x*glu.size_y >600*600 )
	{
		return false;
	}

	// ���˹�����ر��� �ر�ϸ���Ĵ����壬������դ��
	if(glu.size_x*glu.size_y >500*500  &&  glu.size_x/glu.size_y < 1.5)
	{
		return false;
	}

	//���˵�1�׶�һ�µ� СĿ��	
	if(glu.size_z < 100)
	{
		return false;
	}
	//���˵�������ܺ�С��3�׵� //�п���������
	if((glu.size_x + glu.size_y + glu.size_z)<1.5)
	{
		return false;
	}

	// ���˵�������ȳ���3�׵� 
	if(glu.size_y>300)
	{
		return false;
	}

	//if(glu.size_x<40||glu.size_y<40||glu.size_z<40)
	//	continue;

	//�п���������
	if((glu.size_x + glu.size_y) <4)
	{
		return false;
	}

	//���˵�����ȴ���3��  //�кܶ๫�������������
	if(glu.size_x/glu.size_y>3.0)
	{
		return false;
	}

	return true;
}


int TrackerManager::MatchTrackers(VeloScan& scanRef,Tracker& tracker)
{
	int i;
	int minID=-1;
	int minValue=360;
	int value;
	float radiusDiff;
	float thetaDiff;
	float sizeDiff;
	float positionDiff;

	/** �ڲ����ClusterArray*/
	for(i=0; i<scanRef.scanClusterArray.size();++i)
	{
		//�˶�Ŀ��������ṹ
		clusterFeature &glu=scanRef.scanClusterFeatureArray[i];
		cluster &gluData=scanRef.scanClusterArray[i];

		// ȥ�����˺��Ŀ�꣬���μ�ƥ�����
		if(clusterStatus.size()!=0   &&  clusterStatus[i].FilterRet==false)
			continue;

		if(tracker.statusList.size())
		{
			//�����ŵ��趨.
			radiusDiff=fabs(tracker.statusList.back().radius-glu.radius);
			thetaDiff=fabs(tracker.statusList.back().theta-glu.theta);
			sizeDiff =abs(tracker.statusList.back().size-glu.size);
			positionDiff = sqrt(sqr(tracker.statusList.back().avg_x -glu.avg_x) + sqr(tracker.statusList.back().avg_z -glu.avg_z) ) ;
			
	//		cerr <<   tracks.size() << "   " << tracker.frameNO << "   " <<tracker.matchClusterID <<"   " <<tracker.colorIdx <<endl;
	//		cerr <<  tracker.statusList.back().theta << "   " <<glu.theta <<"   " << thetaDiff <<"   "
	//			    <<tracker.statusList.back().radius << "   " <<glu.radius <<"   " <<radiusDiff<<"   "
	//				<<tracker.statusList.back().size << "   " <<glu.size <<"   " <<sizeDiff <<"   "
	//				<<tracker.statusList.back().avg_x << "   " <<glu.avg_x <<"   " <<positionDiff <<"  "
	//				<<tracker.statusList.back().avg_z << "   " <<glu.avg_z <<"   " <<positionDiff <<endl;
		//	if(radiusDiff<=12.0 && thetaDiff<=12.0 && sizeDiff <=8 )

  			if(radiusDiff<=12.0 && thetaDiff<=12.0 )
		//	if(radiusDiff<=12.0 && thetaDiff<=12.0 && sizeDiff <=8 && positionDiff <=800)
			{
			//	value= radiusDiff*1.0 + thetaDiff*1.0  +  sizeDiff*0.8 + positionDiff * 0.03 ;
				value= radiusDiff*1.0 + thetaDiff*1.0  +  sizeDiff*0.8 ;
				if(value<minValue)
				{
					minValue=value;
					minID=i;
				}

			}

		}

	}
	return minID;
}


int TrackerManager::UpdateTrackers(VeloScan& scanRef)
{
	int i,j;
	int matchID;

	/** ������tracks*/
	list<Tracker>::iterator it;
	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		Tracker &tracker=*it;
		//���µ�һ��Ŀ��ô�����������һ��������
		matchID=MatchTrackers(scanRef,  tracker); 

		if(matchID==-1)
		{
			tracker.missMatch=true;
			//	TRACE("Delete tracker %d\n",tracker.colorIdx);
		}
		else
		{
			// save feature
			tracker.statusList.push_back(scanRef.scanClusterFeatureArray[matchID]);
			// save data
			tracker.dataList.push_back(scanRef.scanClusterArray[matchID]);
			// ��¼ ID
			tracker.matchClusterID=matchID;
			tracker.missMatch=false;
			tracker.Matched = true;

			clusterStatus[matchID].Matched=true;
			//	TRACE("Match  tracker %d,len:%d\n",tracker.colorIdx,tracker.statusList.size());
		}

	}
	//tracks.remove_if();
	return 0;
}

/** ����һ֡��cluster������*/
int TrackerManager::DrawScanCluster(VeloScan& scanRef)
{
	int i,j,k,colorIdx;
	GLdouble dVect1[3];
//	glPointSize(1);

//	glBegin(GL_POINTS);
	for(i=0; i<scanRef.scanClusterArray.size();++i)
	{
		//�˶�Ŀ��������ṹ
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
		//�ҵ�һ��cluster�󣬱���cell��ʾ�����еĵ�
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
		//�˶�Ŀ��������ṹ
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

		//�ҵ�һ��cluster�󣬱���cell��ʾ�����еĵ�
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
		//�˶�Ŀ��������ṹ
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

		//�ҵ�һ��cluster�󣬱���cell��ʾ�����еĵ�
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
			//�ҵ�һ��cluster�󣬱���cell��ʾ�����еĵ�
		   }
	}

	return 0;
}


int TrackerManager::DrawTrackersMovtion_Long_Number(vector <VeloScan *> allScans, int n)
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
			//�ҵ�һ��cluster�󣬱���cell��ʾ�����еĵ�
		   }
	}

	return 0;
}
