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

extern int sliding_window_size;
extern int current_sliding_window_pos;
extern Trajectory VelodyneTrajectory;
extern VeloScan * g_pfirstScan;

int TrackerManager::DrawTrackersMovtion_Long_Number_All(vector <Scan *> allScans, int n)
{
	int i,j,k,colorIdx;
    Point p1,p2,p1text,p2text;
 	char object_moving_distance[256];

	list<Tracker>::iterator it;
	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		    Tracker &tracker=*it;
            int size=tracker.statusList.size();
			int firstNO= -1;
			int secondNO= -1;
     //     cout << "tracker number " << tracks.size() <<endl;
            //////////////////////////////////////////////////////////

			if (tracker.moving_distance<constant_static_or_moving||size==0)
			{
				continue;
			}

			if (size < 3)
			{
                for(int i =0;  i <size;  i++ )
			    {
                    clusterFeature &glu1=tracker.statusList[i];
    				cluster &gluData1=tracker.dataList[i];

    			    firstNO = GetScanID_in_SlidingWindow(glu1.frameNO,
                                     current_sliding_window_pos,
                                     sliding_window_size);
     			    if(firstNO <0)
    				    continue;

    			    Scan *firstScan = (Scan *)g_pfirstScan;
    			//    Scan *firstScan = allScans[0];
                    Scan *CurrentScan = allScans[firstNO];

    				double  deltaMat[16];
    				double  deltaMatNext[16];

    				GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);

					p1.x = glu1.avg_x; p1.y= glu1.avg_y;p1.z=glu1.avg_z;
					p1text.x= glu1.avg_x+150; p1text.y= glu1.avg_y+80; p1text.z=glu1.avg_z+50;

    				p1.transform(deltaMat);
    				p1text.transform(deltaMat);

    				colorIdx=tracker.colorIdx%8;

        //            if(tracker.moving_distance < constant_static_or_moving)
    				//{
    				//	DrawObjectPoint(gluData1, 1,
    				//			0.2,
    				//			0.2,
    				//			0.2,
    				//			deltaMat);
    				//}
        //            else
        //   			{
    				//	  DrawObjectPoint(gluData1, 1,
        //						ColorTableShot[colorIdx][0],
        //						ColorTableShot[colorIdx][1],
        //						ColorTableShot[colorIdx][2] ,
    				//			deltaMat);
    				//}

                }
			 //     if(firstNO <0)
    		//		    continue;

         //           sprintf(object_moving_distance, "%d %d %4.2f ",tracker.trackerID, tracker.matchClusterID,  tracker.moving_distance);
          //          DrawTextRGB(p1text, 1, 0, 0, object_moving_distance );
           //         DrawPoint(p1,4,0,0,1);
                    continue;
    		}
            ////////////////////////////////////////////////////////////

			for(int i =0;   i <size-2;  i++ )
			{
				clusterFeature &glu1=tracker.statusList[i];
				cluster &gluData1=tracker.dataList[i];
			    clusterFeature &glu2=tracker.statusList[i+1];
				cluster &gluData2=tracker.dataList[i+1];

			    firstNO = GetScanID_in_SlidingWindow(glu1.frameNO,
                                 current_sliding_window_pos,
                                 sliding_window_size);
                secondNO = GetScanID_in_SlidingWindow(glu2.frameNO,
                                 current_sliding_window_pos,
                                 sliding_window_size);
 			   if(firstNO <0 || secondNO< 0 )
				    continue;

			    Scan *firstScan = (Scan *)g_pfirstScan;
//			    Scan *firstScan = allScans[0];
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

				p1.transform(deltaMat);
				p2.transform(deltaMatNext);
				p1text.transform(deltaMat);
				p2text.transform(deltaMatNext);

				colorIdx=tracker.colorIdx%8;

           /*     if(tracker.moving_distance < constant_static_or_moving)
				{
					DrawObjectPoint(gluData1, 1,
							0.2,
							0.2,
							0.2,
							deltaMat);
					DrawObjectPoint(gluData2, 1,
							0.2,
							0.2,
							0.2,
							deltaMat);
				}
                else
       			{
					  DrawObjectPoint(gluData1, 1,
    						ColorTableShot[colorIdx][0],
    						ColorTableShot[colorIdx][1],
    						ColorTableShot[colorIdx][2] ,
							deltaMat);

					  DrawObjectPoint(gluData2, 1,
    						ColorTableShot[colorIdx][0],
    						ColorTableShot[colorIdx][1],
    						ColorTableShot[colorIdx][2] ,
							deltaMat);
				}*/



			//	char objectID[256];
		//		sprintf(objectID, "%d", glu1.trackNO);
			//	DrawTextRGB(p1text, 0,1,0, objectID );

				//colorIdx=(tracker.colorIdx+1)%8;
				//DrawObjectPoint(gluData2, 1,
				//	ColorTableShot[colorIdx][0],
				//	ColorTableShot[colorIdx][1],
				//	ColorTableShot[colorIdx][2] ,
				//	deltaMatNext);

			//	sprintf(objectID, "%d", glu2.trackNO);
			//	DrawTextRGB(p2text, 0,1,0,objectID );

				DrawPoint(p1,4,0,1,1);
				DrawPoint(p2,4,0,1,1);

			   Draw_Line_GL_RGB(p1, p2, 3,	1, 0, 0, false);

		   }
 		 //     if(firstNO <0)
		//	    continue;

	//		sprintf(object_moving_distance, "%d %d %4.2f ",tracker.trackerID , tracker.matchClusterID, tracker.moving_distance);
	//		DrawTextRGB(p1text, 0, 0, 1, object_moving_distance );
	}

	return 0;
}


int TrackerManager::DrawTrackersContrailAfterFilted(vector<Scan *> allScans)
{
	list<Tracker>::iterator it;
	for(it=tracks.begin();  it!=tracks.end();  it++)
	{
		Tracker &tracker=*it;
		int size=tracker.moveStateList.size();
		int firstNO= -1;
		int secondNO= -1;

		if (size<3||tracker.moving_distance<constant_static_or_moving)
		{
			continue;
		}

		for (int i=0;i<size-1;i++)
		{
			firstNO = GetScanID_in_SlidingWindow(tracker.moveStateList[i].frameNo,
				current_sliding_window_pos,
				sliding_window_size);
			secondNO = GetScanID_in_SlidingWindow(tracker.moveStateList[i+1].frameNo,
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

			clusterFeature &glu1=tracker.statusList[i];
			cluster &gluData1=tracker.dataList[i];
			clusterFeature &glu2=tracker.statusList[i+1];
			cluster &gluData2=tracker.dataList[i+1];

			if (glu1.size < 8) continue;
			if (glu2.size < 8) continue;

			Point p1(tracker.moveStateList[i].targetState.x_position, glu1.avg_y,tracker.moveStateList[i].targetState.z_positon);
			Point p2(tracker.moveStateList[i+1].targetState.x_position, glu2.avg_y, tracker.moveStateList[i+1].targetState.z_positon);
			Point p1text(tracker.moveStateList[i].targetState.x_position, glu1.avg_y+50,tracker.moveStateList[i].targetState.z_positon);
			Point p2text(tracker.moveStateList[i+1].targetState.x_position, glu2.avg_y+50,tracker.moveStateList[i+1].targetState.z_positon);

			p1.transform(deltaMat);
			p2.transform(deltaMatNext);
			p1text.transform(deltaMat);
			p2text.transform(deltaMatNext);

			DrawPoint(p1,4,1,0,1);
			DrawPoint(p2,4,1,0,1);

			Draw_Line_GL_RGB(p1,p2,2,1,0,1,false);
		}

	}

	return 0;
}


int TrackerManager::DrawEgoTrajectory()
{
   	int i, size;
    size =VelodyneTrajectory.path.size();
	for(i=0; i< size; i++)
	{
        Point p = VelodyneTrajectory.path[i];
 		DrawPoint(p,4,0,1,0);
     }

	return 0;
}

