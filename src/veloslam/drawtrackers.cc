/*
 * drawtrackers implementation
 *
 * Copyright (C) Li Wei, Li Ming, Andreas Nuechter,
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief 
 *
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
 */
#include <cstdlib>
#include <cstdio>
#include <cmath>

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

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

                    Scan *CurrentScan = allScans[firstNO];

    				double  deltaMat[16];
				    GetCurrecntdelteMat(*CurrentScan ,  deltaMat);

					p1.x = glu1.avg_x; p1.y= glu1.avg_y;p1.z=glu1.avg_z;
					p1text.x= glu1.avg_x+150; p1text.y= glu1.avg_y+80; p1text.z=glu1.avg_z+50;

    				p1.transform(deltaMat);
    				p1text.transform(deltaMat);

    				colorIdx=tracker.colorIdx%8;

                }
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

                Scan *CurrentScan = allScans[firstNO];
				Scan *CurrentScanNext = allScans[secondNO];

				double  deltaMat[16];
				double  deltaMatNext[16];

				GetCurrecntdelteMat(*CurrentScan ,   deltaMat);
				GetCurrecntdelteMat(*CurrentScanNext ,  deltaMatNext);

				p1.x = glu1.avg_x; p1.y= glu1.avg_y;p1.z=glu1.avg_z;
				p1text.x= glu1.avg_x+150; p1text.y= glu1.avg_y+80; p1text.z=glu1.avg_z+50;
				p2.x = glu2.avg_x; p2.y= glu2.avg_y;  p2.z=glu2.avg_z;
				p2text.x= glu2.avg_x+150; p2text.y= glu2.avg_y+80; p2text.z=glu2.avg_z+50;

				p1.transform(deltaMat);
				p2.transform(deltaMatNext);
				p1text.transform(deltaMat);
				p2text.transform(deltaMatNext);

				colorIdx=tracker.colorIdx%8;


				DrawPoint(p1,4,0,1,1);
				DrawPoint(p2,4,0,1,1);

			   Draw_Line_GL_RGB(p1, p2, 3,	1, 0, 0, false);

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

