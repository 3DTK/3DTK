/*
 * veloscan implementation
 *
 * Copyright (C) Andreas Nuechter, Li Wei, Li Ming
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Implementation of a 3D scan and of 3D scan matching in all variants
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 */

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <sstream>
using std::stringstream;

#include "veloslam/veloscan.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <cstring>
using std::flush;

#include "slam6d/Boctree.h"
#include "veloslam/veloscan.h"
#include "veloslam/pcddump.h"
#include "veloslam/trackermanager.h"
#include "veloslam/clusterboundingbox.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <cstring>
using std::flush;

#include <GL/gl.h>
#include <GL/glu.h>

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif
#include "veloslam/velodefs.h"
#include "veloslam/color_util.h"

int scanCount =0;
TrackerManager trackMgr;
float absf(float a)
{
	return a>0?a:-a;
}

Trajectory::Trajectory()
{

}


/**
 * default Constructor
 */
VeloScan::VeloScan()
//    : BasicScan()
{
    isTrackerHandled =false;
}

/**
 * Desctuctor
 */
VeloScan::~VeloScan()
{
	FreeAllCellAndCluterMemory();
}

int VeloScan::DeletePoints()
{
	FreeAllCellAndCluterMemory();
	return 0;
}

/**
 * Copy constructor
 */
/*
VeloScan::VeloScan(const VeloScan& s)
    : BasicScan(s)
{ }
*/

int VeloScan::TransferToCellArray(int maxDist, int minDist)
{
#define  DefaultColumnSize 360
    Point P;
    DataXYZ xyz(get("xyz"));
    int size= xyz.size();

    int columnSize= 360;	//cfg.cfgPlaneDetect.ColumnSize;
    int CellSize= 50;	    //cfg.cfgPlaneDetect.CellSize;

    int MinRad=minDist;     //cfg.cfgPlaneDetect.MinRad;
    int MaxRad=maxDist;      //cfg.cfgPlaneDetect.MaxRad

    if((MaxRad-MinRad)%CellSize!=0)
        CellSize=10;

    int i,j,count=0;
    int CellNumber=(MaxRad-MinRad)/CellSize;
    float flag;
    int offset;

    if(columnSize==0)
        return -1;

    if(columnSize%360!=0)
        columnSize=DefaultColumnSize;

    int sectionSize=columnSize/8;

    scanCellArray.resize(columnSize);
    for(i=0; i<columnSize; ++i)
        scanCellArray[i].resize(CellNumber);

    float inc=(M_PI*2)/columnSize;

    vector<float> tanv;
    vector<float>::iterator result;

    for(i=0; i<sectionSize; ++i)
    {
        tanv.push_back(tan(inc*i));
    }

    int diff;

    for(i=0; i<size; ++i)
    {
            count++;
            Point pt;
            pt.x = xyz[i][0];
            pt.y = xyz[i][1];
            pt.z = xyz[i][2];

			pt.point_id = i;  //important   for find point in  scans  ---raw points

            pt.rad = sqrt(pt.x*pt.x + pt.z*pt.z);
            pt.tan_theta = pt.z/pt.x ;

            if(pt.rad <=MinRad || pt.rad>=MaxRad)
                continue;

            // some point losted which on the vline or hline
            if(pt.x >=0 && pt.z>=0 )
            {
                if(pt.x > pt.z)
                {
                    flag= pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize-1;
                    }
                    else
                    {
                        offset=result-tanv.begin();
                    }
                }
                else
                {
                    flag=1/pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize;
                    }
                    else
                    {
                        diff=result-tanv.begin();
                        offset=sectionSize*2-1-(diff);
                    }

                }
            }

            else if(pt.x <= 0 && pt.z >=0)
            {
                if(-pt.x>pt.z)
                {
                    flag=-pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize*3;
                    }
                    else
                    {
                        offset=sectionSize*4-1-(result-tanv.begin());
                    }

                }
                else
                {
                    flag=1/-pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize*3-1;
                    }
                    else
                    {
                        offset=sectionSize*2+(result-tanv.begin());
                    }
                }
            }

            else if(pt.x<=0 && pt.z<=0)
            {
                if(-pt.x>-pt.z)
                {
                    flag=pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize*5-1;
                    }
                    else
                    {
                        offset=sectionSize*4+(result-tanv.begin());
                    }
                }
                else
                {
                    flag=1/pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize*5;
                    }
                    else
                    {
                        offset=sectionSize*6-1-(result-tanv.begin());
                    }

                }
            }

            else if(pt.x>=0&&pt.z<=0)
            {
                if(pt.x>-pt.z)
                {
                    flag=-pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize*7;
                    }
                    else
                    {
                        offset=sectionSize*8-1-(result-tanv.begin());
                    }
                }
                else
                {
                    flag=1/-pt.tan_theta;
                    result=upper_bound(tanv.begin(),tanv.end(),flag);
                    if(result==tanv.end())
                    {
                        offset=sectionSize*7-1;
                    }
                    else
                    {
                        offset=sectionSize*6+(result-tanv.begin());
                    }
                }
            }

            else
            {
                continue;
            }

            int k= (int)((pt.rad-MinRad)/(CellSize*1.0));
            scanCellArray[offset][k].push_back(&pt);
    }
	return 0;
}



int VeloScan::CalcCellFeature(cell& cellobj, cellFeature& f)
{
	   int outlier;
	   float lastMaxY;
	   f.size=cellobj.size();
	   f.cellType=0;

	   if(f.size==0)
	   {
		   f.cellType|=CELL_TYPE_INVALID;
		   return 0;
	   }

	   f.ave_x= f.ave_y = f.ave_z=0.0;
	   f.delta_y=0;

	   int i=0;
	   for(i=0; i<f.size; ++i)
	   {
		   f.ave_x+=cellobj[i]->x;
		   f.ave_z+=cellobj[i]->z;
		   f.ave_y+=cellobj[i]->y;

		//   if(cellobj[i]->type & POINT_TYPE_BELOW_R)
		//	   f.cellType |=CELL_TYPE_BELOW_R;

		   if(i==0)
		   {
			   outlier=0;
			   f.min_x=f.max_x=cellobj[i]->x;
			   f.min_z=f.max_z=cellobj[i]->z;
			   lastMaxY=f.min_y=f.max_y=cellobj[i]->y;
		   }
		   else
		   {
			   if(f.max_x<cellobj[i]->x)	f.max_x=cellobj[i]->x;

			   if(f.min_x>cellobj[i]->x)	f.min_x=cellobj[i]->x;

			   if(f.max_z<cellobj[i]->z)	f.max_z=cellobj[i]->z;

			   if(f.min_z>cellobj[i]->z)    f.min_z=cellobj[i]->z;

			   if(f.max_y<cellobj[i]->y)
			   {
				   lastMaxY=f.max_y;
				   f.max_y=cellobj[i]->y;
				   outlier=i;
			   }

			   if(f.min_y>cellobj[i]->y)
				   f.min_y=cellobj[i]->y;

		   }
	   }

	   if(f.size>1)
	   {
		   int y=f.ave_y-cellobj[outlier]->y;
		   y/=(f.size-1)*1.0;

		   if(cellobj[outlier]->y-y<50)
		   {
			   outlier=-1;
			   f.ave_y/=f.size*1.0;
		   }
		   else
		   {
			   f.max_y=lastMaxY;
			   f.ave_y=y;
		   }
	   }
	   else
	   {
		   outlier=-1;
		   f.ave_y/=f.size*1.0;
	   }

	   f.ave_x/=f.size*1.0;
	   f.ave_z/=f.size*1.0;

	   for(i=0; i<f.size; ++i)
	   {
		   if(i==outlier)
			   continue;
		   f.delta_y+= absf(cellobj[i]->y - f.ave_y);
	   }

	   float threshold;
   	   threshold=f.delta_y;

	   float GridThresholdGroundDetect =120;
	   if( threshold >  GridThresholdGroundDetect)
		   f.cellType =CELL_TYPE_STATIC;
	   else
		   f.cellType =CELL_TYPE_GROUND;

    return 0;
}


int VeloScan::CalcScanCellFeature()
{
    int i,j;

    if( scanCellArray.size()==0)
        return -1;

    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

    if( scanCellFeatureArray.size()==0)
    {
        scanCellFeatureArray.resize(columnSize);
        for(i=0; i<columnSize; ++i)
            scanCellFeatureArray[i].resize(cellNumber);
    }

    for(j=0; j <columnSize; j++)
    {
        cellColumn &column=scanCellArray[j];
        cellNumber=column.size();
        for( i=0; i<cellNumber; i++)
        {
            cell &cellObj=scanCellArray[j][i];
            cellFeature &feature=scanCellFeatureArray[j][i];

			feature.columnID=j;
			feature.cellID=i;

            feature.pCell=&cellObj;
            CalcCellFeature(cellObj,feature);

//	         if( feature.delta_y > 120)
//             {
//				 scanCellFeatureArray[j][i].cellType |= CELL_TYPE_STATIC;
//				 for(int k=0;k <scanCellArray[j][i].size(); k++)
//					 scanCellArray[j][i][k]->type |= POINT_TYPE_ABOVE_DELTA_Y;
//
//			 }

        }
    }

    return 0;
}

int VeloScan::SearchNeigh(cluster& clu,charvv& flagvv,int i,int j)
{
    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

	if(i==-1)
		i= columnSize-1;

	if(i==columnSize)
		i= 0;


    if(i<0||i>=columnSize||j<0||j>=cellNumber)
        return 0;

    if(flagvv[i][j]==1)
        return 0;
    if(scanCellFeatureArray[i][j].size==0)
    {
        flagvv[i][j]=1;
        return 0;
    }

    if(scanCellFeatureArray[i][j].cellType & CELL_TYPE_STATIC)
    {
        flagvv[i][j]=1;

    	clu.push_back(&scanCellFeatureArray[i][j]);

        SearchNeigh(clu,flagvv,i-1,j-1);
        SearchNeigh(clu,flagvv,i-1,j);
        SearchNeigh(clu,flagvv,i-1,j+1);
        SearchNeigh(clu,flagvv,i,j-1);
        SearchNeigh(clu,flagvv,i,j+1);
        SearchNeigh(clu,flagvv,i+1,j-1);
        SearchNeigh(clu,flagvv,i+1,j);
        SearchNeigh(clu,flagvv,i+1,j+1);

		SearchNeigh(clu,flagvv,i,j+2);
		SearchNeigh(clu,flagvv,i,j-2);
		SearchNeigh(clu,flagvv,i+2,j);
		SearchNeigh(clu,flagvv,i-2,j);

		//SearchNeigh(clu,flagvv,i,j+3);
		//SearchNeigh(clu,flagvv,i,j-3);
		//SearchNeigh(clu,flagvv,i+3,j);
		//SearchNeigh(clu,flagvv,i-3,j);
    }

    return 0;
}

int VeloScan::CalcClusterFeature(cluster& clu, clusterFeature& f)
{
    f.size=clu.size();

    if(f.size==0)
    {
        return 0;
    }

    f.clusterType=0;
	f.pointNumber=0;

	f.avg_x =0;  	f.avg_y=0;		f.avg_z=0;

    int i=0;
    for(i=0; i<f.size; ++i)
    {
		f.avg_x += clu[i]->ave_x;
		f.avg_y += clu[i]->ave_y;
		f.avg_z += clu[i]->ave_z;

        if(i==0)
        {
            f.min_x=f.max_x=clu[i]->min_x;
 		    f.min_z=f.max_z=clu[i]->min_z;
            f.min_y=f.max_y=clu[i]->min_y;
        }
        else
        {
            if(f.max_x<clu[i]->max_x)
                f.max_x=clu[i]->max_x;

            if(f.min_x>clu[i]->min_x)
                f.min_x=clu[i]->min_x;

            if(f.max_z<clu[i]->max_z)
                f.max_z=clu[i]->max_z;

            if(f.min_z>clu[i]->min_z)
                f.min_z=clu[i]->min_z;

            if(f.max_y<clu[i]->max_y)
                f.max_y=clu[i]->max_y;

            if(f.min_y>clu[i]->min_y)
                f.min_y=clu[i]->min_y;

        }
		f.pointNumber+=clu[i]->size;
		f.theta  += clu[i]->size*clu[i]->columnID;
		f.radius  += clu[i]->size*clu[i]->cellID;
    }

    f.size_x=f.max_x-f.min_x;
	f.size_z=f.max_z-f.min_z;
	f.size_y=f.max_y-f.min_y;

	f.avg_x /= f.size;
	f.avg_y /= f.size;
	f.avg_z /= f.size;

	f.theta=f.theta / (f.pointNumber*1.0);
	f.radius=f.radius / (f.pointNumber*1.0);

    return 0;
}

int VeloScan::FindAndCalcScanClusterFeature()
{
    int i,j;

    if( scanCellArray.size()==0)
        return -1;

    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

    charvv searchedFlag;
    searchedFlag.resize(columnSize);
    for(i=0; i<columnSize; ++i)
        searchedFlag[i].resize(cellNumber,0);

    for(i=0; i<columnSize; ++i)
    {
        for(j=0; j<cellNumber; ++j)
        {
            cluster clu;
            SearchNeigh(clu,searchedFlag,i,j);
            if(clu.size())
                scanClusterArray.push_back(clu);
        }
    }

	int clustersize=scanClusterArray.size();
	if(scanClusterFeatureArray.size()==0)
		scanClusterFeatureArray.resize(clustersize);

	cout<<"clusterSize:"<<clustersize<<endl;
	for(i=0; i<clustersize; ++i)
	{
		CalcClusterFeature(scanClusterArray[i],scanClusterFeatureArray[i]);
//		BoundingBox clusterBox;
//		clusterBox.CalBestRectangleBox(scanClusterArray[i],scanClusterFeatureArray[i]);
	}
	return 0;
}

void VeloScan::FreeAllCellAndCluterMemory()
{
    int i,j;
    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

    for(j=0; j <columnSize; j++)
    {
        cellColumn &column=scanCellArray[j];
        cellFeatureColumn &columnFeature=scanCellFeatureArray[j];

        cellNumber=column.size();
        for( i=0; i<cellNumber; i++)
        {
            cell &cellObj=scanCellArray[j][i];
            cellFeature &feature=scanCellFeatureArray[j][i];
            cellObj.clear();

        }
        column.clear();
        columnFeature.clear();
    }

	scanCellArray.clear();
	scanCellFeatureArray.clear();
    int ClusterSize=scanClusterArray.size();
    for(j=0; j <ClusterSize; j++)
    {
        cluster &cludata=scanClusterArray[j];
        clusterFeature &clu=scanClusterFeatureArray[j];
        cludata.clear();
  //      clu.clear();
    }
	scanClusterArray.clear();
	scanClusterFeatureArray.clear();
}

void VeloScan::calcReducedPoints_byClassifi(double voxelSize, int nrpts, PointType pointtype)
{
     // only copy the points marked POINT_TYPE_STATIC_OBJECT
	int realCount =0;

    // get xyz to start the scan load, separated here for time measurement
    DataXYZ xyz(get("xyz"));
    DataType Pt(get("type"));

    // if the scan hasn't been loaded we can't calculate anything
    if(xyz.size() == 0)
       throw runtime_error("Could not calculate reduced points, XYZ data is empty");

	for (int i = 0;  i < xyz.size(); i++)
	{
		if( Pt[i] &  POINT_TYPE_STATIC_OBJECT)
		{
			realCount++;
		}
    }

	// load  only static part of scans in point_red for icp6D
    DataXYZ xyz_t(create("xyz reduced", sizeof(double)*3*realCount));
    realCount=0;
	int j=0;
	for (int i = 0;  i < xyz.size(); i++)
	{
		if( Pt[i] &  POINT_TYPE_STATIC_OBJECT)
		{
			xyz_t[j][0] = xyz[i][0];
			xyz_t[j][1] = xyz[i][1];
			xyz_t[j][2] = xyz[i][2];
			j++;
			realCount++;
		}
    }

	// build octree-tree from CurrentScan
    // put full data into the octtree
    BOctTree<double> *oct = new BOctTree<double>(PointerArray<double>(xyz_t).get(),
      xyz_t.size(), reduction_voxelSize, reduction_pointtype);

    vector<double*> center;
    center.clear();

    if (reduction_nrpts > 0) {
      if (reduction_nrpts == 1) {
        oct->GetOctTreeRandom(center);
      } else {
        oct->GetOctTreeRandom(center, reduction_nrpts);
      }
    } else {
      oct->GetOctTreeCenter(center);
    }

    // storing it as reduced scan
    unsigned int size = center.size();
    DataXYZ xyz_r(create("xyz reduced", sizeof(double)*3*size));
    for(unsigned int i = 0; i < size; ++i) {
      for(unsigned int j = 0; j < 3; ++j) {
        xyz_r[i][j] = center[i][j];
      }
    }

    delete oct;
}

bool findBusCluster(clusterFeature &glu,  cluster &gluData)
{
  /*  double labelSVM;
	for(int i=0;i<360;i++)
	{
		nod[i].index=i+1;
		nod[i].value=intersectionFeature[i].slashLength/slashMaxLength;
	}
	nod[360].index=-1;
	labelSVM= svm_predict(m,nod);

	ofstream output;
	output.open("intersection.txt");
	output<<"labelSVM:"<<labelSVM<<endl;
	for(int j=0;j<360;j++)
		output<<j<<":"<<"  "<<nod[j].value;
	output.close();

	if(labelSVM>0.5)
		cout<<"intersection"<<endl;
	else
		cout<<"segment"<<endl;
	*/
	return 0;
}

//long objcount =0;
// In one scans find which the more like moving object  such as  pedestrian,  car,  bus.
bool FilterNOMovingObjcets(clusterFeature &glu,  cluster &gluData)
{
	// small object do not use it!
	if(glu.size <3)
		return false;
    if(glu.size_x > 800 ||  glu.size_z > 800 )
   	{
		return false;
	}
	return true;
    // no filter

	//char  filename[256];
	//string file;
	//sprintf(filename,"c:\\filename%d.txt", objcount);
	//file =filename;
	//DumpPointtoFile(gluData, file);
	//DumpFeaturetoFile(glu, "c:\\feature");
	//objcount++;
}

//long objcount =0;
// In one scans find which the more like moving object  such as  pedestrian,  car,  bus.
bool FindMovingObjcets(clusterFeature &glu,  cluster &gluData)
{
	// for debug moving object detections.

	   if( glu.size_y > 200 && ((glu.size_x>glu.size_y?glu.size_x:glu.size_y))<360)
	   {
		   return false;
	   }
	   else if((glu.size_y>350 && glu.size_x<140)|| (glu.size_x>350 && glu.size_y<140))
	   {
		   return false;
	   }
	   else if(glu.size_y > 250 )
	   {
		   return false;
	   }
	   else if((glu.size_x>glu.size_y?glu.size_x:glu.size_y)>420 && glu.size_z<130)
	   {
		   return false;
	   }
	   else if((glu.size_x>glu.size_y?glu.size_x:glu.size_y)>3.5
		   && ((glu.size_x>glu.size_y?glu.size_x:glu.size_y)/(glu.size_x<glu.size_y?glu.size_x:glu.size_y)>4))
	   {
		   return false;
	   }
	   else if(glu.size_x<700 && glu.size_z<700 &&  glu.size_y > 100  )
	   {
		   return true;
	   }
	   if(glu.size_x>1500 || glu.size_z>1500 || glu.size_x*glu.size_z >600*600 )
	   {
		   return false;
	   }
	   if (glu.size_x*glu.size_z > 500*500  &&  glu.size_x/glu.size_z < 1.5)
	   {
		   return false;
	   }
	   if(glu.size_y < 100)
	   {
		   return false;
	   }
	   if((glu.size_x + glu.size_y + glu.size_z)<1.5)
	   {
		   return false;
	   }
	   if(glu.size_z>700)
	   {
		   return false;
	   }
	   if(glu.size_x>700)
	   {
		   return false;
	   }
	   if(( glu.size_x  +  glu.size_z)  <4)
	   {
		   return false;
	   }
	   if( glu.size_x/glu.size_z> 3.0)
	   {
		   return false;
	   }

	   return true;
}


// bi classification for distigushed the moving or static
void VeloScan::ClassifiAllObject()
{
    int i,j;
    int clustersize=scanClusterArray.size();

	//Find moving Ojbects
	for(i=0; i<clustersize; ++i)
	{
     	clusterFeature &glu = scanClusterFeatureArray[i];
		cluster &gluData=scanClusterArray[i];

	    if( FilterNOMovingObjcets(glu,gluData))
		{
               if(FindMovingObjcets(glu,gluData))
				   glu.clusterType  =CLUSTER_TYPE_MOVING_OBJECT;
               else
				   glu.clusterType  =CLUSTER_TYPE_STATIC_OBJECT;
		}
        else
        {
				   glu.clusterType  =CLUSTER_TYPE_STATIC_OBJECT;
        }
	}

	//Mark No Moving Ojbects CELLS
	int k;
	for(i=0; i<clustersize; ++i)
	{
     	clusterFeature &glu = scanClusterFeatureArray[i];
		if(glu.clusterType & CLUSTER_TYPE_MOVING_OBJECT)
		{
				cluster  &gclu = 	scanClusterArray[i];
			    for(j =0; j< gclu.size() ; ++j)
				{
					cellFeature &gcF = *(gclu[j]);
					gcF.cellType = CELL_TYPE_MOVING;
				}
		}
        if(glu.clusterType & CLUSTER_TYPE_STATIC_OBJECT)
	    {
				cluster  &gclu = 	scanClusterArray[i];
				for(j =0; j< gclu.size() ; ++j)
				{
					cellFeature &gcF = *(gclu[j]);
					gcF.cellType = CELL_TYPE_STATIC;
				}
		}


	}
}

void VeloScan::ClassifibyTrackingAllObject(int currentNO ,int windowsize )
{
   trackMgr.ClassifiyTrackersObjects(Scan::allScans, currentNO, windowsize) ;
}

void VeloScan::MarkStaticorMovingPointCloud()
{
	int i,j,k;
    DataXYZ xyz(get("xyz"));
	DataType Pt(get("type"));

	int startofpoint  = 0;
	int colMax=  scanCellFeatureArray.size();
	for(i=0; i<colMax; ++i)
	{
		cellFeatureColumn  &gFeatureCol = scanCellFeatureArray[i];
		int rowMax= gFeatureCol.size();
		for(j=0;j< rowMax; ++j)
		{
		   cellFeature &gcellFreature =  gFeatureCol[j];
		   startofpoint += gcellFreature.pCell->size();
		   cell &gCell =*( gcellFreature.pCell);

		   for( k=0; k< gcellFreature.pCell->size();++k)
		   {
			        // find Point in scan raw points by point_id;
					Point p = *(gCell[k]);
				   if(gcellFreature.cellType & CELL_TYPE_STATIC)
					   Pt[p.point_id] = POINT_TYPE_STATIC_OBJECT;
    			   if(gcellFreature.cellType & CELL_TYPE_MOVING)
					   Pt[p.point_id] = POINT_TYPE_MOVING_OBJECT;
    			   if(gcellFreature.cellType & CELL_TYPE_GROUND)
					   Pt[p.point_id] = POINT_TYPE_GROUND;
		   }

		}
	}
}

void VeloScan::FindingAllofObject(int maxDist, int minDist)
{
	TransferToCellArray(maxDist, minDist);
	CalcScanCellFeature();
	FindAndCalcScanClusterFeature();

    return;
 }


void VeloScan::TrackingAllofObject(int trackingAlgo)
{
	trackMgr.HandleScan(*this,trackingAlgo);
 }

void VeloScan::ClassifiAllofObject()
{
	 ClassifiAllObject();
 }

void VeloScan::ExchangePointCloud()
{
     MarkStaticorMovingPointCloud();
     return;
 }
