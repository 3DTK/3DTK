/**
 * @file
 * @brief Implementation of a 3D scan and of 3D scan matching in all variants
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
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
#include "slam6d/Boctree.h"
#include "slam6d/scan_io.h"
#include "slam6d/d2tree.h"
#include "slam6d/kd.h"
#include "slam6d/kdc.h"

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
//
//void VeloScan::SaveObjectsInPCD(int index, cluster &gClusterData )
//{
//	//char filename[256]; 
//	//pcl::PointCloud<pcl::PointXYZ> cloud;
//	//pcl::PointXYZ point;
//	//cloud.height   = 1;
//	//cloud.is_dense = false;
//
//	////每个Cluster下面有几个Cell，每个Cell有很多点
//	//int startofpoint  = 0;
//	//for (int j = 0;  j <gClusterData.size(); ++j)
//	//{
//	//	/// scanClusterArray中存了Cell的位置信息么，
//	//	//如何用到提取 Cell 我感觉好像没有保存这样的信息。
//	//	cell* pcellObj= gClusterData[j]->pCell;
//	//	int pointSize=pcellObj->size();
//	//	for(int k=0; k < pointSize; ++k)
//	//	{
//	//		point.x = (*pcellObj)[k]->x/100.0;
//	//		point.y = (*pcellObj)[k]->y/100.0;
//	//		point.z = (*pcellObj)[k]->z/100.0;
//	//		cloud.points.push_back(point);
//	//	}
//	//}
//
//	//cloud.width    =  cloud.points.size();
//	//cloud.height   = 1;
//	//cloud.is_dense = false;
//	//if(cloud.width   > 0)
//	//{
//	//	sprintf (filename, "C:\\test\\object\\object%08d.pcd", index); 
//	//	//pcl::io::savePCDFileASCII (filename, cloud);
//	//	pcl::io::savePCDFileBinary( filename, cloud);
//	//}
//	
//}
//
//long indexFrame =0;
//
//void VeloScan::SaveFrameInPCD( )
//{
//	//char filename[256]; 
//	//pcl::PointCloud<pcl::PointXYZ> cloud;
//	//pcl::PointXYZ point;
//	//cloud.height   = 1;
//	//cloud.is_dense = false;
//
//	////每个Cluster下面有几个Cell，每个Cell有很多点
//	//int startofpoint  = 0;
//
//	//int pointSize= points.size();
//	//for(int k=0; k < pointSize; ++k)
//	//{
//	//	point.x = points[k].x/100.0;
//	//	point.y = points[k].y/100.0;
//	//	point.z = points[k].z/100.0;
//	//	cloud.points.push_back(point);
//	//}
//
//	//cloud.width    =  cloud.points.size();
//	//cloud.height   = 1;
//	//cloud.is_dense = false;
//	//if(cloud.width   > 0)
//	//{
//	//	sprintf (filename, "C:\\test\\frame\\frame%08d.pcd", indexFrame++); 
//	//	//pcl::io::savePCDFileASCII (filename, cloud);
//	//	pcl::io::savePCDFileBinary( filename, cloud);
//	//}
//
//}
//
//static int noofframe =0;
//
//void VeloScan::SaveAllofObject()
//{
//		for(int i=0; i<scanClusterArray.size();++i)
//		{
//			//运动目标的特征结构
//			clusterFeature &glu=scanClusterFeatureArray[i];
//			cluster &gluData=scanClusterArray[i];
//			if(glu.size_x>1500 || glu.size_y>1500 || glu.size_x*glu.size_y >600*600 )
//       			SaveObjectsInPCD(noofframe*1000 + i, gluData );
//		}
//}
//
//void VeloScan::SaveNoObjectPointCloud()
//{
//	//int i,j,k;
//	//char filename[256]; 
//	//pcl::PointCloud<pcl::PointXYZ> cloud;
//	//pcl::PointXYZ point;
//	//cloud.height   = 1;
//	//cloud.is_dense = false;
//
//	////每个Cluster下面有几个Cell，每个Cell有很多点
//	//int startofpoint  = 0;
//	//int colMax=  scanCellFeatureArray.size();
//	//for(i=0; i<colMax; ++i)
//	//{ 
//	//	cellFeatureColumn  &gFeatureCol =	scanCellFeatureArray[i];
//	//	int rowMax= gFeatureCol.size();
//	//	for(j=0;j< rowMax; ++j)
//	//	{
//	//	   cellFeature &gcellFreature =  gFeatureCol[j];
//	//	   if( !(gcellFreature.cellType & CELL_TYPE_FOR_SLAM6D))
//	//		   continue;
//
//	//	   startofpoint += gcellFreature.pCell->size();
//	//	   cell &gCell =*( gcellFreature.pCell);
//	//	   for( k=0; k< gcellFreature.pCell->size();++k)
//	//	   {
//	//			Point p = *(gCell[k]);
//	//			point.x = p.x/100.0;
//	//			point.y = p.y/100.0;
//	//			point.z = p.z/100.0;
//	//			cloud.points.push_back(point);
//	//	   }
//	//	
//	//	}
//	//}
//
//	//cloud.width    =  startofpoint;
//	//cloud.height   = 1;
//	//cloud.is_dense = false;
//	//if(cloud.width   > 0)
//	//{
//	//	sprintf (filename, "C:\\test\\noobject\\noobject%08d.pcd", indexFrame); 
//	//	//pcl::io::savePCDFileASCII (filename, cloud);
//	//	pcl::io::savePCDFileBinary( filename, cloud);
//	//}
//	//
//}