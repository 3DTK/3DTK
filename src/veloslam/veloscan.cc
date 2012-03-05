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

/**
 * default Constructor
 */
VeloScan::VeloScan()
  : Scan()
{ }

/**
 * Constructor
 * @param *euler 6D pose: estimation of the scan location, e.g. based on odometry
 * @param maxDist Regard only points up to an (Euclidean) distance of maxDist
 * transformation matrices when match (default: false)
 */
VeloScan::VeloScan(const double* euler, int maxDist)
  : Scan(euler, maxDist)
{ }

VeloScan::VeloScan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts)
  : Scan(_rPos, _rPosTheta, pts)
{ }

/**
 * Constructor
 * @param _rPos[3] 3D position: estimation of the scan location, e.g. based on odometry
 * @param _rPosTheta[3] 3D orientation: estimation of the scan location, e.g. based on odometry
 * @param maxDist Regard only points up to an (Euclidean) distance of maxDist
 */
VeloScan::VeloScan(const double _rPos[3], const double _rPosTheta[3], const int maxDist)
  : Scan(_rPos, _rPosTheta, maxDist) 
{ }

/**
 * Constructor for creating a metascan from a list of scans
 * It joins all the points and contructs a new search tree
 * and reinitializes the cache, iff needed
 *
 * @param MetaScan Vector that contains the 3D scans
 * @param nns_method Indicates the version of the tree to be built
 * @param cuda_enabled indicated, if cuda should be used for NNS
 */
VeloScan::VeloScan(const vector < VeloScan* >& MetaScan, int nns_method, bool cuda_enabled)
{
  kd = 0;
  ann_kd_tree = 0;
  scanNr = numberOfScans++;
  rPos[0] = 0;
  rPos[1] = 0;
  rPos[2] = 0;
  rPosTheta[0] = 0;
  rPosTheta[1] = 0;
  rPosTheta[2] = 0;
  M4identity(transMat);
  M4identity(transMatOrg);
  M4identity(dalignxf);

  // copy points
  int numpts = 0;
  int end_loop = (int)MetaScan.size();
  for (int i = 0; i < end_loop; i++) {
    numpts += MetaScan[i]->points_red_size;
  }
  points_red_size = numpts;
  points_red = new double*[numpts];  
  int k = 0;
  for (int i = 0; i < end_loop; i++) {
    for (int j = 0; j < MetaScan[i]->points_red_size; j++) {
	 points_red[k] = new double[3];
	 points_red[k][0] = MetaScan[i]->points_red[j][0];
	 points_red[k][1] = MetaScan[i]->points_red[j][1];
	 points_red[k][2] = MetaScan[i]->points_red[j][2];
	 k++;
    }
  }

  fileNr = -1; // no need to store something from a meta scan!
  scanNr = numberOfScans++;

  // build new search tree
  createTree(nns_method, cuda_enabled);
  // update max num point in scan iff you have to do so
  if (points_red_size > (int)max_points_red_size) max_points_red_size = points_red_size;

  // add Scan to ScanList
  allScans.push_back(this);

  meta_parts = MetaScan;
}
  
/**
 * Desctuctor
 */
VeloScan::~VeloScan()
{ }

/**
 * Copy constructor
 */
VeloScan::VeloScan(const VeloScan& s)
  : Scan(s)
{ }

/**
 * Reads specified scans from given directory. 
 * Scan poses will NOT be initialized after a call 
 * to this function. It loads a shared lib where the 
 * actual file processing takes place
 * 
 * @param type Specifies the type of the flies to be loaded
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param _dir The drectory containing the data
 * @param maxDist Reads only Points up to this distance
 * @param minDist Reads only Points from this distance
 * @param openFileForWriting Opens .frames files to store the 
 *        scan matching results
 */
void VeloScan::readScans(reader_type type,
					int start, int end, string &_dir, int maxDist, int minDist, 
					bool openFileForWriting)
{
  outputFrames = openFileForWriting;
  dir = _dir;
  double eu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  vector <Point> ptss;
  int _fileNr;
  scanIOwrapper my_ScanIO(type);

  // read Scan-by-scan until no scan is available anymore
  while ((_fileNr = my_ScanIO.readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1) {
    VeloScan *currentScan = new VeloScan(eu, maxDist);
    
    currentScan->setFileNr(_fileNr);
    currentScan->setPoints(&ptss);    // copy points
    ptss.clear();                   // clear points
    allScans.push_back(currentScan);
  }
  return;
}


void VeloScan::readScansRedSearch(reader_type type,
		     int start, int end, string &_dir, int maxDist, int minDist, 
						double voxelSize, int nrpts, // reduction parameters
						int nns_method, bool cuda_enabled, 
						bool openFileForWriting)
{
  outputFrames = openFileForWriting;
  dir = _dir;
  double eu[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  vector <Point> ptss;
  int _fileNr;
  scanIOwrapper my_ScanIO(type);

#ifndef _MSC_VER 
#ifdef _OPENMP
#pragma omp parallel
  {
#pragma omp single nowait
    {
#endif
#endif
      // read Scan-by-scan until no scan is available anymore
      while ((_fileNr = my_ScanIO.readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1) {
        VeloScan *currentScan = new VeloScan(eu, maxDist);
	   currentScan->setFileNr(_fileNr);
	   currentScan->setPoints(&ptss);    // copy points
        ptss.clear();                  // clear points
        allScans.push_back(currentScan);

#ifndef _MSC_VER 
#ifdef _OPENMP
#pragma omp task
#endif
#endif
        {
          cout << "removing dynamic objects, reducing scan " << currentScan->getFileNr() << " and creating searchTree" << endl;
		currentScan->GetAllofObject();
		currentScan->calcReducedPoints(voxelSize, nrpts);
          currentScan->transform(currentScan->getTransMatOrg(), INVALID); //transform points to initial position
		currentScan->clearPoints();
		currentScan->createTree(nns_method, cuda_enabled);
		
        }
      }
#ifndef _MSC_VER 
#ifdef _OPENMP
    }
  }
#pragma omp taskwait
#endif
#endif

  return;
}

/*
int VeloScan::CalcRadAndTheta()
{
    int i,j;
    int size=  points.size();

    for(i=0; i< size; ++i)
    {
	points[i].rad  = sqrt(  points[i].x*points[i].x   +   points[i].y*points[i].y );
	points[i].tan_theta  = points[i].y/points[i].x;
    }
    return 0;
}
*/

 /*
void VeloScan::SaveAllofObject()
{
		for(unsigned int i=0; i<scanClusterArray.size();++i)
		{
			//运动目标的特征结构
			clusterFeature &glu=scanClusterFeatureArray[i];
			cluster &gluData=scanClusterArray[i];
			if(glu.size_x>1500 || glu.size_y>1500 || glu.size_x*glu.size_y >600*600 )
       			SaveObjectsInPCD(noofframe*1000 + i, gluData );
		}
}
 */
  /*
void VeloScan::SaveNoObjectPointCloud()
{
	//int i,j,k;
	//char filename[256]; 
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::PointXYZ point;
	//cloud.height   = 1;
	//cloud.is_dense = false;

	////每个Cluster下面有几个Cell，每个Cell有很多点
	//int startofpoint  = 0;
	//int colMax=  scanCellFeatureArray.size();
	//for(i=0; i<colMax; ++i)
	//{ 
	//	cellFeatureColumn  &gFeatureCol =	scanCellFeatureArray[i];
	//	int rowMax= gFeatureCol.size();
	//	for(j=0;j< rowMax; ++j)
	//	{
	//	   cellFeature &gcellFreature =  gFeatureCol[j];
	//	   if( !(gcellFreature.cellType & CELL_TYPE_FOR_SLAM6D))
	//		   continue;

	//	   startofpoint += gcellFreature.pCell->size();
	//	   cell &gCell =*( gcellFreature.pCell);
	//	   for( k=0; k< gcellFreature.pCell->size();++k)
	//	   {
	//			Point p = *(gCell[k]);
	//			point.x = p.x/100.0;
	//			point.y = p.y/100.0;
	//			point.z = p.z/100.0;
	//			cloud.points.push_back(point);
	//	   }
	//	
	//	}
	//}

	//cloud.width    =  startofpoint;
	//cloud.height   = 1;
	//cloud.is_dense = false;
	//if(cloud.width   > 0)
	//{
	//	sprintf (filename, "C:\\test\\noobject\\noobject%08d.pcd", indexFrame); 
	//	//pcl::io::savePCDFileASCII (filename, cloud);
	//	pcl::io::savePCDFileBinary( filename, cloud);
	//}
	//
}
  */
void VeloScan::ExchangeNoObjectPointCloud()
{
  unsigned int i,j,k;
	points.clear();

	//每个Cluster下面有几个Cell，每个Cell有很多点
	int startofpoint  = 0;
	unsigned int colMax=  scanCellFeatureArray.size();
	for(i=0; i<colMax; ++i)
	{ 
		cellFeatureColumn  &gFeatureCol = scanCellFeatureArray[i];
		unsigned int rowMax= gFeatureCol.size();
		for(j=0;j< rowMax; ++j)
		{
		   cellFeature &gcellFreature =  gFeatureCol[j];
		   if( !(gcellFreature.cellType & CELL_TYPE_FOR_SLAM6D))
			   continue;

		   startofpoint += gcellFreature.pCell->size();
		   cell &gCell =*( gcellFreature.pCell);
		   for( k=0; k< gcellFreature.pCell->size();++k)
		   {
			Point p = *(gCell[k]);
			points.push_back(p);
		   }
		
		}
	}


}


void VeloScan::FreeAllCellAndCluterMemory()
{
	scanCellArray.clear();
	scanCellFeatureArray.clear();

	scanClusterArray.clear();;
	scanClusterFeatureArray.clear();
}


bool VeloScan::FilterNOMovingObjcets(clusterFeature &glu)
{
	/**树电杆之类**/
	if( glu.size_z > 200 && ((glu.size_x>glu.size_y?glu.size_x:glu.size_y))<360)
	{
		return false;
	}
	/**至少3.5米长，但宽小于1.4m，所以不可能为车**/
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

	// 过滤掉超过15米的 超级大的物体肯定是建筑物
	if(glu.size_x>1500 || glu.size_y>1500 || glu.size_x*glu.size_y >600*600 )
	{
		return false;
	}

	// 过滤过面积特别大的 特别细长的大物体，可能是栅栏
	if(glu.size_x*glu.size_y >500*500  &&  glu.size_x/glu.size_y < 1.5)
	{
		return false;
	}

	//过滤掉1米二一下的 小目标	
	if(glu.size_z < 100)
	{
		return false;
	}
	//过滤掉长宽高总和小于3米的 //有可能是行人
	if((glu.size_x + glu.size_y + glu.size_z)<1.5)
	{
		return false;
	}

	// 过滤掉超过宽度超过3米的 
	if(glu.size_y>300)
	{
		return false;
	}

	//if(glu.size_x<40||glu.size_y<40||glu.size_z<40)
	//	continue;

	//有可能是行人
	if((glu.size_x + glu.size_y) <4)
	{
		return false;
	}

	//过滤掉长宽比大于3的  //有很多公汽符合这个条件
	if(glu.size_x/glu.size_y>3.0)
	{
		return false;
	}

	return true;
}

void VeloScan::GetAllofObject()
{
  //	int i,j;
  //	int size;

	if(points.size() > 0)
	{
	  //		noofframe +=1000;
	    //  保存整帧数据到PCD文件
	//	SaveFrameInPCD();
		//  散列到饼型Cell中
		TransferToCellArray();
		//  计算相应的特征 并的到障碍物的Cell
		CalcScanCellFeature();
		//  聚类为Cluster对象
    		FindAndCalcScanClusterFeature();
		//保存动态目标的对象到PCD文件
	//	SaveAllofObject();
	//	SaveNoObjectPointCloud();
	        ExchangeNoObjectPointCloud();
		// 显示处理的结果，用于调试。
	//        Show(0);
		//可以保存非运动目标的点到PCD文件中，看一下效果啊。

		//需要清理所有的存储数据。
		FreeAllCellAndCluterMemory();
	}
    return;
 }
 
 
int VeloScan::TransferToCellArray()
{
#define  DefaultColumnSize 360

    int columnSize= 360;	  //cfg.cfgPlaneDetect.ColumnSize;
    int CellSize= 50;	 //cfg.cfgPlaneDetect.CellSize;	 
    int MinRad=0;		//cfg.cfgPlaneDetect.MinRad;	
    int MaxRad=6000;		//cfg.cfgPlaneDetect.MaxRad

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
    
	// 计算距离值和每个点的旋角
    //    CalcRadAndTheta();

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
//     printf("scanCellArray.size= %d\n",scanCellArray.size());
    for(i=0; i< points.size(); ++i)
    {
            count++;
            Point  &pt= points[i]; 

		  
		  
			// 选择在距离雷达的一定的范围内进行测试
            if( sqrt( pt.x*pt.x + pt.y*pt.y ) <= MinRad ||
			 sqrt( pt.x*pt.x + pt.y*pt.y ) >= MaxRad)
                continue;

		// 四个象限的散列 第一个象限
            if(pt.x >0 && pt.y>0 )
            {
                if(pt.x > pt.y)
                {
                    flag=pt.y/pt.x; 
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
			   flag=1 / (pt.y/pt.x); 
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

			//第二象限
            else if(pt.x<0 && pt.y>0)
            {
                if(-pt.x>pt.y)
                {
                    flag=-pt.y/pt.x; 
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
			   flag=1/(-pt.y/pt.x);
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

			//第三象限
            else if(pt.x<0&&pt.y<0)
            {
                if(-pt.x>-pt.y)
                {
                    flag=pt.y/pt.x; 
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
			   flag=1/ (pt.y/pt.x); 
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

			//第四象限
            else if(pt.x>0&&pt.y<0)
            {
                if(pt.x>-pt.y)
                {
                    flag=-pt.y/pt.x; 
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
			   flag=1 / (-pt.y/pt.x); 
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

			//都不属于
            else
            {
                continue;
            }

         //散列过程   
            int k= (int)((sqrt( pt.x*pt.x + pt.y*pt.y )-MinRad)/(CellSize*1.0));
            scanCellArray[offset][k].push_back(&pt);
    }
	return 0;
} 


int VeloScan::CalcScanCellFeature()
{
    int i,j;

    if( scanCellArray.size()==0)
        return -1;

    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

//分配空间。总共有columnSize(360)个方向的column,每一个column分配cellNumber个网格空间。
    if( scanCellFeatureArray.size()==0)
    {
        scanCellFeatureArray.resize(columnSize);
        for(i=0; i<columnSize; ++i)
            scanCellFeatureArray[i].resize(cellNumber); 
    }

//计算每个Cell的特征。
    for(j=0; j <columnSize; j++)
    {
        cellColumn &column=scanCellArray[j];
        cellNumber=column.size();
        for( i=0; i<cellNumber; i++)
        {
            cell &cellObj=scanCellArray[j][i];
            cellFeature &feature=scanCellFeatureArray[j][i];
            feature.pCell=&cellObj;
            CalcCellFeature(cellObj,feature);

			// 得到是障碍物的点簇
	    if( feature.delta_z > 120)
  		scanCellFeatureArray[j][i].cellType |= CELL_TYPE_ABOVE_DELTA_Z;

        }
    }

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
            if(clu.size())                                          //该集群里有符合要求的cell，则保存该集群
                scanClusterArray.push_back(clu);
        }
    }

	int clustersize=scanClusterArray.size();                //总共有clustersize个集群
	if(scanClusterFeatureArray.size()==0)
		scanClusterFeatureArray.resize(clustersize);

	//计算每一个目标的特征。
	for(i=0; i<clustersize; ++i)
		CalcClusterFeature(scanClusterArray[i],scanClusterFeatureArray[i]);

	//Find moving Ojbects
	for(i=0; i<clustersize; ++i)
	{
     	clusterFeature &glu = scanClusterFeatureArray[i];
		glu.clusterType != CLUSTER_TYPE_OBJECT;
	    if( FilterNOMovingObjcets(glu))
		{
				clusterFeature &gclu = 	scanClusterFeatureArray[i];
				gclu.clusterType  |=CLUSTER_TYPE_MOVING_OBJECT;
		}
	}

	//Mark No Moving Ojbects CELLS
	int k;
	for(i=0; i<clustersize; ++i)
	{
     	clusterFeature &glu = scanClusterFeatureArray[i];
		if( !(glu.clusterType & CLUSTER_TYPE_MOVING_OBJECT))
		{
				cluster  &gclu = 	scanClusterArray[i];
				for(j =0; j< gclu.size() ; ++j)
				{
					cellFeature &gcF = *(gclu[j]);
					gcF.cellType |= CELL_TYPE_FOR_SLAM6D;
				}
		}
	
	}

	return 0;
}


int VeloScan::SearchNeigh(cluster& clu,charvv& flagvv,int i,int j)
{
    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

	//到了格网边缘，停止搜索
    if(i<0||i>=columnSize||j<0||j>=cellNumber)
        return 0;
	//到了跟节点
    if(flagvv[i][j]==1)
        return 0;

    if(scanCellFeatureArray[i][j].size==0)
    {
        flagvv[i][j]=1;
        return 0;
    }

    if(scanCellFeatureArray[i][j].cellType & CELL_TYPE_ABOVE_DELTA_Z) //有符合要求的cell
    {
        flagvv[i][j]=1;

	clu.push_back(&scanCellFeatureArray[i][j]); //存进集群里

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
    }

    return 0;
}


int VeloScan::CalcClusterFeature(cluster& clu,clusterFeature& f)
{
    f.size=clu.size();
	
    if(f.size==0)
    {
        return 0;
    }
	
    f.clusterType=0;

    int i=0;
    for(i=0; i<f.size; ++i)
    {
        if(i==0)
        {
            f.min_x=f.max_x=clu[i]->min_x;
            f.min_y=f.max_y=clu[i]->min_y;
            f.min_z=f.max_z=clu[i]->min_z;
        }
        else
        {
            if(f.max_x<clu[i]->max_x)
                f.max_x=clu[i]->max_x;

            if(f.min_x>clu[i]->min_x)
                f.min_x=clu[i]->min_x;

            if(f.max_y<clu[i]->max_y)
                f.max_y=clu[i]->max_y;

            if(f.min_y>clu[i]->min_y)
                f.min_y=clu[i]->min_y;

            if(f.max_z<clu[i]->max_z)
                f.max_z=clu[i]->max_z;

            if(f.min_z>clu[i]->min_z)
                f.min_z=clu[i]->min_z;

        }
    }
    f.size_x=f.max_x-f.min_x;
	f.size_y=f.max_y-f.min_y;
	f.size_z=f.max_z-f.min_z;

    return 0;
}


int VeloScan::CalcCellFeature(cell& cellobj, cellFeature& f)
{
	   int outlier;
	   float lastMaxZ;
	   f.size=cellobj.size();
	   f.cellType=0;
	   
	   if(f.size==0)
	   {
		   f.cellType|=CELL_TYPE_INVALID;
		   return 0;
	   }
	   
	   f.ave_x=f.ave_y=f.ave_z=0.0;
	   f.delta_z=0;
	
	   int i=0;
	   for(i=0; i<f.size; ++i)
	   {
		   f.ave_x+=cellobj[i]->x;
		   f.ave_y+=cellobj[i]->y;
		   f.ave_z+=cellobj[i]->z;
	
		   if(cellobj[i]->type & POINT_TYPE_BELOW_R)
			   f.cellType|=CELL_TYPE_BELOW_R;

		   if(i==0)
		   {
			   outlier=0;
			   f.min_x=f.max_x=cellobj[i]->x;
			   f.min_y=f.max_y=cellobj[i]->y;
			   lastMaxZ=f.min_z=f.max_z=cellobj[i]->z;
		   }
		   else
		   {
			   if(f.max_x<cellobj[i]->x)
				   f.max_x=cellobj[i]->x;
	
			   if(f.min_x>cellobj[i]->x)
				   f.min_x=cellobj[i]->x;
	
			   if(f.max_y<cellobj[i]->y)
				   f.max_y=cellobj[i]->y;
	
			   if(f.min_y>cellobj[i]->y)
				   f.min_y=cellobj[i]->y;
	
			   if(f.max_z<cellobj[i]->z)
			   {
				   lastMaxZ=f.max_z;
				   f.max_z=cellobj[i]->z;
				   outlier=i;
			   }
	
			   if(f.min_z>cellobj[i]->z)
				   f.min_z=cellobj[i]->z;
	
		   }
	   }
	
	   if(f.size>1)
	   {
		   int z=f.ave_z-cellobj[outlier]->z;
		   z/=(f.size-1)*1.0;
	
		   if(cellobj[outlier]->z-z<50)
		   {
			   outlier=-1;
			   f.ave_z/=f.size*1.0;
		   }
		   else
		   {
			   f.max_z=lastMaxZ;
			   f.ave_z=z;
		   }
	   }
	   else
	   {
		   outlier=-1;
		   f.ave_z/=f.size*1.0;
	   }
	
	   f.ave_x/=f.size*1.0;
	   f.ave_y/=f.size*1.0;

	   for(i=0; i<f.size; ++i)
	   {
		   if(i==outlier)
			   continue;
		   f.delta_z+= fabs(cellobj[i]->z-f.ave_z);
	   }


		//  进行路面和障碍物的分割，具体就是修改Cell的属性
	   float threshold;
//	   threshold=f.delta_z/((f.size)*1.0);
   	   threshold=f.delta_z;

	   float GridThresholdGroundDetect =120;
	   if(threshold >  GridThresholdGroundDetect)
		   f.cellType|=CELL_TYPE_ABOVE_DELTA_Z;
	   else
		   f.cellType|=CELL_TYPE_BELOW_DELTA_Z;
	
    return 0;
}
