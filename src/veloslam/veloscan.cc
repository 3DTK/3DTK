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

#include "veloslam/veloscan.h"
#include "veloslam/trackermanager.h"

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

#include <GL/gl.h>			/* OpenGL header file */
#include <GL/glu.h>			/* OpenGL utilities header file */

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

#include "veloslam/color_util.h"
TrackerManager trackMgr;
float absf(float a)
{
	return a>0?a:-a;
}


extern int Show(int frameno);

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
    for (int i = 0; i < end_loop; i++)
    {
        numpts += MetaScan[i]->points_red_size;
    }
    points_red_size = numpts;
    points_red = new double*[numpts];
    int k = 0;
    for (int i = 0; i < end_loop; i++)
    {
        for (int j = 0; j < MetaScan[i]->points_red_size; j++)
        {
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
    while ((_fileNr = my_ScanIO.readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1)
    {
        VeloScan *currentScan = new VeloScan(eu, maxDist);

        currentScan->setFileNr(_fileNr);
        currentScan->setPoints(&ptss);    // copy points
        ptss.clear();                   // clear points
        allScans.push_back(currentScan);
        cout << "removing dynamic objects ... ";
        currentScan->GetAllofObject();
        cout << "done" << endl;
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

//#ifndef _MSC_VER
//#ifdef _OPENMP
//    #pragma omp parallel
//    {
//        #pragma omp single nowait
//        {
//#endif
//#endif
            // read Scan-by-scan until no scan is available anymore
            while ((_fileNr = my_ScanIO.readScans(start, end, dir, maxDist, minDist, eu, ptss)) != -1)
            {
                VeloScan *currentScan = new VeloScan(eu, maxDist);
                currentScan->setFileNr(_fileNr);
                currentScan->setPoints(&ptss);    // copy points
                ptss.clear();                  // clear points
                allScans.push_back(currentScan);

//#ifndef _MSC_VER
//#ifdef _OPENMP
//                #pragma omp task
//#endif
//#endif
                {
                    cout << "removing dynamic objects, reducing scan " << currentScan->getFileNr() << " and creating searchTree" << endl;
                    currentScan->GetAllofObject();
                    currentScan->calcReducedPoints(voxelSize, nrpts);
                    currentScan->transform(currentScan->getTransMatOrg(), INVALID); //transform points to initial position
                    currentScan->clearPoints();
                    currentScan->createTree(nns_method, cuda_enabled);

                }
            }
//#ifndef _MSC_VER
//#ifdef _OPENMP
//        }
//    }
//    #pragma omp taskwait
//#endif
//#endif

    return;
}


int VeloScan::CalcRadAndTheta()
{
    int i,j;
    int size=  points.size();

	// ������ת�����꣬����y��Ϊz��
    for(i=0; i< size; ++i)
    {
		points[i].rad  = sqrt(  points[i].x*points[i].x   +   points[i].z*points[i].z );
		points[i].tan_theta  = points[i].z/points[i].x;
    }
    return 0;
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
    
	// �������ֵ��ÿ���������
    CalcRadAndTheta();

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

			// ѡ���ھ����״��һ���ķ�Χ�ڽ��в���
            if(pt.rad <=MinRad || pt.rad>=MaxRad)
                continue;

		// �ĸ����޵�ɢ�� ��һ������  ��ת����ϵ

            if(pt.x >0 && pt.z>0 )
            {
                if(pt.x > pt.z)
                {
                    flag=pt.tan_theta;
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

			//�ڶ�����
            else if(pt.x<0 && pt.z>0)
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

			//��������
            else if(pt.x<0&&pt.z<0)
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

			//��������
            else if(pt.x>0&&pt.z<0)
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

			//��������
            else
            {
                continue;
            }

         //ɢ�й���   
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
   

		   if(cellobj[i]->point_type & POINT_TYPE_BELOW_R)
			   f.cellType|=CELL_TYPE_BELOW_R;

		   if(i==0)
		   {
			   outlier=0;
			   f.min_x=f.max_x=cellobj[i]->x;
			   f.min_z=f.max_z=cellobj[i]->z;
			   lastMaxY=f.min_y=f.max_y=cellobj[i]->y;  // �޸�����
		   }
		   else
		   {
			   if(f.max_x<cellobj[i]->x)		f.max_x=cellobj[i]->x;
	
			   if(f.min_x>cellobj[i]->x)	   f.min_x=cellobj[i]->x;
	
			   if(f.max_z<cellobj[i]->z)	   f.max_z=cellobj[i]->z;
	
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

		//  ����·����ϰ���ķָ��������޸�Cell������
	   float threshold;
   	   threshold=f.delta_y;

	   float GridThresholdGroundDetect =120;
	   if(threshold >  GridThresholdGroundDetect)
		   f.cellType|=CELL_TYPE_ABOVE_DELTA_Y;
	   else
		   f.cellType|=CELL_TYPE_BELOW_DELTA_Y;
	
    return 0;
}


int VeloScan::CalcScanCellFeature()
{
    int i,j;

    if( scanCellArray.size()==0)
        return -1;

    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

//����ռ䡣�ܹ���columnSize(360)�������column,ÿһ��column����cellNumber������ռ䡣
    if( scanCellFeatureArray.size()==0)
    {
        scanCellFeatureArray.resize(columnSize);
        for(i=0; i<columnSize; ++i)
            scanCellFeatureArray[i].resize(cellNumber); 
    }

//����ÿ��Cell��������
    for(j=0; j <columnSize; j++)
    {
        cellColumn &column=scanCellArray[j];
        cellNumber=column.size();
        for( i=0; i<cellNumber; i++)
        {
            cell &cellObj=scanCellArray[j][i];
            cellFeature &feature=scanCellFeatureArray[j][i];
			//��ס������λ��
			feature.columnID=j;
			feature.cellID=i;

			//���ڻ�����ڼ�����صĵ���
            feature.pCell=&cellObj;
            CalcCellFeature(cellObj,feature);

			// �õ����ϰ���ĵ��
	         if( feature.delta_y > 120)
             {
				 scanCellFeatureArray[j][i].cellType |= CELL_TYPE_ABOVE_DELTA_Y;
				 for(int k=0;k <scanCellArray[j][i].size(); k++)
					 scanCellArray[j][i][k]->point_type |= POINT_TYPE_ABOVE_DELTA_Y;

			 }

        }
    }

    return 0;
}

int VeloScan::SearchNeigh(cluster& clu,charvv& flagvv,int i,int j)
{
    int columnSize=scanCellArray.size();
    int cellNumber=scanCellArray[0].size();

	//���˸�����Ե��ֹͣ����
    if(i<0||i>=columnSize||j<0||j>=cellNumber)
        return 0;
	//���˸��ڵ�
    if(flagvv[i][j]==1)
        return 0;

    if(scanCellFeatureArray[i][j].size==0)
    {
        flagvv[i][j]=1;
        return 0;
    }

    if(scanCellFeatureArray[i][j].cellType & CELL_TYPE_ABOVE_DELTA_Y) //�з���Ҫ���cell
    {
        flagvv[i][j]=1;

    	clu.push_back(&scanCellFeatureArray[i][j]); //�����Ⱥ��

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

int VeloScan::CalcClusterFeature(cluster& clu,clusterFeature& f)
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
            if(clu.size())                                          //�ü�Ⱥ���з���Ҫ���cell���򱣴�ü�Ⱥ
                scanClusterArray.push_back(clu);
        }
    }

	int clustersize=scanClusterArray.size();                //�ܹ���clustersize����Ⱥ
	if(scanClusterFeatureArray.size()==0)
		scanClusterFeatureArray.resize(clustersize);

	//����ÿһ��Ŀ���������
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

void VeloScan::ExchangeNoObjectPointCloud()
{
	int i,j,k;

	// ���ڸ��ٵ�������
	points_tracking= points;
	points.clear();

	//ÿ��Cluster�����м���Cell��ÿ��Cell�кܶ��
	int startofpoint  = 0;
	int colMax=  scanCellFeatureArray.size();
	for(i=0; i<colMax; ++i)
	{ 
		cellFeatureColumn  &gFeatureCol = scanCellFeatureArray[i];
		int rowMax= gFeatureCol.size();
		for(j=0;j< rowMax; ++j)
		{
		   cellFeature &gcellFreature =  gFeatureCol[j];
		   // ���е�Objectȫ���μ�����
		//   if( !(gcellFreature.cellType & CELL_TYPE_FOR_SLAM6D))
		//	   continue;

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
	if(glu.size < 8)
		return false; // small object do not use it!

	return true; // no filter

	/**�����֮��**/
	//if( glu.size_z > 200 && ((glu.size_x>glu.size_y?glu.size_x:glu.size_y))<360)
	//{
	//	return false;
	//}
	///**����3.5�׳�������С��1.4m�����Բ�����Ϊ��**/
	//else if((glu.size_z>350 && glu.size_x<140)|| (glu.size_x>350 && glu.size_z<140))
	//{
	//	return false;
	//}
	//else if(glu.size_z > 250 )
	//{
	//	return false;
	//}
	//else if((glu.size_x>glu.size_y?glu.size_x:glu.size_y)>420 && glu.size_z<130)
	//{
	//	return false;
	//}

	//else if((glu.size_x>glu.size_y?glu.size_x:glu.size_y)>3.5
	//	&& ((glu.size_x>glu.size_y?glu.size_x:glu.size_y)/(glu.size_x<glu.size_y?glu.size_x:glu.size_y)>4))
	//{
	//	return false;
	//}

	//else if(glu.size_x<700 && glu.size_z<700 &&  glu.size_y > 100  )
	//{
	//	return true;
	//}

	//// ���˵�����15�׵� �����������϶��ǽ�����
	//if(glu.size_x>1500 || glu.size_z>1500 || glu.size_x*glu.size_z >600*600 )
	//{
	//	return false;
	//}

	//// ���˹�����ر��� �ر�ϸ���Ĵ����壬������դ��
	//if(glu.size_x*glu.size_z >500*500  &&  glu.size_x/glu.size_z < 1.5)
	//{
	//	return false;
	//}

	////���˵�1�׶�һ�µ� СĿ��	
	//if(glu.size_y < 100)
	//{
	//	return false;
	//}
	////���˵�������ܺ�С��3�׵� //�п���������
	//if((glu.size_x + glu.size_y + glu.size_z)<1.5)
	//{
	//	return false;
	//}

	// ���˵�������ȳ���3�׵� 
	//if(glu.size_z>700)
	//{
	//	return false;
	//}

	//if(glu.size_x>700)
	//{
	//	return false;
	//}

	////�п���������
	//if((glu.size_x + glu.size_z) <4)
	//{
	//	return false;
	//}

	//���˵�����ȴ���3��  //�кܶ๫�������������
	//if(glu.size_x/glu.size_z>3.0)
	//{
	//	return false;
	//}

//	return true;
}

static int scancont =0;

void VeloScan::GetAllofObject()
{
	int i,j;
	int size;

	if(points.size() > 0)
	{
			//	noofframe +=1000;
			//  ������֡���ݵ�PCD�ļ�
			//	SaveFrameInPCD();
       
			//  ɢ�е�����Cell��
			TransferToCellArray();
			//  ������Ӧ������ ���ĵ��ϰ����Cell
			CalcScanCellFeature();
			//  ����ΪCluster����
			FindAndCalcScanClusterFeature();
			//  ��ʼ���٣�ÿ֡����һ��
			/////////////////////////////////////////////
			trackMgr.HandleScan(*this);

			//TRACE("DrawTrackers\n");
	        
//			if(scancont ==3)
//    	     	Show(0);

			scancont ++;
			/////////////////////////////////////////////////

			//���涯̬Ŀ��Ķ���PCD�ļ�
			//	SaveAllofObject();
			//	SaveNoObjectPointCloud();
		//	ExchangeNoObjectPointCloud();

			// ��ʾ����Ľ�������ڵ��ԡ�
			//       Show(0);
			//���Ա�����˶�Ŀ��ĵ㵽PCD�ļ��У���һ��Ч������

			//��Ҫ�������еĴ洢���ݡ�
		//   FreeAllCellAndCluterMemory();
	}
    return;
 }
