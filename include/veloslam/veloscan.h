/**
 * @file
 * @brief Representation of a 3D scan and implementation of scan matching
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
 */

#ifndef __VELOSCAN_H__
#define __VELOSCAN_H__

#ifdef _MSC_VER
#define snprintf _snprintf
#undef _STDIO_DEFINED
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/scan.h"

/** ÎÞÐ§Cell£¬ÀïÃæÃ»ÓÐµã*/
#define CELL_TYPE_INVALID                0x00000001
/** ´óÓÚ°ë¾¶¾ùÖµÆ½¾ù²îãÐÖµ*/
#define CELL_TYPE_ABOVE_DELTA_R  0x00000002
/** Ð¡ÓÚ°ë¾¶¾ùÖµÆ½¾ù²îãÐÖµ*/
#define CELL_TYPE_BELOW_DELTA_R  0x00000004
/** ´óÓÚ¸ß³Ì¾ùÖµÆ½¾ù²îãÐÖµ*/
#define CELL_TYPE_ABOVE_DELTA_Z  0x00000008
/** Ð¡ÓÚ¸ß³Ì¾ùÖµÆ½¾ù²îãÐÖµ*/
#define CELL_TYPE_BELOW_DELTA_Z  0x00000010
/** ´óÓÚ°ë¾¶ãÐÖµ*/
#define CELL_TYPE_ABOVE_R               0x00000020
/** Ð¡ÓÚ°ë¾¶ãÐÖµ*/
#define CELL_TYPE_BELOW_R              0x00000040
/** ±ÈÀ×´ï¸ß*/
#define CELL_TYPE_ABOVE_LIDAR       0x00000080
/** ÔÚÕÏ°­Îï¸ú×Ù¼ì²â¾àÀë·¶Î§ÄÚ*/
#define CELL_TYPE_IN_OBSTACLE_RANGE       0x00000100
/** ¿ÉÒÔÓÃÓÚSlam6DÆ´½ÓµÄµãÔÆ*/
#define CELL_TYPE_FOR_SLAM6D        0x00000200
/** ·ÇÖÐ¿Õ£¬ÔÚÐ£Ô°ÀïÃæÈç¹ûÍ·¶¥ÓÐÊ÷£¬ÔòÎªÖÐ¿Õ*/
#define CELL_TYPE_NOT_HOLLOW        0x00000400

/** ±ÈÀ×´ï¸ß*/
#define CUDE_TYPE_ABOVE_LIDAR       0x00000001
/** ÔÚÕÏ°­Îï¸ú×Ù¼ì²â¾àÀë·¶Î§ÄÚ*/
#define CUDE_TYPE_IN_OBSTACLE_RANGE       0x00000002
/** ¸Ã¸ñ×ÓÀïÓÐË®Æ½ sick µÄÉ¨Ãèµã£¬ËµÃ÷¸Ã¸ñ×ÓÊÇÕÏ°­Îï¸ñ×Ó **/
#define CUDE_TYPE_CONTAIN_SICK        0x00000400

/** ±ÈÀ×´ï¸ß*/
#define CLUSTER_TYPE_OBJECT       0x00000001
/** ÔÚÕÏ°­Îï¸ú×Ù¼ì²â¾àÀë·¶Î§ÄÚ*/
#define CLUSTER_TYPE_MOVING_OBJECT       0x00000002

/** ¸ñÍøµ¥Ôª,±ýÐÍºÍ·½ÐÎÍø¸ñµ¥Ôª£¬ÀïÃæ°üº¬ÁË¸Ãµ¥ÔªËùº¬ËùÓÐµãµÄÖ¸Õë*/
typedef vector<Point*> cell;
/** °ë¾¶·½ÏòÉÏµÄ ÏàÍ¬Ðý½ÇµÄ Ò»ÁÐ¸ñÍøµ¥Ôª*/
typedef vector<cell> cellColumn;
/** Õû¸ö¸ñÍø£¬±ýÐÍºÍ·½¸ñÐÎ*/
typedef vector<cellColumn> cellArray;  //±ý×´¸ñÍø·½Ê½

//!!!!

typedef vector<char> charv;
typedef vector<charv> charvv;

/** ÎÞÐ§µã£¬´òµ½¾àÀëÍâ£¬»òÕßÔÚÎÒÃÇ¶¨ÒåµÄ¾àÀëÖ®Íâ*/
#define POINT_TYPE_INVALID       0x00000001
/** ´óÓÚ°ë¾¶¾ùÖµÆ½¾ù²îãÐÖµ*/
#define POINT_TYPE_ABOVE_DELTA_R 0x00000002
/** Ð¡ÓÚ°ë¾¶¾ùÖµÆ½¾ù²îãÐÖµ*/
#define POINT_TYPE_BELOW_DELTA_R 0x00000004
/** ´óÓÚ¸ß³Ì¾ùÖµÆ½¾ù²îãÐÖµ*/
#define POINT_TYPE_ABOVE_DELTA_Z 0x00000008
/** Ð¡ÓÚ¸ß³Ì¾ùÖµÆ½¾ù²îãÐÖµ*/
#define POINT_TYPE_BELOW_DELTA_Z 0x00000010
/** ´óÓÚ°ë¾¶ãÐÖµ*/
#define POINT_TYPE_ABOVE_R       0x00000020
/** Ð¡ÓÚ°ë¾¶ãÐÖµ*/
#define POINT_TYPE_BELOW_R       0x00000040

//!!!!


/** ¸ñÍøµ¥ÔªµÄÌØÕ÷  cellFeature¡¡Õâ¸öÊÇ±ýÐÍÍø¸ñÌØÕ÷*/
class cellFeature
{
public:
	/** x,y,zµÄ¾ùÖµ*/
	float ave_x,ave_y,ave_z;
	/** x,y,zµÄ×îÖµ*/
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float delta_z;

	/** ÓÐ¶àÉÙ¸öcircleµÄµã */
	int circleNum;
	int size; //µãµÄ¸öÊý
	unsigned int cellType;
	cell* pCell;
};

typedef vector<cellFeature> cellFeatureColumn; //ÏàÍ¬Ðý½Ç·½ÏòÉÏµÄÍø¸ñÌØÕ÷Êý×é
typedef vector<cellFeatureColumn> cellFeatureArray;//Õû¸öÍø¸ñÌØÕ÷Êý×é

/** ¼¯ÈºµÄÌØÕ÷ Õâ¸öÊÇ±ýÐÍÍø¸ñ¼¯ÈºµÄÌØÕ÷*/// ¶¯Ä¿±êµÄÌØÕ÷ 
class clusterFeature
{
public:
	/** x,y,zµÄ×îÖµ*/
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float min_y_x,   max_y_x;           //×î´ó×îÐ¡Y ¶ÔÓ¦µÄx £¬ÖìÉñÌí Ìí¼Ó
	float size_x,size_y,size_z;
	float speed_x, speed_y, speed;   // ËÙ¶ÈÌØÕ÷

	int size;
	unsigned int clusterType;
};
typedef vector<clusterFeature> clusterFeatureArray;

//Ò»Á¾³µÊÇÓÉºÜ¶àÍø¸ñ×é³ÉµÄ
/** ¼¯Èº  */
typedef vector<cellFeature*> cluster;//Ò»¸ö¼¯ÈºÀïÓÐºÃ¼¸¸öÍø¸ñ£¬Ã¿¸öÍø¸ñÌØÕ÷ÏàËÆ

/** ¼¯ÈºÊý×é */
typedef vector<cluster> clusterArray;
typedef vector<clusterFeature> clusterFeatureArray;

typedef vector<cellFeature> cellFeatureColumn; //ÏàÍ¬Ðý½Ç·½ÏòÉÏµÄÍø¸ñÌØÕ÷Êý×é
typedef vector<cellFeatureColumn> cellFeatureArray;//Õû¸öÍø¸ñÌØÕ÷Êý×é


/**
 * @brief 3D scan representation and implementation of dynamic velodyne scan matching
 */
class VeloScan : public Scan {

public:
  VeloScan();
  VeloScan(const double *euler, int maxDist = -1);
  VeloScan(const double rPos[3], const double rPosTheta[3], int maxDist = -1);
  VeloScan(const double _rPos[3], const double _rPosTheta[3], vector<double *> &pts);
  VeloScan(const vector < VeloScan* >& MetaScan, int nns_method, bool cuda_enabled);
  VeloScan(const VeloScan& s);

  ~VeloScan();

  static void readScans(reader_type type,
				    int start, int end, string &dir, int maxDist, int minDist,
				    bool openFileForWriting = false);  
  static void readScansRedSearch(reader_type type,
						   int start, int end, string &dir, int maxDist, int minDist,
						   double voxelSize, int nrpts, // reduction parameters
						   int nns_method, bool cuda_enabled, 
						   bool openFileForWriting = false);  

private:

  /**
   * Vector storing single scans of a metascan
   */
  vector <VeloScan *> meta_parts;
  
  // FIXME
  void GetAllofObject();
  //  void SaveAllofObject();
  //  void SaveNoObjectPointCloud();
  int TransferToCellArray();
  //  int CalcRadAndTheta();
  bool FilterNOMovingObjcets(clusterFeature &glu);
  void ExchangeNoObjectPointCloud();
  void FreeAllCellAndCluterMemory();
  
  int CalcScanCellFeature();
  int CalcCellFeature(cell& cellobj,cellFeature& f);
  int FindAndCalcScanClusterFeature();
  int SearchNeigh(cluster& clu,charvv& flagvv,int i,int j);
  int CalcClusterFeature(cluster& clu,clusterFeature& f);
  void SaveObjectsInPCD(int index, cluster &gClusterData );
  void SaveFrameInPCD( );

  /** scanCellFeatureArray ºÍscanCellArrayÊÇÒ»Ò»¶ÔÓ¦µÄ¹ØÏµ£¬°üº¬ËùÓÐcellµÄÌØÕ÷ */
  cellArray scanCellArray;
  cellFeatureArray scanCellFeatureArray;
  
  clusterArray scanClusterArray;
  clusterFeatureArray scanClusterFeatureArray;

};

#endif
