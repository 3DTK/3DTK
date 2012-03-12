#ifndef __GRID_CELL_H__
#define __GRID_CELL_H__

#ifdef _MSC_VER
#define snprintf _snprintf
#undef _STDIO_DEFINED
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/scan.h"

//!!!!
typedef vector<char> charv;
typedef vector<charv> charvv;

#define CELL_TYPE_INVALID                0x00000001
#define CELL_TYPE_ABOVE_DELTA_R  0x00000002
#define CELL_TYPE_BELOW_DELTA_R  0x00000004
#define CELL_TYPE_ABOVE_DELTA_Y  0x00000008
#define CELL_TYPE_BELOW_DELTA_Y  0x00000010
#define CELL_TYPE_ABOVE_R               0x00000020
#define CELL_TYPE_BELOW_R              0x00000040
#define CELL_TYPE_ABOVE_LIDAR       0x00000080
#define CELL_TYPE_IN_OBSTACLE_RANGE       0x00000100
#define CELL_TYPE_FOR_SLAM6D        0x00000200
#define CELL_TYPE_NOT_HOLLOW        0x00000400

typedef vector<Point*> cell;
typedef vector<cell> cellColumn;
typedef vector<cellColumn> cellArray;  

/** 格网单元的特征  cellFeature　这个是饼型网格特征*/
class cellFeature
{
public:
	/** x,y,z的均值*/
	float ave_x,ave_y,ave_z;
	/** x,y,z的最值*/
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float delta_y;

	/** 有多少个circle的点 */
	int circleNum;
	int size; //点的个数
	unsigned int cellType;
	cell* pCell;

	/** 格网单元在格网中的位置*/
	int columnID;
	int cellID;
};

typedef vector<cellFeature> cellFeatureColumn; 
typedef vector<cellFeatureColumn> cellFeatureArray;

#endif  //__GRID_CELL_H__
