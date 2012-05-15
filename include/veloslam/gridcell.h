#ifndef __GRID_CELL_H__
#define __GRID_CELL_H__

#ifdef _MSC_VER
#define snprintf _snprintf
#undef _STDIO_DEFINED
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/scan.h"
#include <cstring>
#include <string>
using std::string;
#include <vector>
using std::vector;

typedef std::vector<char> charv;
typedef std::vector<charv> charvv;

#define CELL_TYPE_INVALID        0x00000001
#define CELL_TYPE_ABOVE_DELTA_R  0x00000002
#define CELL_TYPE_BELOW_DELTA_R  0x00000004
#define CELL_TYPE_ABOVE_DELTA_Y  0x00000008
#define CELL_TYPE_BELOW_DELTA_Y  0x00000010
#define CELL_TYPE_ABOVE_R        0x00000020
#define CELL_TYPE_BELOW_R        0x00000040
#define CELL_TYPE_ABOVE_LIDAR       0x00000080
#define CELL_TYPE_IN_OBSTACLE_RANGE 0x00000100
#define CELL_TYPE_STATIC        0x00000200
#define CELL_TYPE_MOVING        0x00000400
#define CELL_TYPE_GROUND        0x00000800

typedef std::vector<Point*> cell;
typedef std::vector<cell> cellColumn;
typedef std::vector<cellColumn> cellArray;

class cellFeature
{
public:
	float ave_x,ave_y,ave_z;
	float min_x,min_y,min_z;
	float max_x,max_y,max_z;
	float delta_y;

	int circleNum;
	int size;
    unsigned int cellType;
	cell* pCell;

	int columnID;
	int cellID;
};

typedef std::vector<cellFeature> cellFeatureColumn;
typedef std::vector<cellFeatureColumn> cellFeatureArray;

#endif  //__GRID_CELL_H__
