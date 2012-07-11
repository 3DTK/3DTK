/*
 * pcddump implementation
 *
 * Copyright (C) YuanJun, ZhangLiang, Li Wei, Li Ming, Andreas Nuechter,
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Main programm for dynamic Velodyne SLAM
 *
 * @author Andreas Nuechter. Jacobs University Bremen, Germany
 * @author YuanJun, Wuhan University, China
 * @author ZhangLiang, Wuhan University, China
 * @author Li Wei, Wuhan University, China
 * @author Li Ming, Wuhan University, China
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
#include "veloslam/pcddump.h"

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

void DumpPointtoFile(cluster &gluData1, string filename)
{
	cell* pCell;
    ofstream redptsout(filename.c_str());

	for(int j=0; j<gluData1.size();++j)
	{
		pCell=gluData1[j]->pCell;
		for(int k=0; k<pCell->size();++k)
		{
			redptsout <<(*pCell)[k]->x << " "
				           <<(*pCell)[k]->y<< " "
						   <<(*pCell)[k]->z<< endl;
		}
	}
    redptsout.close();
    redptsout.clear();
}

void DumpFeaturetoFile(clusterFeature &glu, string filename)
{
	ofstream redptsout(filename.c_str(), ios::app);
    redptsout <<glu.size_x << " "<< glu.size_y << " "<< glu.size_z <<" "<<
	            glu.speed<< " "<< glu.speed_x<< " "<< glu.speed_y<<" "<< glu.pointNumber <<endl;

    redptsout.close();
    redptsout.clear();
}
