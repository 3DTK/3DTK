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
#include "veloslam/pcddump.h"
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

void DumpPointtoFile(cluster &gluData1, string filename)
{  
	//cell* pCell;
 //   ofstream redptsout(filename);
 //
	//for(int j=0; j<gluData1.size();++j)
	//{
	//	pCell=gluData1[j]->pCell;
	//	for(int k=0; k<pCell->size();++k)
	//	{
	//		redptsout <<(*pCell)[k]->x << " "
	//			           <<(*pCell)[k]->y<< " "
	//					   <<(*pCell)[k]->z<< endl;

	//	}
	//}
 //   redptsout.close();
 //   redptsout.clear();
}

void DumpFeaturetoFile(clusterFeature &glu, string filename)
{  
	//ofstream redptsout("c:\\featue", ios::app);
 //   redptsout <<glu.size_x << " "<< glu.size_y << " "<< glu.size_z <<" "<<
	//	               glu.speed<< " "<< glu.speed_x<< " "<< glu.speed_y<<" "<< glu.pointNumber <<endl;

 //   redptsout.close();
 //   redptsout.clear();
}
