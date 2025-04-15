#ifndef __CONTINUOUS_H_
#define __CONTINUOUS_H_


#include <vector>
using std::vector;

#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
#include "slam6d/graphSlam6D.h"
#include "graphSlam6DL.h"
#include "lum6Deuler.h"

#include "linescan.h"

inline void transform(double *point, const double *alignxf)
{
  double x_neu, y_neu, z_neu;
  x_neu = point[0] * alignxf[0] + point[1] * alignxf[4] + point[2] * alignxf[8];
  y_neu = point[0] * alignxf[1] + point[1] * alignxf[5] + point[2] * alignxf[9];
  z_neu = point[0] * alignxf[2] + point[1] * alignxf[6] + point[2] * alignxf[10];
  point[0] = x_neu + alignxf[12];
  point[1] = y_neu + alignxf[13];
  point[2] = z_neu + alignxf[14];
}

inline void linearDistributeError(vector<LineScan*> &linescans, int begin, int end, const double  *transmatnew, bool globalanim=false) {
  int length = end - begin;
  double *transmatold = linescans[end]->transMat;

  // compute change between old and corrected pose
  double transMatDIFF[16];
  double transMatOldInv[16];
  M4inv(transmatold,transMatOldInv);
  MMult(transmatnew,transMatOldInv,transMatDIFF);

  double qDiff[4];     // change in rotation as quternion
  double transDiff[3]; // change in translation (linear)
  Matrix4ToQuat(transMatDIFF,qDiff,transDiff);

  /// for a nice animation
  if (!globalanim) {
    double id[16]; M4identity(id);
    // first show scans before they are matched and moved
    for (unsigned int j = 0;j<2;j++) {
      for (int i = 0; i < begin; i++)  {
        linescans[i]->transform(id, Scan::ICPINACTIVE, 1);
      }
      for (int i = begin; i <= end; i++)  {
        linescans[i]->transform(id, Scan::ICP, 1);
      }
      for (unsigned int i = end+1; i < linescans.size(); i++)  {
        linescans[i]->transform(id, Scan::INVALID, 1);
      }
    }

    // next, show them after deformation. scans 0 to begin are not moved so plot again
    for (int i = 0; i < begin; i++)  {
      linescans[i]->transform(id, Scan::ICPINACTIVE, 1);
    }
  }
  ///////////

  double *transMat;

  // interpolate rotation with slerp and linear iterpolate translation
  // for all scans between begin and end
  double qStart[4]{1,0,0,0};
  for (int i = begin; i <= end; i++)  {
    transMat = linescans[i]->transMat;
    double t = (static_cast<double>(i-begin) / static_cast<double>(length));
    double qDiff_i[4];
    double transDiff_i[3];
    slerp(qStart,qDiff,t,qDiff_i);
    for(unsigned int j=0; j<3; j++) {
        transDiff_i[j] = transDiff[j]*t;
    }
    double transMatDiff_i[16];
    QuatToMatrix4(qDiff_i,transDiff_i,transMatDiff_i);

    if(globalanim && i == begin) {
        linescans[i]->transform(transMatDiff_i,Scan::INVALID,3);
    } else {
        linescans[i]->transform(transMatDiff_i,Scan::ICP,1);
    }
  }


// transform all subsequent scans in the same manner as "end"
  for (unsigned int i = end+1; i < linescans.size(); i++)  {
    if (!globalanim) {
      linescans[i]->transform(transMatDIFF, Scan::INVALID, 1 );
    } else {
      linescans[i]->transform(transMatDIFF, Scan::INVALID, 3 );
    }
  }
}

inline void outputScans(vector<LineScan*> &linescans, int begin, int end, const char *filename, const double *transMat = 0 ) {
  if (begin < 0) begin = 0;
  if ((unsigned int)end >= linescans.size()) end = linescans.size() -1;
  ofstream fout(filename);
  double p[3];

  double tinv[16];
  if (transMat == 0) {
    transMat = linescans[begin]->transMat;
  }

  M4inv(transMat, tinv);
  for (int i = begin; i <= end; i++)  {
    transMat = linescans[i]->transMat;
    transMat = linescans[i]->frames_mat;

    for (int j = 0; j < linescans[i]->nrpts; j++) {
      p[0] = linescans[i]->points[j][0];
      p[1] = linescans[i]->points[j][1];
      p[2] = linescans[i]->points[j][2];

      transform(p, transMat);
      transform(p, tinv);

      fout << p[0] << " " << p[1] << " " << p[2] << " " << linescans[i]->points[j][3] << endl;
    }
  }


}


/**
 * Creates a slam6D scan from the linescans begin-end, with the pose of index as reference coordinate system
 *
 */
Scan *joinLines(vector<LineScan*> &linescans, int begin, int end, int &index, double voxelsize = -1.0, int nrpts = 0 );

/**
 * Does ICP matching with a 3D scan created around linescan "index" (with "width" linescans in either direction)
 *
 * @param earliest is the index of the earliest linescan to match against
 * @param latest is the index of the latest linescan to match against
 *
 * Essentially we create 2 scans, that are rigidly matched using icp
 * the difference in the pose estimates of the "index" linescan is then distributed along the linescans in between
 *
 */
void preRegistration(vector<LineScan*> &linescans, int learliest, int llatest, int earliest, int latest, icp6D *icp, double voxelsize, int nr_octpts, int lindex=-1, int findex=-1);
void preRegistration(vector<LineScan*> &linescans, LScan* first, LScan* last, icp6D *icp, double voxelsize, int nr_octpts);

void SemiRigidRegistration(vector<LineScan*> &linescans, graphSlam6DL *slam, Graph *&gr, int iterations, int slamiters, double mdm, int scaninterval, int scansize, double voxelsize, int nr_octpts);


double SSRR(vector<LineScan*> &linescans, graphSlam6D *slam, graphSlam6DL *slaml, Graph *&gr, icp6D *icp, int slamiters, int clpairs, double mdm, int scaninterval, int scansize, double voxelsize, int nr_octpts, volatile bool *abort_flag = nullptr);

double SSRR(vector<LineScan*> &linescans, vector<LSegment*> lsegments, graphSlam6D *slam, graphSlam6DL *slaml, Graph *&gr, icp6D *icp, int slamiters, int clpairs, double mdm, double voxelsize, int nr_octpts, volatile bool *abort_flag = nullptr);
#endif
