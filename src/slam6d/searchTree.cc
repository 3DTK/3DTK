/** 
 * @file 
 * @brief Representation of a general search trees
 * @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/searchTree.h"
#include "slam6d/scan.h"
#include "slam6d/globals.icc"



#ifndef WITH_SCANSERVER
void SearchTree::getPtPairs(vector <PtPair> *pairs, 
    double *source_alignxf,                          // source
    double * const *q_points, unsigned int startindex, unsigned int endindex,  // target
    int thread_num,
    int rnd, double max_dist_match2, double &sum,
    double *centroid_m, double *centroid_d)
#else //WITH_SCANSERVER
void SearchTree::getPtPairs(vector <PtPair> *pairs, 
    double *source_alignxf,                          // source
    const DataXYZ& xyz_r, unsigned int startindex, unsigned int endindex,  // target
    int thread_num,
    int rnd, double max_dist_match2, double &sum,
    double *centroid_m, double *centroid_d)
#endif //WITH_SCANSERVER
{
#ifdef WITH_SCANSERVER
  // prepare this tree for resource access in FindClosest
  lock();
#endif //WITH_SCANSERVER
  
  double local_alignxf_inv[16];
  M4inv(source_alignxf, local_alignxf_inv);
  
  // t is the original point from target, s is the (inverted) query point from target and then
  // the closest point in source
  double t[3], s[3];
  for (unsigned int i = startindex; i < endindex; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only
    
#ifndef WITH_SCANSERVER
    t[0] = q_points[i][0];
    t[1] = q_points[i][1];
    t[2] = q_points[i][2];
#else //WITH_SCANSERVER
    t[0] = xyz_r[i][0];
    t[1] = xyz_r[i][1];
    t[2] = xyz_r[i][2];
#endif //WITH_SCANSERVER
    
    transform3(local_alignxf_inv, t, s);
    
    double *closest = this->FindClosest(s, max_dist_match2, thread_num);
    if (closest) {
      transform3(source_alignxf, closest, s);
      
      // This should be right, model=Source=First=not moving
      centroid_m[0] += s[0];
      centroid_m[1] += s[1];
      centroid_m[2] += s[2];
      centroid_d[0] += t[0];
      centroid_d[1] += t[1];
      centroid_d[2] += t[2];
      
      PtPair myPair(s, t);
      double p12[3] = { 
        myPair.p1.x - myPair.p2.x, 
        myPair.p1.y - myPair.p2.y,
        myPair.p1.z - myPair.p2.z };
      sum += Len2(p12);
      
      pairs->push_back(myPair);
    /*cout << "PTPAIR" << i << " " 
      << p[0] << " "
      << p[1] << " "
      << p[2] << " - " 
      << q_points[i][0] << " "
      << q_points[i][1] << " "
      << q_points[i][2] << "          " << Len2(p12) << endl; */
    }

  }
  
#ifdef WITH_SCANSERVER
  // release resource access lock
  unlock();
#endif //WITH_SCANSERVER

  return;
}

