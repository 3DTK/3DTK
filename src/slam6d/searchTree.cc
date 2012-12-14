/*
 * searchTree implementation
 *
 * Copyright (C) Jan Elseberg, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/** 
 * @file 
 * @brief Representation of a general search trees
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#include "slam6d/searchTree.h"
#include "slam6d/scan.h"
#include "slam6d/globals.icc"

#include <stdexcept>

double *SearchTree::FindClosestAlongDir(double *_p,
                                        double *_dir,
                                        double maxdist2,
                                        int threadNum) const
{
  throw std::runtime_error("Method FindClosestAlongDir is not implemented");
}

void SearchTree::getPtPairs(vector <PtPair> *pairs, 
                            double *source_alignxf,      // source
                            double * const *q_points,
                            unsigned int startindex,
                            unsigned int endindex,       // target
                            int thread_num,
                            int rnd,
                            double max_dist_match2,
                            double &sum,
                            double *centroid_m,
                            double *centroid_d)
{
  // prepare this tree for resource access in FindClosest
  lock();

  double local_alignxf_inv[16];
  M4inv(source_alignxf, local_alignxf_inv);
  
  // t is the original point from target,
  // s is the (inverted) query point from target and then
  // the closest point in source
  double t[3], s[3];
  for (unsigned int i = startindex; i < endindex; i++) {
    // take about 1/rnd-th of the numbers only
    if (rnd > 1 && rand(rnd) != 0) continue;  
    
    t[0] = q_points[i][0];
    t[1] = q_points[i][1];
    t[2] = q_points[i][2];
    
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
    }
  }
  
  // release resource access lock
  unlock();

  return;
}

void SearchTree::getPtPairs(vector <PtPair> *pairs, 
                            double *source_alignxf,         // source
                            const DataXYZ& xyz_r,
                            const DataNormal& normal_r,
                            unsigned int startindex,
                            unsigned int endindex,          // target
                            int thread_num,
                            int rnd,
                            double max_dist_match2,
                            double &sum,
                            double *centroid_m,
                            double *centroid_d,
                            PairingMode pairing_mode)
{
  // prepare this tree for resource access in FindClosest
  lock();
  
  double local_alignxf_inv[16];
  M4inv(source_alignxf, local_alignxf_inv);
  
  // t is the original point from target,
  // s is the (inverted) query point from target and then
  // the closest point in source
  double t[3], s[3], normal[3];
  for (unsigned int i = startindex; i < endindex; i++) {
    // take about 1/rnd-th of the numbers only
    if (rnd > 1 && rand(rnd) != 0) continue; 
    
    t[0] = xyz_r[i][0];
    t[1] = xyz_r[i][1];
    t[2] = xyz_r[i][2];

    transform3(local_alignxf_inv, t, s);

    double *closest;

    if (pairing_mode != CLOSEST_POINT) {
      normal[0] = normal_r[i][0];
      normal[1] = normal_r[i][1];
      normal[2] = normal_r[i][2];
      Normalize3(normal);
    }

    if (pairing_mode == CLOSEST_POINT_ALONG_NORMAL_SIMPLE) {
      transform3normal(local_alignxf_inv, normal);
      closest = this->FindClosestAlongDir(s,
                                          normal,
                                          max_dist_match2,
                                          thread_num);

      // discard points farther than 20 cm
      //     if (closest && sqrt(Dist2(closest, s)) > 20) closest = NULL;
    } else {
      closest = this->FindClosest(s, max_dist_match2, thread_num);
    }

    if (closest) {
      transform3(source_alignxf, closest, s);

      if (pairing_mode == CLOSEST_PLANE_SIMPLE) {
        // need to mutate s if we are looking for closest point-to-plane
        // s_ = (n,s-t)*n + t
        // to find the projection of s onto plane formed by normal n and point t
        double tmp[3], s_[3];
        double dot;
        sub3(s, t, tmp);
        dot = Dot(normal, tmp);
        scal_mul3(normal, dot, tmp);
        add3(tmp, t, s_);
        s[0] = s_[0];
        s[1] = s_[1];
        s[2] = s_[2];
      }

      // This should be right, model=Source=First=not moving
      centroid_m[0] += s[0];
      centroid_m[1] += s[1];
      centroid_m[2] += s[2];
      centroid_d[0] += t[0];
      centroid_d[1] += t[1];
      centroid_d[2] += t[2];

      PtPair myPair(s, t, normal);
      double p12[3] = { 
        myPair.p1.x - myPair.p2.x, 
        myPair.p1.y - myPair.p2.y,
        myPair.p1.z - myPair.p2.z };
      sum += Len2(p12);
      
      pairs->push_back(myPair);
    }
  }
  
  // release resource access lock
  unlock();

  return;
}
