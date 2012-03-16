/** 
 * @file 
 * @brief Representation of a general search trees
 * @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/searchTree.h"
#include "slam6d/globals.icc"

void SearchTree::getPtPairs(vector <PtPair> *pairs, 
    double *source_alignxf,                          // source
    double * const *q_points, unsigned int startindex, unsigned int nr_qpts,  // target
    int thread_num,
    int rnd, double max_dist_match2, double &sum,
    double *centroid_m, double *centroid_d, Scan *Target)
{
  centroid_m[0] = 0.0;
  centroid_m[1] = 0.0;
  centroid_m[2] = 0.0;
  centroid_d[0] = 0.0;
  centroid_d[1] = 0.0;
  centroid_d[2] = 0.0;

  double local_alignxf_inv[16];
  M4inv(source_alignxf, local_alignxf_inv);

  for (unsigned int i = startindex; i < (unsigned int)nr_qpts; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    transform3(local_alignxf_inv, q_points[i], p);

    double *closest = this->FindClosest(p, max_dist_match2, thread_num);
    if (closest) {
      transform3(source_alignxf, closest, p);
     
      // This should be right, model=Source=First=not moving
      centroid_m[0] += p[0];
      centroid_m[1] += p[1];
      centroid_m[2] += p[2];	 
      centroid_d[0] += q_points[i][0];
      centroid_d[1] += q_points[i][1];
      centroid_d[2] += q_points[i][2];

      PtPair myPair(p, q_points[i]);
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

  if (pairs->size() == 0) return;

  centroid_m[0] /= pairs->size();
  centroid_m[1] /= pairs->size();
  centroid_m[2] /= pairs->size();
  centroid_d[0] /= pairs->size();
  centroid_d[1] /= pairs->size();
  centroid_d[2] /= pairs->size();

  return;
}

