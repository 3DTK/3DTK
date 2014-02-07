/**
 * @file geoRefrencing.h
 * @brife geo refrence a scan with the set of corespondences.
 * This class geo refrence a scan with respect to the selected ponints.
 * It determines the transformation matrix.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @date Date: 2014/01/17 1:00
 */

#ifndef GEOREFRENCING_H_
#define GEOREFRENCING_H_

#include "fbr_global.h"
#include "slam6d/icp6Dquat.h"

using namespace std;

namespace fbr{
  /**
   * @class geoRefrencing : geo refrence a scan with selected points.
   */

  class geoRefrencing{
    double bestAlign[16];
    unsigned int maxInlier;
    float minError;
  
  private:
    
    /**
     * findAlign : find the best align
     * @param localPoints vetor that contains the selected points of the scan with local coordinate
     * @param geoPoints vector that contains the selected points of the scan with world coordinate
     */
    int findAlign(unsigned int i, unsigned int j, unsigned int k, vector< vector< float > > localPoints, vector< vector< float > > geoPoints);
    
    
  public:
    geoRefrencing(float _minError);
    
    /**
     * findRegistration : find the transformation matrix between the scan and world coordinate
     * @param localPoints vetor that contains the selected points of the scan with local coordinate
     * @param geoPoints vector that contains the selected points of the scan with world coordinate
     */
    void findRegistration(vector< vector< float > > localPoints, vector< vector< float > > geoPoints);
    

    double * getBestAlign();
    vector<double> getBestAlignVector();
    unsigned int getMaxInlier();
    void getDescription();
  };
}
#endif /* GEOREFRENCING_H_ */
