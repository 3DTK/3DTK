/*
 * geoRefrencing implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/geoRefrencing.h"

using namespace std;

namespace fbr{
  
  geoRefrencing::geoRefrencing(float _minError)
  {
    minError = _minError;
    maxInlier = 0;
    for(unsigned int i = 0; i < 16; i++)
      {
	bestAlign[i] = 0;
      }
  }
  
  void geoRefrencing::findRegistration(vector< vector< float > > localPoints, vector < vector< float > > geoPoints)
  {
    for(unsigned int i = 0; i < localPoints.size(); i++)
      {
	for(unsigned int j = 0; j < localPoints.size(); j++)
	  {
	    for(unsigned int k = 0; k < localPoints.size(); k++)
	      {
		findAlign(i, j, k, localPoints, geoPoints);
	      }
	  }
      }
  }

  int geoRefrencing::findAlign(unsigned int i, unsigned int j, unsigned int k, vector< vector< float > > localPoints, vector< vector< float > > geoPoints)
  {
    if(i == j || i == k || j == k)
      return 0;
    
    //calculate the centroids
    double centroidl[3], centroidg[3];
    for(unsigned int l = 0; l < 3; l++)
      {
	centroidl[l] = (localPoints[i][l] + localPoints[j][l] + localPoints[k][l]) / 3;
	centroidg[l] = (geoPoints[i][l] + geoPoints[j][l] + geoPoints[k][l]) / 3;
      }

    //create PtPair and calc the align with icp6D_QUAT
    vector<PtPair> pairs;
    double geoPoint1[3], geoPoint2[3], geoPoint3[3];
    double localPoint1[3], localPoint2[3], localPoint3[3];
    for(int l = 0; l < 3; l++)
      {
	geoPoint1[l] = geoPoints[i][l];
	geoPoint2[l] = geoPoints[j][l];
	geoPoint3[l] = geoPoints[k][l];

	localPoint1[l] = localPoints[i][l];
	localPoint2[l] = localPoints[j][l];
	localPoint3[l] = localPoints[k][l];
      }
    pairs.push_back(PtPair(geoPoint1, localPoint1));
    pairs.push_back(PtPair(geoPoint2, localPoint2));
    pairs.push_back(PtPair(geoPoint3, localPoint3));
    double align[16];
    icp6D_QUAT q(true);
    q.Align(pairs, align, centroidg, centroidl);

    //transform the the points with align if the error is less than minerror
    unsigned int eIdx = 0;
    for(unsigned int p = 0; p < localPoints.size(); p++)
      {
	if(p == i || p == j || p == k)
	  continue;
      
	//get a new point
	double geoPoint[3], localPoint[3];
	for(unsigned int l = 0; l < 3; l++)
	  {
	    geoPoint[l] = geoPoints[p][l];
	    localPoint[l] = localPoints[p][l];
	  }

	//transform the local point with the align to global coordinate and check with the global point
	double local_geoPoint[3];
	local_geoPoint[0] = align[0] * localPoint[0] + align[4] * localPoint[1] + align[8] * localPoint[2] + align[12];
	local_geoPoint[1] = align[1] * localPoint[0] + align[5] * localPoint[1] + align[9] * localPoint[2] + align[13];
	local_geoPoint[2] = align[2] * localPoint[0] + align[6] * localPoint[1] + align[10] * localPoint[2] + align[14];

	cv::Point3f testPoint;
	testPoint.x = local_geoPoint[0] - geoPoint[0];
	testPoint.y = local_geoPoint[1] - geoPoint[1];
	testPoint.z = local_geoPoint[2] - geoPoint[2];
      
	if(norm(testPoint) < minError)
	  {
	    eIdx++;
	  }
      }
    
    //check for maxnlier and find the best align
    if(eIdx >= maxInlier)
      { 
	maxInlier = eIdx;
      	for(int a = 0; a < 16; a++) 
	  bestAlign[a] = align[a];
      }
    return 1;
  }

  double * geoRefrencing::getBestAlign()
  {
    return bestAlign;
  }

  vector<double> geoRefrencing::getBestAlignVector()
  {
    vector<double> align;
    for(unsigned int i = 0; i < 16; i++)
      {
	align.push_back(bestAlign[i]);
      }
    return align;
  }
  
  unsigned int geoRefrencing::getMaxInlier()
  {
    return maxInlier;
  }

  void geoRefrencing::getDescription(){
    cout<<"Registration minError: "<<minError<<", maxInlier: "<<maxInlier<<"."<<endl;
    cout<<"align Matrix:"<<endl;
    cout<<bestAlign[0]<<"  "<<bestAlign[4]<<"  "<<bestAlign[8]<<"  "<<bestAlign[12]<<endl;
    cout<<bestAlign[1]<<"  "<<bestAlign[5]<<"  "<<bestAlign[9]<<"  "<<bestAlign[13]<<endl;
    cout<<bestAlign[2]<<"  "<<bestAlign[6]<<"  "<<bestAlign[10]<<"  "<<bestAlign[14]<<endl;
    cout<<bestAlign[3]<<"  "<<bestAlign[7]<<"  "<<bestAlign[11]<<"  "<<bestAlign[15]<<endl;
    cout<<endl; 
  }

}

