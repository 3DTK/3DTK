/*
 * Feature.h
 *
 *  Created on: April, 15, 2010
 *      Author: darko makreshanski
 */

#ifndef CONTROL_POINT_H_
#define CONTROL_POINT_H_

#include <vector>
#include <fstream>
#include "FeatureBase.h"

class Feature : public FeatureBase 
{
 public:
  Feature(){}
  Feature(double nx, double ny, double sc, double ori, std::vector<int> dsc) 
    {
      x = nx;
      y = ny;
      scale = sc; 
      orientation = ori; 
      descriptor = dsc;
    }
  void serialize(std::ofstream& ostr);
  Feature(std::ifstream& istr);
  virtual ~Feature() {}
  std::vector<int> descriptor;
};

#endif /* CONTROL_POINT_H_ */
