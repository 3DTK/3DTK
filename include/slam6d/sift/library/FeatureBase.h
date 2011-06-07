/**
 * @file FeatureBase.h
 * @brief FeatureBase is an entity with x, y, scale, orientation
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

#include <vector>
#include <fstream>

class FeatureBase 
{
 public:
  FeatureBase(){}
  FeatureBase(double nx, double ny, double sc, double ori) 
    {
      x = nx;
      y = ny;
      scale = sc; 
      orientation = ori; 
    }
  void serialize(std::ofstream& ostr);
  FeatureBase(std::ifstream& istr);
  virtual ~FeatureBase() {}
  double x;
  double y;
  double scale;
  double orientation;
};

#endif /* CONTROL_POINT_H_ */
