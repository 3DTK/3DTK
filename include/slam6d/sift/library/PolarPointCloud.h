/**
 * @file PolarPointCloud.h
 * @brief Create the vector PolarPoint and get the min and max of horizantal and vertical angles
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef POLARPOINTCLOUD_H_
#define POLARPOINTCLOUD_H_

#include "PolarPoint.h"
#include "PointC.h"
#include <vector>
#include <string>

class PolarPointCloud 
{
 public:
  PolarPointCloud(std::string scanid, std::vector<PolarPoint> *data, long length, double mina, double maxa, double minb, double maxb, double minz, double maxz);
  //?????
  PolarPointCloud(PointC* data, long length);
  PolarPointCloud(const char* filename, int readevery = 1); 
  virtual ~PolarPointCloud();
  
  void serialize(const char* filename);
  
  const std::vector<PolarPoint> *getData();
  long getLength() const;
  void getz (double minz, double maxz);
  
 public:
  
  double mina; //horizontal angle
  double maxa; //horizontal angle
  
  double minb; //vertical angle
  double maxb; //vertical angle
  
  double minz; //for z axis projection
  double maxz; //for z axis projection
  
  std::string scanid;
  std::vector<PolarPoint> *data;
  long length;
};
#endif /* POLARPOINTCLOUD_H_ */
