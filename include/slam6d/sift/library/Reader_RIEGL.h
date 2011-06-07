/**
 * @file Reader_RIEGL.h
 * @brief read the txt files and creat the polarpointcloud
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef READER_RIEGL_H_
#define READER_RIEGL_H_

#include "PolarPointCloud.h"
#include "PointCloud.h"
#include "Reader.h"
#include <string>
#include "limits.h"

class Reader_RIEGL : Reader 
{
 public:
  Reader_RIEGL();
  virtual ~Reader_RIEGL();
  
  static PolarPointCloud readPolarPointCloud(std::string scanid, std::string filename);
  static PolarPointCloud readPolarPointCloud(std::string scanid, std::string filename, int n);
};
#endif /* READER_RIEGL_H_ */
