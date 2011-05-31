/**
 * Reader_RIEGL.h
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 *      Maintained : HamidReza May 04, 2011
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
