/**
 * @file ScanTransform.h
 * @brief transform the coordinates
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __SCAN_TRANSFORM_H__
#define __SCAN_TRANSFORM_H__

#include <string>

class ScanTransform
{
 public:
  ScanTransform (std::string scanid) 
    {
      this->scanid = scanid;
      for (int i = 0 ; i < 4 ; i++) 
	{
	  for (int j = 0 ; j < 4 ; j++) 
	    {
	      transform[i][j] = (i == j) ? 1 : 0;
	    }
	}
    }
  ScanTransform (std::string scanid, double nt[][4]) 
    {
      for (int i = 0 ; i < 4 ; i++) 
	{
	  for (int j = 0 ; j < 4 ; j++) 
	    {
	      transform[i][j] = nt[i][j];
	    }
	}
      this->scanid = scanid;
    }
  virtual ~ScanTransform () {}
  
  ScanTransform inverse();
  ScanTransform multiply(ScanTransform);
  
  std::string scanid;
  double transform[4][4];
};
#endif /* __SCAN_TRANSFORM_H__ */
