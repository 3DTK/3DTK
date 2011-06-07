/**
 * @file Reader_RIEGL.cc
 * @brief Implementation of Reader_RIEGL class
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/Reader_RIEGL.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#define M_PI 3.14159265

using namespace std;

//TODO
Reader_RIEGL::Reader_RIEGL()
{
  // TODO Auto-generated constructor stub
}

//TODO
Reader_RIEGL::~Reader_RIEGL()
{
  // TODO Auto-generated destructor stub
}

//call for PolarPointCloud Reader_RIEGL::readPolarPointCloud(string scanid, string filename, int countevery)
PolarPointCloud Reader_RIEGL::readPolarPointCloud(string scanid, string filename)
{
  return Reader_RIEGL::readPolarPointCloud(scanid, filename, 1);
}

//read the input file and create the polarpointcloud
PolarPointCloud Reader_RIEGL::readPolarPointCloud(string scanid, string filename, int countevery)
{
  if (countevery < 1) 
    {
      cerr << "Invalid entry for 'countevery' at Reader_RIEGL::readPolarPointCloud" << endl;
      exit(-1);
    }
  
  ifstream file;
  file.open(filename.c_str(), ios::in);
  if (!file.is_open()) 
    {
      throw FILE_NOT_FOUND;
    }
  
  long lines;
  file >> lines;
  long pointcount = lines / countevery;
  vector<PolarPoint>* data = new vector<PolarPoint>();
  char nullbuf[1024];
  
  double mind = INT_MAX;
  double maxd = INT_MIN;
  double minr = INT_MAX;
  double maxr = INT_MIN;
  double minb = INT_MAX;
  double maxb = INT_MIN;
  double mina = INT_MAX;
  double maxa = INT_MIN;
  //for finding a min and max value of x,y,z
  double minx = INT_MAX;
  double maxx = INT_MIN;
  double miny = INT_MAX;
  double maxy = INT_MIN;
  double minz = INT_MAX;
  double maxz = INT_MIN;
  
  try {
    float ta, tb, td, tr, tx, ty, tz;
    for (long l = 0; l < lines; ++l)
      {
	if (l % countevery == 0)
	  {
	    file.getline(nullbuf, 1024);
	    //sscanf(nullbuf, "%f %f %f %f %f %f %f", &tx, &ty, &tz, &td, &tb, &ta, &tr);
	    //read the file in slam6d format
	    sscanf(nullbuf, "%f %f %f %f %f %f %f", &tz, &tx, &ty, &td, &tb, &ta, &tr);
	    //horizantal angle of view of [0:360] and vertical of [-40:60]
	    ta = 360.0 - ta;
	    ta = ta * 2.0 * M_PI / 360.0;
	    tb -= 90;
	    tb *= -1;
	    tb *= 2.0 * M_PI / 360.0;
	    
	    if (tr > maxr) maxr = tr;
	    if (tr < minr) minr = tr;
	    
	    tr += 32;
	    tr /= 64;
	    tr -= 0.2;
	    tr /= 0.3;
	    
	    if (tr < 0) tr = 0;
	    if (tr > 1) tr = 1;
	    
	    //check for the vertical angle of view and eliminate the ones with more than 60
	    if((tb / 2 / M_PI * 360) <= 60)
	      {
		PolarPoint pp;
		pp.a = ta;
		pp.b = tb;
		pp.d = td;
		pp.r = tr;
		//change the x,y,z to the slam6d format
		//pp.x = tx;
		pp.x = tx * (-100);
		//pp.y = ty;
		pp.y = ty * 100;
		//pp.z = tz;
		pp.z = tz * 100;
		data->push_back(pp);
		
		if (td < mind) mind = td;
		if (td > maxd) maxd = td;
		if (ta < mina) mina = ta;
		if (ta > maxa) maxa = ta;
		if (tb < minb) minb = tb;
		if (tb > maxb) maxb = tb;
		if (tx < minx) minx = tx;
		if (tx > maxx) maxx = tx;
		if (ty < miny) miny = ty;
		if (ty > maxy) maxy = ty;
		if (tz < minz) minz = tz;
		if (tz > maxz) maxz = tz;
	      }
	    else
	      {
		cout << "tb = " << tb / 2 / M_PI * 360 << endl;
	      }
	  }
	else
	  {
	    file.getline(nullbuf, 1024);
	  }
      }
    cout << data->size() << endl;
    cout << minr << " refl " << maxr << endl;
    cout << mind << " dist " << maxd << endl;
    cout << minb / 2 / M_PI * 360 << " b " << maxb / 2 / M_PI * 360 << endl;
    cout << mina / 2 / M_PI * 360 << " a " << maxa / 2 / M_PI * 360 << endl;
    cout << minx << " x " << maxx << endl;
    cout << miny << " y " << maxy << endl;
    cout << minz << " z " << maxz << endl;
    
  } catch(void* e) {
    throw e;
  }
  file.close();
  //????????????????? the min and max of angels???  PolarPointCloud(std::string scanid, std::vector<PolarPoint> *data, long length, double mina, double maxa, double minb, double maxb);
  //return PolarPointCloud(scanid, data, pointcount, 0, 360, 0, 130, minz, maxz);
  return PolarPointCloud(scanid, data, pointcount, 0, 360, -40, 60, minz, maxz);
}
