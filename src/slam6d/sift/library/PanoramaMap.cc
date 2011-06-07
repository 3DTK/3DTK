/**
 * @file PanoramaMap.cc
 * @brief Implementation of Class PanoramaMap, Implementation of Equirectangular,Cylindrical,zaxis,Mercator,
 *        Rectilinear, Pannini projections
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/PanoramaMap.h"
#include <iostream>
#include <fstream>
#include <new>
#include <vector>
#include <vigra/imageinfo.hxx>
#include <vigra/impex.hxx>
#include <math.h>

using namespace std;

//Vertical angle of view of scanner
#define MAX_ANGLE  60.0
#define MIN_ANGLE  -40.0

PanoramaMap::PanoramaMap()
{
}

//Implementation of projections
PanoramaMap::PanoramaMap(PolarPointCloud* cloud, int nwidth, int nheight, int method, double panninid, int imagen, double stereor)
{
  scanid = cloud->scanid;

  //for adding the mean values to super pixel
  //vector< vector<int> > index(nwidth, vector<int>(nheight, 0));

  data = new SuperPixel*[nwidth];
  for (int i = 0 ; i < nwidth ; i++)
    {
      data[i] = new SuperPixel[nheight];
      for (int j = 0 ; j < nheight ; j++)
	{
	  data[i][j].value = 0;
	  data[i][j].meta = 0;
	  data[i][j].x = 0;
	  data[i][j].y = 0;
	  data[i][j].z = 0;
	}
    }
  
  if( method == STANDARD)
    {
      long length = cloud->getLength();
      const vector<PolarPoint> *pdata = cloud->getData();
      //adding the langitude to x axis and latitude to y axis
      double xmaxt = (double) nwidth / 2 / M_PI;
      int xmax = nwidth - 1;
      double ymaxt = (double) nheight / ((MAX_ANGLE - MIN_ANGLE) / 360 * 2 * M_PI);
      //shift all the valuse to positive points on image 
      double ylow = (0 - MIN_ANGLE) / 360 * 2 * M_PI;
      int ymax = nheight - 1;
      
      for (int i = 0 ; i < length ; i++)
        {
	  int x = (int) ( xmaxt * (*pdata)[i].a);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ( ymaxt * ((*pdata)[i].b + ylow) );
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  //for finding the mean values
	  /*
            index[x][y] = index[x][y] + 1;
            data[x][y].value += (*pdata)[i].r;
            data[x][y].meta += (*pdata)[i].d;
            data[x][y].x += (*pdata)[i].x;
            data[x][y].y += (*pdata)[i].y;
            data[x][y].z += (*pdata)[i].z;
	  */
	  //adding the point with max distance
	  if( data[x][y].meta < (*pdata)[i].d )
            {
	      data[x][y].value = (*pdata)[i].r;
	      data[x][y].meta = (*pdata)[i].d;
	      data[x][y].x = (*pdata)[i].x;
	      data[x][y].y = (*pdata)[i].y;
	      data[x][y].z = (*pdata)[i].z;
	    }
        }
      projection = STANDARD;
    }
  //cylindrical projection
  if(method == CYLINDRICAL)
    {
      long length = cloud->getLength();
      const vector<PolarPoint> *pdata = cloud->getData();
      //adding the longitude to x and tan(latitude) to y
      //find the x and y range
      double xmaxt = (double) nwidth / 2 / M_PI;
      int xmax = nwidth - 1;
      double ymaxt = (double) nheight / (tan(MAX_ANGLE / 360 * 2 * M_PI) - tan(MIN_ANGLE / 360 * 2 * M_PI));
      double ylow = (MIN_ANGLE) / 360 * 2 * M_PI;
      int ymax = nheight - 1;
      //go through all points and find the x and y of image pixel coresponding to them
      for (int i = 0 ; i < length ; i++)
        {
	  int x = (int) ( xmaxt * (*pdata)[i].a);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ((double) ymaxt * (tan((*pdata)[i].b) - tan(ylow)));
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  //adding the point with max distance
	  if( data[x][y].meta < (*pdata)[i].d )
            {
	      data[x][y].value = (*pdata)[i].r;
	      data[x][y].meta = (*pdata)[i].d;
	      data[x][y].x = (*pdata)[i].x;
	      data[x][y].y = (*pdata)[i].y;
	      data[x][y].z = (*pdata)[i].z;
            }
        }
      projection = CYLINDRICAL;
    }
  //Z axes Projection
  if(method == NEW)
    {
      long length = cloud->getLength();
      const vector<PolarPoint> *pdata = cloud->getData();
      //find the x and y range and add longitude to x and z to y
      double xmaxt = (double) nwidth / 2 / M_PI;
      int xmax = nwidth - 1;
      double ymaxt = (double) nheight / (cloud->maxz - cloud->minz);
      // double ymaxt = (double) nheight / ((3200));
      //double ymaxt = (double) nheight / ((20m) / maxmetr);
      int ymax = nheight - 1;
      double ylow = cloud->minz;
      //double ylow = (200);
      for (int i = 0 ; i < length ; i++)
        {
	  int x = (int) ( xmaxt * (*pdata)[i].a);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ( ymaxt * ((*pdata)[i].y - ylow) );
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  /*
	  int y;
	  if((*pdata)[i].y < 3000)
            {
	      y = (int) (ymaxt * (ylow + (*pdata)[i].y ));
	      if (y < 0) y = 0;
	      if (y > ymax) y = ymax;
            }
	  else
            {
	      y = (int) (ymaxt * 3200);
	      if (y < 0) y = 0;
	      if (y > ymax) y = ymax;
            }
	  */
	  //add the point with max distance
	  if( data[x][y].meta < (*pdata)[i].d )
            {
	      data[x][y].value = (*pdata)[i].r;
	      data[x][y].meta = (*pdata)[i].d;
	      data[x][y].x = (*pdata)[i].x;
	      data[x][y].y = (*pdata)[i].y;
	      data[x][y].z = (*pdata)[i].z;
            }
        }
      projection = NEW;
    }
  //Mercator Projection
  if( method == MERCATOR)
    {
      long length = cloud->getLength();
      const vector<PolarPoint> *pdata = cloud->getData();
      //find the x and y range
      double xmaxt = (double) nwidth / 2 / M_PI;
      int xmax = nwidth - 1;
      double ymaxt = (double) nheight / ( log( tan( MAX_ANGLE / 360 * 2 * M_PI ) + ( 1 / cos( MAX_ANGLE / 360 * 2 * M_PI ) ) ) - log ( tan( MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI) ) ) );
      double ylow = log(tan(MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI)));
      int ymax = nheight - 1;
      //go through all points and find the x and y
      for (int i = 0 ; i < length ; i++)
        {
	  int x = (int) ( xmaxt * (*pdata)[i].a);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ( ymaxt * (log(tan((*pdata)[i].b) + (1/cos((*pdata)[i].b))) - ylow) );
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  //add the point with max distance
	  if( data[x][y].meta < (*pdata)[i].d )
            {
	      data[x][y].value = (*pdata)[i].r;
	      data[x][y].meta = (*pdata)[i].d;
	      data[x][y].x = (*pdata)[i].x;
	      data[x][y].y = (*pdata)[i].y;
	      data[x][y].z = (*pdata)[i].z;
            }
        }
      projection = MERCATOR;
    }
  //Rectilinear Projection
  if(method == RECTILINEAR)
    {
      long length = cloud->getLength();
      const vector<PolarPoint> *pdata = cloud->getData(); 
      //go through all points 
      for (int i = 0 ; i < length ; i++)
	{
	  //check for point in the first 90 degree
	  if((*pdata)[i].a < (M_PI/2) && (*pdata)[i].a > 0)
	    {
	      double max, min, cm;
	      //the longitude and latitude of projection center
	      double l0 = M_PI/4;
	      double p1 = 0;
	      //finding the min and max of the x direction
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(M_PI/2 - l0);
	      max = (cos(M_PI/3) * sin(M_PI/2 -l0) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(0 - l0);
	      min = (cos(-M_PI/3) * sin(0 - l0) / cm);
	      double xmaxt = (double) (nwidth / 4) / (max -min);
	      double xlow = min;
	      int xmax = (nwidth/4) - 1;
	      //finding the min and max of y direction
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(M_PI/2 - l0);
	      max = ( (cos(p1) * sin(M_PI/3) - sin(p1) * cos(M_PI/3) * cos(M_PI/2 - l0) )/ cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(0 - l0);
	      min = ( (cos(p1) * sin(-M_PI/3) - sin(p1) * cos(-M_PI/3) * cos(0 - l0) )/ cm);
	      double ymaxt = (double) nheight / (max - min);
	      double ylow = min;
	      int ymax = nheight - 1;
	      //project the points and add them to image
	      double c = sin(p1) * sin((*pdata)[i].b) + cos(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0);
	      int x = (int)(xmaxt) * ((cos((*pdata)[i].b) * sin((*pdata)[i].a - l0) / c) - xlow);
	      if (x < 0) x = 0;
	      if (x > xmax) x = xmax;
	      int y = (int) (ymaxt) * (( (cos(p1) * sin((*pdata)[i].b) - sin(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0)) / c) - ylow);
	      if (y < 0) y = 0;
	      if (y > ymax) y = ymax;
	      //add the point with max distance
	      if( data[x][y].meta < (*pdata)[i].d )
		{
		  data[x][y].value = (*pdata)[i].r;
		  data[x][y].meta = (*pdata)[i].d;
		  data[x][y].x = (*pdata)[i].x;
		  data[x][y].y = (*pdata)[i].y;
		  data[x][y].z = (*pdata)[i].z;
		}
	    }
	  //same as first interval
	  if((*pdata)[i].a < (M_PI) && (*pdata)[i].a > (M_PI/2))
	    {
	      double max, min, cm;
	      double l0 = M_PI/2 + M_PI/4;
	      double p1 = 0;
	      
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(M_PI - l0);
	      max = (cos(M_PI/3) * sin(M_PI -l0) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(M_PI/2 - l0);
	      min = (cos(-M_PI/3) * sin(M_PI/2 - l0) / cm);
	      double xmaxt = (double) (nwidth / 4) / (max -min);
	      double xlow = min;
	      int xmax = (nwidth/4) - 1;
	   
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(M_PI - l0);
	      max = ((cos(p1) * sin(M_PI/3) - sin(p1) * cos(M_PI/3) * cos(M_PI - l0)) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(M_PI/2 - l0);
	      min = ((cos(p1) * sin(-M_PI/3) - sin(p1) * cos(-M_PI/3) * cos(M_PI/2 - l0)) / cm);
	      double ymaxt = (double) nheight / (max - min);
	      double ylow = min;
	      int ymax = nheight - 1;
	      
	      double c = sin(p1) * sin((*pdata)[i].b) + cos(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0);
	      int x = (int)(xmaxt) * ((cos((*pdata)[i].b) * sin((*pdata)[i].a - l0) / c) - xlow);
	      if (x < 0) x = 0;
	      if (x > xmax) x = xmax;
	      x = x + (nwidth/4);
	      int y = (int) (ymaxt) * (((cos(p1) * sin((*pdata)[i].b) - sin(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0)) / c) - ylow);
	      if (y < 0) y = 0;
	      if (y > ymax) y = ymax;
	      if( data[x][y].meta < (*pdata)[i].d )
		{
		  data[x][y].value = (*pdata)[i].r;
		  data[x][y].meta = (*pdata)[i].d;
		  data[x][y].x = (*pdata)[i].x;
		  data[x][y].y = (*pdata)[i].y;
		  data[x][y].z = (*pdata)[i].z;
		}
	    }
	  //same as fisrt interval
	  if((*pdata)[i].a < (3*M_PI/2) && (*pdata)[i].a > (M_PI))
	    {
	      double max, min, cm;
	      double l0 = M_PI + M_PI/4;
	      double p1 = 0;
	     
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(3*M_PI/2 - l0);
	      max = (cos(M_PI/3) * sin(3*M_PI/2 -l0) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(M_PI - l0);
	      min = (cos(-M_PI/3) * sin(M_PI - l0) / cm);
	      double xmaxt = (double) (nwidth / 4) / (max -min);
	      double xlow = min;
	      int xmax = (nwidth/4) - 1;
	      
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(3*M_PI/2 - l0);
	      max = ((cos(p1) * sin(M_PI/3) - sin(p1) * cos(M_PI/3) * cos(3*M_PI/2 - l0)) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(M_PI - l0);
	      min = ((cos(p1) * sin(-M_PI/3) - sin(p1) * cos(-M_PI/3) * cos(M_PI - l0)) / cm);
	      double ymaxt = (double) nheight / (max - min);
	      double ylow = min;
	      int ymax = nheight - 1;
	      
	      double c = sin(p1) * sin((*pdata)[i].b) + cos(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0);
	      int x = (int)(xmaxt) * ((cos((*pdata)[i].b) * sin((*pdata)[i].a - l0) / c) - xlow);
	      if (x < 0) x = 0;
	      if (x > xmax) x = xmax;
	      x = x + 2*(nwidth/4);
	      int y = (int) (ymaxt) * (((cos(p1) * sin((*pdata)[i].b) - sin(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0)) / c) - ylow);
	      if (y < 0) y = 0;
	      if (y > ymax) y = ymax;
	      if( data[x][y].meta < (*pdata)[i].d )
		{
		  data[x][y].value = (*pdata)[i].r;
		  data[x][y].meta = (*pdata)[i].d;
		  data[x][y].x = (*pdata)[i].x;
		  data[x][y].y = (*pdata)[i].y;
		  data[x][y].z = (*pdata)[i].z;
		}
	    }
	  //same as first interval
	  if((*pdata)[i].a < (2*M_PI) && (*pdata)[i].a > (3*M_PI/2))
	    {
	      double max, min, cm;
	      double l0 = 3*M_PI/2 + M_PI/4;
	      double p1 = 0;
	      
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(2*M_PI - l0);
	      max = (cos(M_PI/3) * sin(2*M_PI -l0) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(3*M_PI/2 - l0);
	      min = (cos(-M_PI/3) * sin(3*M_PI/2 - l0) / cm);
	      double xmaxt = (double) (nwidth / 4) / (max -min);
	      double xlow = min;
	      int xmax = (nwidth/4) - 1;
	      
	      cm = sin(p1) * sin(M_PI/3) + cos(p1) * cos(M_PI/3) * cos(2*M_PI - l0);
	      max = ((cos(p1) * sin(M_PI/3) - sin(p1) * cos(M_PI/3) * cos(2*M_PI - l0)) / cm);
	      cm = sin(p1) * sin(-M_PI/3) + cos(p1) * cos(-M_PI/3) * cos(3*M_PI/2 - l0);
	      min = ((cos(p1) * sin(-M_PI/3) - sin(p1) * cos(-M_PI/3) * cos(3*M_PI/2 - l0)) / cm);	      
	      double ymaxt = (double) nheight / (max - min);
	      double ylow = min;
	      int ymax = nheight - 1;
	      
	      double c = sin(p1) * sin((*pdata)[i].b) + cos(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0);
	      int x = (int)(xmaxt) * ((cos((*pdata)[i].b) * sin((*pdata)[i].a - l0) / c) - xlow);
	      if (x < 0) x = 0;
	      if (x > xmax) x = xmax;
	      x = x + 3*(nwidth/4);
	      int y = (int) (ymaxt) * (((cos(p1) * sin((*pdata)[i].b) - sin(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0)) / c) - ylow);
	      if (y < 0) y = 0;
	      if (y > ymax) y = ymax;
	      if( data[x][y].meta < (*pdata)[i].d )
		{
		  data[x][y].value = (*pdata)[i].r;
		  data[x][y].meta = (*pdata)[i].d;
		  data[x][y].x = (*pdata)[i].x;
		  data[x][y].y = (*pdata)[i].y;
		  data[x][y].z = (*pdata)[i].z;
		}
	    }
	}
      projection = RECTILINEAR;
    }
  //PANNINI Projection
  if(method == PANNINI)
    {
      long length = cloud->getLength();
      //double d = 0;
      d = panninid;
      n = imagen;
      cout << "Parameter d is:" << d <<", Number of images per scan is:" << n << endl;
      const vector<PolarPoint> *pdata = cloud->getData(); 
      double l0, p1, iminx, imaxx, iminy, imaxy, interval;
      interval = 2 * M_PI / n;
      //iminy = -M_PI/3;
      iminy = -2*M_PI/9;
      imaxy = M_PI/3;
      //latitude of projection center
      p1 = 0;
      //go through all points 
      for (int i = 0 ; i < length ; i++)
	{
	  for(int j = 0 ; j < n ; j++)
	    {
	      iminx = j * interval;
	      imaxx = (j +1) * interval;
	      //check for point in interval
	      if((*pdata)[i].a < (imaxx) && (*pdata)[i].a > (iminx))
		{
		  double max, min, s;
		  //the longitude of projection center
		  l0 = iminx + interval / 2;
		  // p1 = 0;
		  //finding the min and max of the x direction
		  //use the S variable of pannini projection mentioned in the thesis
		  s = (d + 1) / (d + sin(p1) * tan(imaxy) + cos(p1) * cos(imaxx - l0));
		  max = s * (sin(imaxx - l0));
		  s = (d + 1) / (d + sin(p1) * tan(iminy) + cos(p1) * cos(iminx - l0));
		  min = s * (sin(iminx - l0));
		  double xmaxt = (double) (nwidth / n) / (max -min);
		  double xlow = min;
		  int xmax = (nwidth/n) - 1;
		  //finding the min and max of y direction
		  s = (d + 1) / (d + sin(p1) * tan(imaxy) + cos(p1) * cos(imaxx - l0));
		  max = s * (tan(imaxy) * (cos(p1) - sin(p1) * 1/tan(imaxy) * cos(imaxx - l0)));
		  s = (d + 1) / (d + sin(p1) * tan(iminy) + cos(p1) * cos(iminx - l0));
		  min = s * (tan(iminy) * (cos(p1) - sin(p1) * 1/tan(iminy) * cos(iminx - l0)));
		  double ymaxt = (double) nheight / (max - min);
		  double ylow = min;
		  int ymax = nheight - 1;
		  //project the points and add them to image
		  s = (d + 1) / (d + sin(p1) * tan((*pdata)[i].b) + cos(p1) * cos((*pdata)[i].a - l0));
		  int x = (int)(xmaxt) * (s * sin((*pdata)[i].a - l0) - xlow);
		  if (x < 0) x = 0;
		  if (x > xmax) x = xmax;
		  x = x + (j * nwidth / n);
		  int y = (int) (ymaxt) * ( (s * tan((*pdata)[i].b) * (cos(p1) - sin(p1) * (1/tan((*pdata)[i].b)) * cos((*pdata)[i].a - l0) ) ) - ylow );
		  if (y < 0) y = 0;
		  if (y > ymax) y = ymax;
		  //add the point with max distance
		  if( data[x][y].meta < (*pdata)[i].d )
		    {
		      data[x][y].value = (*pdata)[i].r;
		      data[x][y].meta = (*pdata)[i].d;
		      data[x][y].x = (*pdata)[i].x;
		      data[x][y].y = (*pdata)[i].y;
		      data[x][y].z = (*pdata)[i].z;
		    }
		}
	    }
	}
      projection = PANNINI;
    }
  //Stereographic Projection
  if(method == STEREOGRAPHIC)
    {
      long length = cloud->getLength();
      r = stereor;
      n = imagen;
      cout << "Paremeter r is:" << r << ", Number of images per scan is:" << n << endl;
      const vector<PolarPoint> *pdata = cloud->getData();
      // l0 and p1 are the center of projection iminx, imaxx, iminy, imaxy are the bounderis of intervals
      double l0, p1, iminx, imaxx, iminy, imaxy, interval;
      interval = 2 * M_PI / n;
      //iminy = -M_PI/3;
      iminy = -2*M_PI/9;
      imaxy = M_PI/3;
      //latitude of projection center
      p1 = 0;
      //go through ll points
      for ( int i = 0 ; i < length ; i++)
	{
	  for ( int j = 0 ; j < n ; j++)
	    {
	      iminx = j * interval;
	      imaxx = (j+1) * interval;
	      //check for point in intervals
	      if((*pdata)[i].a < (imaxx) && (*pdata)[i].a > (iminx))
		{
		  double max, min, k;
		  //longitude of projection center
		  l0 = iminx + interval / 2;
		  //finding the min and max of x direction
		  //use the k variable of stereographic projection mentioned in the thesis
		  k = (2 * r) / (1 + sin(p1) * sin(p1) + cos(p1) * cos(p1) * cos(imaxx - l0));
		  max = k * cos(p1) * sin (imaxx - l0);
		  k = (2 * r) / (1 + sin (p1) * sin(p1) + cos(p1) * cos(p1) * cos(iminx -l0));
		  min = k * cos(p1) * sin (iminx -l0);
		  double xmaxt = (double) (nwidth / n) / (max - min);
		  double xlow = min;
		  int xmax = (nwidth / n) - 1;
		  //finding the min and max of y direction
		  k = (2 * r) / (1 + sin(p1) * sin(imaxy) + cos(p1) * cos(imaxy) * cos(imaxx - l0));
		  max = k * (cos(p1) * sin(imaxy) - sin(p1) * cos(imaxy) * cos(imaxx - l0));
		  k = (2 * r) / (1 + sin(p1) * sin(iminy) + cos(p1) * cos(iminy) * cos(iminx - l0));
		  min = k * (cos(p1) * sin(iminy) - sin(p1) * cos(iminy) * cos(iminx - l0));
		  double ymaxt = (double) nheight / (max - min);
		  double ylow = min;
		  int ymax = nheight - 1;
		  //project the points and add them to image
		  k = (2 * r) / (1 + sin(p1) * sin((*pdata)[i].b) + cos(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0));
		  int x = (int) (xmaxt) * (k * cos((*pdata)[i].b) * sin((*pdata)[i].a - l0) - xlow);
		  if (x < 0) x = 0;
		  if (x > xmax) x = xmax;
		  x = x + (j * nwidth / n);
		  int y = (int) (ymaxt) * (k * ( cos(p1) * sin((*pdata)[i].b) - sin(p1) * cos((*pdata)[i].b) * cos((*pdata)[i].a - l0) ) - ylow);
		  if (y < 0) y = 0;
		  if (y > ymax) y = ymax;
		  //add the point with max distance
		  if (data[x][y].meta < (*pdata)[i].d)
		    {
		      data[x][y].value = (*pdata)[i].r;
		      data[x][y].meta = (*pdata)[i].d;
		      data[x][y].x = (*pdata)[i].x;
		      data[x][y].y = (*pdata)[i].y;
		      data[x][y].z = (*pdata)[i].z;
		    }
		}
	    }
	}
      projection = STEREOGRAPHIC;
    }
  //use the added values and index to find the mean 
  /*
    int ind;
    for (int i = 0 ; i < nwidth ; i++)
    {
    for (int j = 0 ; j < nheight ; j++)
    {
    ind = index[i][j];
    if (ind != 0)
    {
    data[i][j].value /= ind;
    data[i][j].meta /= ind;
    data[i][j].x /= ind;
    data[i][j].y /= ind;
    data[i][j].z /= ind;
    }
    else
    {
    data[i][j].value = 0;
    data[i][j].meta = 0;
    data[i][j].x = 0;
    data[i][j].y = 0;
    data[i][j].z = 0;
    }
    }
    }
  */
  width = nwidth;
  height = nheight;
  
  mina = cloud->mina;
  maxa = cloud->maxa;
  minb = cloud->minb;
  maxb = cloud->maxb;
  minz = cloud->minz;
  maxz = cloud->maxz;
}

//read the .map file and create PanaromaMap
PanoramaMap::PanoramaMap(const char* filename)
{
  ifstream in(filename, ios::binary);
  if (!in.good()) {cerr << "File not found" << endl; throw 1; }
  
  int stringsize;
  in.read((char*) &stringsize, sizeof(stringsize));
  scanid = "";
  for (int i = 0 ; i < stringsize ; i++)
    {
      char s;
      in.read((char*)&s, sizeof(s));
      scanid += s;
    }
  
  in.read((char*)&width, sizeof(width));
  in.read((char*)&height, sizeof(height));
  
  data = new SuperPixel*[width];
  for (int i = 0 ; i < width ; i++)
    {
      data[i] = new SuperPixel[height];
    }
  
  for (int x = 0 ; x < width ; x++)
    {
      for (int y = 0 ; y < height ; y++)
	{
	  in.read((char *) &(data[x][y]), sizeof(data[x][y]));
	}
    }
  
  in.read((char*) &mina, sizeof(mina));
  in.read((char*) &maxa, sizeof(maxa));
  in.read((char*) &minb, sizeof(minb));
  in.read((char*) &maxb, sizeof(maxb));
  in.read((char*) &minz, sizeof(minz));
  in.read((char*) &maxz, sizeof(maxz));
  in.read((char*) &projection, sizeof(projection));
  
  in.close();
}

PanoramaMap::~PanoramaMap() 
{
  for (int i = 0 ; i < width; i++) 
    {
      delete [] data[i];
    }
  delete [] data;
}

//create the panaroma Jpeg of Range or Refelctance data
void PanoramaMap::toJpeg(std::string filename, int method)
{
  vigra::ImageExportInfo info(filename.c_str());
  info.setCompression("90");
  
  vigra::BRGBImage img(width, height);
  //creating the range image
  if(method == RANGE)
    {
      for (int x = 0 ; x < width ; x++)
        {
	  for (int y = 0 ; y < height ; y++)
            {
	      img(x,height - y - 1) = vigra::RGBValue<unsigned char>((1-log10(data[x][y].meta)/2.6) * 255);
            }
        }
    }
  //creating the refelectance image
  if(method == REFELCTANCE)
    {
      for (int x = 0 ; x < width ; x++)
        {
	  for (int y = 0 ; y < height ; y++)
            {
	      img(x,height - y - 1) = vigra::RGBValue<unsigned char>(data[x][y].value * 255);
            }
        }
    }
  
  vigra::exportImage(vigra::srcImageRange(img), info);
}

//serialize the PanoramaMap to a file
void PanoramaMap::serialize(const char* filename)
{
  string str(filename);
  cout << filename << endl;
  ofstream out(filename, ios::binary);
  int stringsize = scanid.size();
  out.write((char*) &stringsize, sizeof(stringsize));
  for (int i = 0 ; i < stringsize ; i++)
    {
      char s = scanid[i];
      out.write((char*)&s, sizeof(s));
    }
  
  out.write((char *) &width, sizeof(width));
  out.write((char *) &height, sizeof(height));
  for (int x = 0 ; x < width ; x++) 
    {
      for (int y = 0 ; y < height ; y++) 
	{
	  out.write((char *) &(data[x][y]), sizeof(data[x][y]));
	}
    }
  
  out.write((char*) &mina, sizeof(mina));
  out.write((char*) &maxa, sizeof(maxa));
  out.write((char*) &minb, sizeof(minb));
  out.write((char*) &maxb, sizeof(maxb));
  out.write((char*) &minz, sizeof(minz));
  out.write((char*) &maxz, sizeof(maxz));
  out.write((char*) &projection, sizeof(projection));
  
  out.close();
}
