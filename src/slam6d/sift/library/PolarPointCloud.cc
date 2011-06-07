/**
 * @file PolarPointCloud.cc
 * @brief Implementation of PolarPoint class
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/PolarPointCloud.h"
#include <iostream>
#include <math.h>
#include <fstream>

using namespace std;

//creat polarpointcloud from input data 
PolarPointCloud::PolarPointCloud(string scanid, vector<PolarPoint> *_data, long length, double mina, double maxa, double minb, double maxb, double minz, double maxz)
{
  this->scanid = scanid;
  this->data = _data;
  this->length = length;
  this->mina = mina;
  this->maxa = maxa;
  this->minb = minb;
  this->maxb = maxb;
  this->minz = minz;
  this->maxz = maxz;
}

//create polarpointcloud read from a .ppc file
PolarPointCloud::PolarPointCloud(const char* filename, int readevery)
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
  
  in.read((char*)&length, sizeof(length));
  
  data = new vector<PolarPoint>();
  for (int i = 0 ; i < length ; i++)
    {
      PolarPoint p;
      double a,b,r,d,x,y,z;
      in.read((char*) &a, sizeof(a));
      in.read((char*) &b, sizeof(b));
      in.read((char*) &r, sizeof(r));
      in.read((char*) &d, sizeof(d));
      in.read((char*) &x, sizeof(x));
      in.read((char*) &y, sizeof(y));
      in.read((char*) &z, sizeof(z));
      p.a = a;
      p.b = b;
      p.r = r;
      p.d = d;
      p.x = x;
      p.y = y;
      p.z = z;
      if (i % readevery == 0)
	{
	  data->push_back(p);
	}
    }
  
  length = data->size();
  
  in.read((char*) &mina, sizeof(mina));
  in.read((char*) &maxa, sizeof(maxa));
  in.read((char*) &minb, sizeof(minb));
  in.read((char*) &maxb, sizeof(maxb));
  in.read((char*) &minz, sizeof(minz));
  in.read((char*) &maxz, sizeof(maxz));
  
  in.close();
}
//??????
PolarPointCloud::PolarPointCloud(PointC *tdata, long length )
{
  data = new vector<PolarPoint>(length);
  
  double a,b,d,x,y,z;
  for (int i = 0 ; i < length ; ++i) 
    {
      x = tdata[i].x;
      y = tdata[i].y;
      z = tdata[i].z;
      d = sqrt(x*x + y*y + z*z);
      a = atan2(z,x);
      b = asin(y / d);
      (*data)[i].a = a;
      (*data)[i].b = b;
      (*data)[i].d = d;
      (*data)[i].r = tdata[i].r;
      (*data)[i].x = tdata[i].x;
      (*data)[i].y = tdata[i].y;
      (*data)[i].z = tdata[i].z;
    }
  
  this->length = length; 
}

long PolarPointCloud::getLength() const
{
  return this->length;
}

const vector<PolarPoint> *PolarPointCloud::getData()
{
  return data;
}

void PolarPointCloud::getz(double minz, double maxz)
{
  minz = this->minz;
  maxz = this->maxz;
}

PolarPointCloud::~PolarPointCloud() 
{

}

//create a binary file .ppc and write the polarpointcloud to it
void PolarPointCloud::serialize(const char* filename)
{
  ofstream out(filename, ios::binary);
  
  int stringsize = scanid.size();
  out.write((char*) &stringsize, sizeof(stringsize));
  for (int i = 0 ; i < stringsize ; i++) 
    {
      char s = scanid[i];
      out.write((char*)&s, sizeof(s));
    }
  
  out.write((char*) &length, sizeof(length));
  for (int i = 0 ; i < length ; i++) 
    {
      double a,b,r,d,x,y,z;
      PolarPoint p = (*data)[i];
      a = p.a;
      b = p.b;
      r = p.r;
      d = p.d;
      x = p.x;
      y = p.y;
      z = p.z;
      out.write((char*) &a, sizeof(a));
      out.write((char*) &b, sizeof(b));
      out.write((char*) &r, sizeof(r));
      out.write((char*) &d, sizeof(d));
      out.write((char*) &x, sizeof(x));
      out.write((char*) &y, sizeof(y));
      out.write((char*) &z, sizeof(z));
    }
  out.write((char*) &mina, sizeof(mina));
  out.write((char*) &maxa, sizeof(maxa));
  out.write((char*) &minb, sizeof(minb));
  out.write((char*) &maxb, sizeof(maxb));
  out.write((char*) &minz, sizeof(minz));
  out.write((char*) &maxz, sizeof(maxz));
  
  out.close();
}
