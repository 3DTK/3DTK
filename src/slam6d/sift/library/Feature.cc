/**
 * @file Feature.cc
 * @brief Implementation of Class Feature
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include <fstream>
#include "slam6d/sift/library/Feature.h"

using namespace std;

void Feature::serialize(ofstream& out)
{
  out.write((char*)&x, sizeof(x));
  out.write((char*)&y, sizeof(y));
  out.write((char*)&scale, sizeof(scale));
  out.write((char*)&orientation, sizeof(orientation));
  int dsize = descriptor.size();
  out.write((char*)&dsize, sizeof(dsize));
  vector<int>::iterator it;
  for(it = descriptor.begin() ; it != descriptor.end() ; it++) 
    {
      int d = (*it);
      out.write((char*)&d, sizeof(d));
    }
}

Feature::Feature(ifstream& in)
{
  in.read((char*)&x, sizeof(x));
  in.read((char*)&y, sizeof(y));
  in.read((char*)&scale, sizeof(scale));
  in.read((char*)&orientation, sizeof(orientation));
  int dsize;
  in.read((char*)&dsize, sizeof(dsize));
  //vector<int>::iterator it;
  for(int i = 0 ; i < dsize ; i++ ) 
    {
      int d;
      in.read((char*)&d, sizeof(d));
      descriptor.push_back(d);
    }
}
