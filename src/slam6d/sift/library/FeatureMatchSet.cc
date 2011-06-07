/**
 * @file FeatureMatchSet.cc
 * @brief Implementation of Class FeatureMatchSet
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/FeatureMatchSet.h"
#include <fstream>
#include <iostream>	

using namespace std;

FeatureMatchSet::FeatureMatchSet()
{
	
}

FeatureMatchSet::~FeatureMatchSet()
{
	
}

void FeatureMatchSet::serialize(ofstream &out)
{
  int stringsize = firstscan.size();
  out.write((char*) &stringsize, sizeof(stringsize));
  for (int i = 0 ; i < stringsize ; i++) 
    {
      char s = firstscan[i];
      out.write((char*)&s, sizeof(s));
    }
  stringsize = secondscan.size();
  out.write((char*) &stringsize, sizeof(stringsize));
  for (int i = 0 ; i < stringsize ; i++) 
    {
      char s = secondscan[i];
      out.write((char*)&s, sizeof(s));
    }	
  int msize = matches.size();
  out.write((char*) &msize, sizeof(msize));
  for (int i = 0 ; i < msize ; i++)
    { 
      matches[i].serialize(out);
    }
}

FeatureMatchSet::FeatureMatchSet(ifstream &in)
{
  int stringsize;
  in.read((char*) &stringsize, sizeof(stringsize));
  firstscan = "";
  for (int i = 0 ; i < stringsize ; i++) 
    {
      char s;
      in.read((char*)&s, sizeof(s));
      firstscan += s;
    }
  
  in.read((char*) &stringsize, sizeof(stringsize));
  secondscan = "";
  for (int i = 0 ; i < stringsize ; i++) 
    {
      char s;
      in.read((char*)&s, sizeof(s));
      secondscan += s;
    }
  
  int msize;
  in.read((char*) &msize, sizeof(msize));
  for (int i = 0 ; i < msize ; i++) 
    {
      FeatureMatch match(in);
      matches.push_back(match);
    }
}
