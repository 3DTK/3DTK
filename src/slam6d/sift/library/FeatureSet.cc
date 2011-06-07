/**
 * @file FeatureSet.cc
 * @brief Implementation of Class FeatureSet
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/FeatureSet.h"
#include <iostream>

using namespace std;

void FeatureSet::serialize(const char* filename)
{
  ofstream out(filename, ios::binary);
  int stringsize = scanid.size();
  out.write((char*) &stringsize, sizeof(stringsize));
  for (int i = 0 ; i < stringsize ; i++) 
    {
      char s = scanid[i];
      out.write((char*)&s, sizeof(s));
    }
  long setsize = features.size();
  out.write((char*) &setsize, sizeof(setsize));
  for (long i = 0 ; i < setsize ; i++) 
    {
      features[i].serialize(out);
    }
  out.close();
}

FeatureSet::FeatureSet(const char* filename)
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
	
  long setsize;
  in.read((char*) &setsize, sizeof(setsize));
  for (long i = 0 ; i < setsize ; i++) 
    {
      Feature feat(in);
      features.push_back(feat);
    }
  in.close();
}
