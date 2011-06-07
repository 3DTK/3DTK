/**
 * @file FeatureMatchSetGroup.cc
 * @brief Implementation of Class FeatureMatchSetGroup
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/FeatureMatchSetGroup.h"
#include <fstream>
#include <iostream>

using namespace std;

FeatureMatchSetGroup::FeatureMatchSetGroup()
{
	
}

FeatureMatchSetGroup::~FeatureMatchSetGroup()
{
	
}

void FeatureMatchSetGroup::serialize(const char * filename)
{
  ofstream out(filename, ios::binary);
  
  int length = matchsets.size();
  out.write((char*)&length, sizeof(length));
  
  list<FeatureMatchSet>::iterator it;
  for (it = matchsets.begin() ; it != matchsets.end() ; it++) 
    {
      it->serialize(out);
    }
  
  out.close();
}

FeatureMatchSetGroup::FeatureMatchSetGroup(const char * filename)
{
  ifstream in(filename, ios::binary);
  if (!in.good()) {cerr << "File not found" << endl; throw 1; }
  
  int length;
  in.read((char*) &length, sizeof(length));
  
  for (int i = 0 ; i < length ; i++) 
    {
      //		cout << i << endl;
      FeatureMatchSet mset(in);
      matchsets.push_back(mset);
    }
  
  in.close();	
}
