/**
 * @file FeatureBase.cc
 * @brief Implementation of Class FeatureBase
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include <fstream>
#include "slam6d/sift/library/FeatureBase.h"

using namespace std;

void FeatureBase::serialize(ofstream& out)
{
  out.write((char*)&x, sizeof(x));
  out.write((char*)&y, sizeof(y));
  out.write((char*)&scale, sizeof(scale));
  out.write((char*)&orientation, sizeof(orientation));
}

FeatureBase::FeatureBase(ifstream& in)
{
  in.read((char*)&x, sizeof(x));
  in.read((char*)&y, sizeof(y));
  in.read((char*)&scale, sizeof(scale));
  in.read((char*)&orientation, sizeof(orientation));
}
