#include <fstream>
#include "FeatureMatch.h"

using namespace std;

void FeatureMatch::serialize(ofstream& out)
{
  first.serialize(out);
  second.serialize(out);
  out.write((char*)&dist1, sizeof(dist1));
  out.write((char*)&dist2, sizeof(dist2));
}

FeatureMatch::FeatureMatch(ifstream& in)
{
  first = FeatureBase(in);
  second = FeatureBase(in);
  in.read((char*)&dist1, sizeof(dist1));
  in.read((char*)&dist2, sizeof(dist2));
}
