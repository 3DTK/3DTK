#ifndef __DETECTPLANES_H_
#define __DETECTPLANES_H_

#include "shapes/hough.h"
#include "shapes/shape.h"
#include "shapes/ransac.h"

enum plane_alg {
  RG, RHT, SHT, PHT, PPHT, APHT, RANSAC
};

inline plane_alg formatname_to_plane_alg(const char* string){
  if (strcasecmp(string, "rht") == 0) return RHT;
  else if (strcasecmp(string, "rg") == 0) return RG;
  else if (strcasecmp(string, "sht") == 0) return SHT;
  else if (strcasecmp(string, "pht") == 0) return PHT;
  else if (strcasecmp(string, "ppht") == 0) return PPHT;
  else if (strcasecmp(string, "apht") == 0) return APHT;
  else if (strcasecmp(string, "ran") == 0) return RANSAC;
  else throw std::runtime_error(std::string("Plane Detection Algorithm ") + string + std::string(" is unknown"));
}

class Detect
{
public:
    Detect(Scan* scan, std::string path, bool quiet=true, std::string config="bin/hough.cfg");
    ~Detect();

    void detect(plane_alg algo);

    void writeClusterPoints();
    void writeClusterPlanes();

    Scan* getScanPtr();

private:
    Scan* scan;
    Hough* detector;

    std::string outpath;

    void clusterRansac();
};

#endif // __DETECTPLANES_H_