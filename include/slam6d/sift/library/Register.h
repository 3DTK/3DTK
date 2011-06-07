/**
 * @file Register.h
 * @brief Register two and more scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __REGISTER_H__
#define __REGISTER_H__

#include "FeatureMatchSetGroup.h"
#include "PanoramaMap.h"
#include "ScanTransform.h"
#include <list>
#include <map>

enum MODE 
{
  ALL,
  PERCENTAGE,
  MAXIMUM
};

class Register
{
 public:
  Register (FeatureMatchSetGroup *group, std::vector<PanoramaMap*> &maps);
  virtual ~Register () {}
  
  void registerScans();
  
  bool registerSet(FeatureMatchSet *set, PanoramaMap *map1, PanoramaMap *map2, double trans[][4]);
  inline void processTrianglePair(FeatureMatchSet *set, PanoramaMap *map1, PanoramaMap *map2,
				  int p1, int p2, int p3,
				  double &best_error, int &best_ind, int best_pt[3], double best_align[16], Coord &tr1, Coord &tr2,
				  vector<Coord> &coordsA,
				  vector<bool> &hasCoordsA,
				  vector<Coord> &coordsB,
				  vector<bool> &hasCoordsB
				  );

 public:
  
  MODE mode;
  double mode_percentage;
  int mode_maximum;
  
  int d;
  double t;
  
  double dinfluence;
  
  double mind;
  double mina;
  
  FeatureMatchSetGroup *group;
  std::map<std::string, PanoramaMap *> maps;
};
#endif /* __REGISTER_H__ */
