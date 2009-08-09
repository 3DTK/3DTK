#ifndef __NURBSPATH_H__
#define __NURBSPATH_H__

#include <list>
using std::list;
#include <vector>
using std::vector;
#include "PathGraph.h"
//#include "../map_sim/EdgeList.h"

/**
 *  @class NurbsPath
 *  This class uses Non-Uniform rational B-Splines (NURBS) to smoothen
 *  a given path of nodes.
 *  @author Anton Fl&uuml;gge
 *  @author Hannes Schulz
 */
class NurbsPath{
  public:
    NurbsPath(){}
    vector<PointXY> getNurbsPath(list<PGNode*>&, unsigned int);
    vector<PointXY> getNurbsPath(vector<PointXY>&, unsigned int);
  private:
    float coxDeBoor(float u,int i,int k,const float* Knots);
    void getOutpoint(float t,float OutPoint[],float*opx,float*opy,const float*knots);
    unsigned int ivN;
    unsigned int ivDegree;
    unsigned int ivOrder;
    unsigned int ivNumKnots;
    unsigned int ivNumP;
};

#endif /* __NURBSPATH_H__ */
