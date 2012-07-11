/*
 * NurbsPath implementation
 *
 * Copyright (C)  Christof Soeger, Marcel Junker, Anton Fluegge, Hannes Schulz,
                  Andreas Nuechter, Kai Lingemann, Jan Elseberg
 *
 * Released under the GPL version 3.
 *
 */

#include "show/NurbsPath.h"
#include <stdio.h>
#include <iostream>
/**
 * Smoothes a given path, using the Cox-De-Boor algorithm to compute
 * new nodes on the B-Spline.
 * @param origP vector of PointXYs representing the path
 * @param nump  number of points, which should be computed for the smoothed path
 * @return vector of PointXYs representing the smoothed path
 */
vector<double> NurbsPath::camRatio = vector<double>();

vector<PointXY> 
NurbsPath::getNurbsPath(vector<PointXY>& origP,  
                        unsigned int nump, int inter_by_dist){
  list<PGNode*> pgnl;
  list<PGNode*> freepgnl;
  for(unsigned int i=0; i<origP.size();i++){
    PGNode* p = new PGNode();
    p->point.x = origP[i].x;
    p->point.y = origP[i].y;
    pgnl.push_back(p);
    freepgnl.push_back(p);
  }

  vector<PointXY> vec = getNurbsPath(pgnl,nump,inter_by_dist);

  for(list<PGNode*>::iterator it = freepgnl.begin();it!=freepgnl.end();it++) {
    PGNode *p = *it;
    delete p;
  }
  pgnl.clear();
  freepgnl.clear();
  
  return vec;
}

/**
 * Smoothes a given path, using the Cox-De-Boor algorithm to compute
 * new nodes on the B-Spline.
 * @param origP list of PGNode*s representing the path
 * @param nump  number of points, which should be computed for the smoothed path
 * @return vector of PointXYs representing the smoothed path
 */
vector<PointXY> 
NurbsPath::getNurbsPath(list<PGNode*>& origP,  
                        unsigned int nump, int inter_by_dist){
  ivN = origP.size();
  if(ivN<=1){
    vector<PointXY> pv;
    return pv;
  }

  //hack for short paths
  if(ivN==2){
    list<PGNode*>::iterator it = origP.begin();
    PGNode* tmpStart = *it;
    it++;
    origP.push_back(*it);
    origP.push_front(tmpStart);
    ivN=4;
  }
  if(ivN==3){
    list<PGNode*>::iterator it = origP.begin();
    it++;
    origP.insert(it, (*it));
    ivN=4;
  }
  //end short path hack

  float *origPX = new float[ivN];
  float *origPY = new float[ivN];
  list<PGNode*>::iterator it;
  unsigned int i=0;
  for(it=origP.begin();it!=origP.end();it++,i++){
    origPX[i] = (*it)->point.x;
    origPY[i] = (*it)->point.y;
  }

  ivDegree = 3;
  ivOrder  = ivDegree+1;
  ivNumKnots = ivN + ivDegree;
  if(inter_by_dist) {
    ivNumP   = nump;
  } else {
    ivNumP   = camRatio.size();
  }
  float *ivpKnots = new float[ivNumKnots+1];

  for(i=0;i<=ivDegree;i++)
    ivpKnots[i] = 0.0f;

  int numIntKnots = ((ivNumKnots-ivDegree)-(ivOrder));

  for(i=ivOrder; i < ivNumKnots - ivDegree ; i++)
    ivpKnots[i] = ((float)(i-ivDegree))/(numIntKnots+1);

  for(i=ivNumKnots-ivDegree;i<=ivNumKnots;i++)
    ivpKnots[i] = 1.f;

  vector<PointXY> nurbsPath;
  nurbsPath.reserve(ivNumP);
  PointXY outPoint;
  float out[2]={0,0};

  for(i=0;i<=ivNumP; i++){
    out[0] = 0;
    out[1] = 0; 
    if(i!=ivNumP)
      if(inter_by_dist) {
        getOutpoint((float)i/ivNumP,out,origPX,origPY,ivpKnots);
      } else {
        getOutpoint(camRatio[i],out,origPX,origPY,ivpKnots);
      }
    else
      getOutpoint(.999999,out,origPX,origPY,ivpKnots);

    outPoint.x = out[0];
    outPoint.y = out[1];
    nurbsPath.push_back(outPoint);
  }

  delete [] origPX;
  delete [] origPY;
  delete [] ivpKnots;

  return nurbsPath;
}


/**
 * Implementation of the Cox-De-Boor algorithm.
 * The implementation of this algorithm is inspired by the Nurbs Curve Example on
 * http://www.robthebloke.org/opengl_programming.html
 * @param u measure for the length already gone on the smoothed path for the point to be calculated (it should hold that 0 <= u < 1)
 * @param i effect of which original path node on the output path should be computed
 * @param k order (degree of the B-Splines plus 1)
 * @param Knots array of knots needed to compute the B-Spline
 */
float 
NurbsPath::coxDeBoor(float u,int i,int k,const float* Knots) {
	if(k==1)
	{
		if( Knots[i] <= u && u < Knots[i+1] ) {
			return 1.0f;
		}
		return 0.0f;
	}
	float Den1 = Knots[i+k-1] - Knots[i];
	float Den2 = Knots[i+k] - Knots[i+1];
	float Eq1=0,Eq2=0;
	if(Den1>0) {
		Eq1 = ((u-Knots[i]) / Den1) * coxDeBoor(u,i,k-1,Knots);
	}
	if(Den2>0) {
		Eq2 = (Knots[i+k]-u) / Den2 * coxDeBoor(u,i+1,k-1,Knots);
	}
	return Eq1+Eq2;
}

/**
 *  Computes one value on the B-Spline
 *  @param[in] t which point on the spline you want to compute (betwenn 0 for the start point and 1-epsilon for the end point of the smoothed path)
 *  @param[out] OutPoint[] array of two floats, will be overwritten with the x and y coordinate of the point
 */
void 
NurbsPath::getOutpoint(float t,float OutPoint[],float*opx,float*opy,const float*knots) {

	// sum the effect of all CV's on the curve at this point to 
	// get the evaluated curve point
	// 
	for(unsigned int i=0;i!=ivN;++i) {

		// calculate the effect of this point on the curve
		float Val = coxDeBoor(t,i,ivOrder,knots);

		if(Val>0.0001f) {

			// sum effect of CV on this part of the curve
			OutPoint[0] += Val * opx[i];
			OutPoint[1] += Val * opy[i];
		}
	}
}
