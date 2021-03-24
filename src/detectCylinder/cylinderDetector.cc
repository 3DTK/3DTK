#include <detectCylinder/cylinderDetector.h>
using namespace NEWMAT;
/**
  * Cylinder Orientation Detection using Hough Transformation
  * Important Refrence papers:
  *
  * EFFICIENT HOUGH TRANSFORM FOR AUTOMATIC DETECTION OF CYLINDERS IN POINT CLOUDS
  * Author of paper: Tahir Rabbani and Frank van den Heuvel
  *
  * The 3D Hough Transform for Plane Detection in Point Clouds: A Review and a new Accumulator Design
  * Author of paper: Dorit Borrmann, Jan Elseberg, Kai Lingemann, Andreas Nuechter
  *
  * ADAPTIVE RANDOMIZED HOUGH TRANSFORM FOR CIRCLE DETECTION USING MOVING WINDOW
  * SI-YU GUO 1 , XU-FANG ZHANG 2 , FAN ZHANG 3
  *
  * A Simple Approach for the Estimation of Circular Arc Center and Its Radius
  * SAMUEL M. THOMAS, Y. T. CHAN
  */


/**
 * CylinderDetector - Constructor
 *
 */
CylinderDetector::CylinderDetector(){
  //load Paras from config File (no file --> use defines )
  ConfigFileCylinderDetector cfg;
  cfg.loadCFG();
  cfg.printConfigParas();


  para.maxCylAxis = cfg.paras.maxCylAxisN;
  para.sameAngleDeg = cfg.paras.minDiffAngleAxisDeg;
  para.nPhi = cfg.paras.nPhi;
  para.nTheta = cfg.paras.nTheta;
  para.windowSize = cfg.paras.windowSize;
  para.minPLateralSurface = cfg.paras.minCylPLateralSurface;
  para.th_maxDisSPoint2Plane = cfg.paras.maxDisP2GreatCircle;

  para.minNumberPointsCircle = cfg.paras.minPointsCircle;
  para.minCostCircle = cfg.paras.minCostCircle;
  para.thresDisCircle2Point = cfg.paras.maxDisPoint2Circle;
  para.valThres = cfg.paras.validationThres;
  para.thresRHT_radius = cfg.paras.rhtThresRadius;
  para.thresRHTXY_m = cfg.paras.rhtThresPosMiddle;
  para.roundValue = cfg.paras.rhtRoundValue;
  para.lengthOfArc = cfg.paras.lsLengthOfArc;
  para.p_nSegment = cfg.paras.lsPositivSectors;

  initCAD_CD = false;
  std::cout << "Cylinder Detector Created" << '\n';
}


/**
 * CylinderDetector - Destructor
 *
 */
CylinderDetector::~CylinderDetector(){
  //Delete Cylinder axis Detector + Circle Detector
  if(initCAD_CD){
    delete cad;
    delete cd;
  }

  //Delete Cylinder Points
  for(std::vector<Cylinder*>::iterator it = v_cylinder.begin(); it != v_cylinder.end(); ++it){
    delete (*it)->cylinderPoints;
    delete (*it);
  }
  v_cylinder.clear();
}


/**
 * void addScanPoints - Add Scan + Normals of Scan to v_inputPoints
 *
 * @param  {type} DataXYZ* points     Points of Scan x
 * @param  {type} DataNormal* normals Normals of Scan x
 */
void CylinderDetector::addScanPoints(DataXYZ* points, DataNormal* normals){
  for(unsigned int i = 0; i < points->size(); i++){
    double p[] = {(*points)[i][0], (*points)[i][1], (*points)[i][2]};
    double n[] = {(*normals)[i][0], (*normals)[i][1], (*normals)[i][2]};

    v_inputPoints.push_back({p, n});
  }
  std::cout << "Scan points added" << '\n';
}



/**
 * std::vector<Cylinder*> detectCylinder - Detect Cylinder (RHT) in all Points added with addScanPoints
 *
 * @return {type}  Detected Cylinder
 */
std::vector<Cylinder*> CylinderDetector::detectCylinder(){
  //check if Input Points available
  if(v_inputPoints.empty()) return v_cylinder;

  //create Circle Detector + CylinderAxisDetector
  cad = new CylinderAxisDetector(&v_inputPoints, para.sameAngleDeg, para.nPhi, para.nTheta, para.windowSize);
  cd = new CircleDetector();
  initCAD_CD = true;

  //set CircleDetector Paras
  cd->para.minNumberPointsCircle = para.minNumberPointsCircle;
  cd->para.minCost = para.minCostCircle;
  cd->para.thresDisCircle2Point = para.thresDisCircle2Point;
  cd->para.valThres = para.valThres;
  cd->para.thresRHT_radius = para.thresRHT_radius;
  cd->para.thresRHTXY_m = para.thresRHTXY_m;
  cd->para.roundValue = para.roundValue;
  cd->para.lengthOfArc = para.lengthOfArc;
  cd->para.p_nSegment = para.p_nSegment;

  //detect Cylinder axis
  cad->calculateCylAxis();


  int cylinderCounter = 0;
  int noCylinderFoundRound = 0;
  for(unsigned int i = 0; i < para.maxCylAxis; i++){
    //Get new cylinder axis hypothese if not valid break
    CylinderAxis ca = cad->getCylAxisHypothese(i);
    if(ca.nVotes == 0)
      break;

    //Calc orthnormal coor sys
    float orthoSysMat[3][3];
    calcOrthnormalCoorSys(&ca, orthoSysMat);

    //find Possible Cylinder Points
    std::vector<origPoint3D> v_PCP = getPossibleCylinderPoints(ca);
    int tryCircle = 0;
    std::vector<Cylinder*> cylFound;
    bool detectCircle = true;
    while(tryCircle < 100 && detectCircle){
      bool possibleCylPUsed[v_PCP.size()];
      for(unsigned int i = 0; i < v_PCP.size(); i++){
        possibleCylPUsed[i] = false;
      }
      detectCircle = false;
      tryCircle++;

      //Calc circle to get position + radius
      std::vector<Circle3D> circles = cd->detectCylinderCircles(v_PCP, orthoSysMat);
      if(circles.empty()){
        noCylinderFoundRound++;
        break;
      }else{
        noCylinderFoundRound = 0;
      }

      //Combine to get Cylinder
      unsigned int newCylinderFound = 0;
      for(unsigned int i = 0; i < circles.size(); i++){
        float c[] = {circles.at(i).middleP[0], circles.at(i).middleP[1], circles.at(i).middleP[2]};
        Cylinder* cyl = new Cylinder();
        cyl->cylinderNumber = cylinderCounter;
        cyl->cylinderPoints = new std::vector<origPoint3D>();
        for(std::vector<origPoint3D>::iterator it = circles.at(i).v_origPoints.begin(); it != circles.at(i).v_origPoints.end(); ++it){
          cyl->cylinderPoints->push_back((*it));
        }
        cyl->axis[0] = ca.axis[0];
        cyl->axis[1] = ca.axis[1];
        cyl->axis[2] = ca.axis[2];
        cyl->pAxis[0] = c[0];
        cyl->pAxis[1] = c[1];
        cyl->pAxis[2] = c[2];
        cyl->radius =  circles.at(i).r;
        cyl->inlierPoints = circles.at(i).inlierPoints;

        if(findCylinderEndsNew(cyl)){
          //Test if Circle correct
          if(tryCircle == 1){
            cylFound.push_back(cyl);
            detectCircle = true;
          }else{
            bool foundNewCyl = false;
            for(std::vector<Cylinder*>::iterator it = cylFound.begin(); it != cylFound.end(); ++it){
              if(intersectCylinder((*(*it)), (*cyl))){
                foundNewCyl = true;
              }
            }
            if(foundNewCyl){
              cylFound.push_back(cyl);
              detectCircle = true;
            }
          }
          v_cylinder.push_back(cyl);
          for(std::vector<origPoint3D>::iterator it = cyl->cylinderPoints->begin(); it != cyl->cylinderPoints->end(); ++it){
            possibleCylPUsed[(*it).indexVPossibleCircle] = true;
          }
          cylinderCounter++;
          newCylinderFound++;
        }else{
          delete cyl;
          continue;
        }
      }
      for(unsigned int m = 0; m < v_PCP.size(); m++){
        if(possibleCylPUsed[m]){
          v_PCP.erase(v_PCP.begin()+m);
        }
      }
    }

  }
  deleteCylinderIntersectingAxes(true);
  return v_cylinder;
}

/**
 * void calcOrthnormalCoorSys - Calculate orthnormales coordinatesystem, where ca is one axis
 *
 * @param  {type} CylinderAxis* ca        Cylinder axis
 * @param  {type} float orthoSysMat[3][3] orthnormal coorsys, with {[0][0], [1][0], [2][0]} = ca
 * @return {type}                         description
 */
void CylinderDetector::calcOrthnormalCoorSys(CylinderAxis* ca, float orthoSysMat[3][3]){
  float u[3];
  if(ca->axis[1] != 0){
    u[0] = 0;
    u[1] = -ca->axis[2];
    u[2] = ca->axis[1];
  }else{
    u[0] = -ca->axis[2];
    u[1] = 0;
    u[2] = ca->axis[0];
  }

  float w[3];
  crossProduct(ca->axis, u, w);

  float normU = euclideanNorm3D(u);
  u[0] /= normU; u[1] /= normU; u[2] /= normU;
  float normW = euclideanNorm3D(w);
  w[0] /= normW; w[1] /= normW; w[2] /= normW;

  orthoSysMat[0][0] = ca->axis[0]; orthoSysMat[1][0] = ca->axis[1]; orthoSysMat[2][0] = ca->axis[2];
  orthoSysMat[0][1] = u[0]; orthoSysMat[1][1] = u[1]; orthoSysMat[2][1] = u[2];
  orthoSysMat[0][2] = w[0]; orthoSysMat[1][2] = w[1]; orthoSysMat[2][2] = w[2];
}

/**
*  vector<origPoint3D> getPossibleCylinderPoints - Get all Points voting for cylinder axis ca
*
* @param  {type} CylinderAxis ca         Cylinder axis
* @return {type} vector<origPoint3D>     All points voting for cylinder axis ca
*/
std::vector<origPoint3D> CylinderDetector::getPossibleCylinderPoints(CylinderAxis ca){
 std::vector<origPoint3D> v_PCP; unsigned int indexC = 0;
 for(std::vector<InputPoint>::iterator it = v_inputPoints.begin(); it != v_inputPoints.end(); it++){
   double d = abs((*it).pN[0] *  ca.axis[0] + (*it).pN[1]  * ca.axis[1] + (*it).pN[2]  * ca.axis[2]);
   if(d < para.th_maxDisSPoint2Plane){
     origPoint3D o;
     o.point[0] = (*it).p[0];
     o.point[1] = (*it).p[1];
     o.point[2] = (*it).p[2];
     o.indexOrig = indexC;
     v_PCP.push_back(o);
   }
   indexC++;
 }
 return v_PCP;
}


/**
 * bool CylinderDetector - Calc Cylinder Ends from mean dis between later surface neighbours projected on cyl axis
 *
 * @param  {type} Cylinder *c Calc cylidner ends of c
 */
bool CylinderDetector::findCylinderEndsNew(Cylinder *c){
  std::vector<Point> projectedPointsOnAxes;
  if(c->cylinderPoints->empty()){
    return false;
  }

  //Project Cylinder Points on Axes
  for(std::vector<origPoint3D>::iterator it = c->cylinderPoints->begin(); it!=c->cylinderPoints->end();){
    ColumnVector c_vec(3); c_vec << c->pAxis[0] << c->pAxis[1] << c->pAxis[2];
    ColumnVector cp_vec(3); cp_vec << (*it).point[0]-c->pAxis[0] << (*it).point[1]-c->pAxis[1] << (*it).point[2]-c->pAxis[2];
    ColumnVector w_vec(3); w_vec <<  c->axis[0]  <<  c->axis[1] <<  c->axis[2];
    Real cp_norm = cp_vec.NormFrobenius();
    Real w_norm =  w_vec.NormFrobenius();

    //Calc Distance to acis and delete if to small or big
    ColumnVector crossD(3); crossD = 0;
    crossProduct(cp_vec.Store(), w_vec.Store(), crossD.Store());
    double dist = crossD.NormFrobenius() / w_norm;
    if((dist < c->radius - 1) || (dist > c->radius + 1)){
      if(dist < c->radius - 1)
        c->inlierPoints++;
      c->cylinderPoints->erase(it);
    continue;
    }

    //Calc projected Point on cyl axis
    Real d = DotProduct(cp_vec, w_vec);
    d /= cp_norm; d /= w_norm;
    Real a1 = cp_norm * w_norm * d;
    ColumnVector projected_c(3);
    projected_c = a1 * w_vec + c_vec;

    //Round so sort is possible after x,y,z without calculation error
    Point projected;
    projected.x = round(projected_c(1) * SORT_ROUND_VALUE) / SORT_ROUND_VALUE;
    projected.y = round(projected_c(2) * SORT_ROUND_VALUE) / SORT_ROUND_VALUE;
    projected.z = round(projected_c(3) * SORT_ROUND_VALUE) / SORT_ROUND_VALUE;
    projectedPointsOnAxes.push_back(projected);
    ++it;
  }

  //Check if to much inlierPoints --> highly likely false Detection
  if(c->cylinderPoints->size() * 10.0  < c->inlierPoints){
    return false;
  }

  //Sort points along axis, so between first and last element is biggest distance along axis
  std::sort(projectedPointsOnAxes.begin(), projectedPointsOnAxes.end(), [](const Point p1, const Point p2){
    if(p1.x != p2.x)
      return p1.x > p2.x;
    if(p1.y != p2.y)
      return p1.y > p2.y;
    return p1.z > p2.z;
  });

  //Calc mean Distance between points
  double meanDistNeighbours = 0; int divide = 0;
  double maxDistance = 0;
  double dis[projectedPointsOnAxes.size()-1];
  for(unsigned int i = 0; i < projectedPointsOnAxes.size()-1; i++){
    float p1[3] = {projectedPointsOnAxes.at(i).x, projectedPointsOnAxes.at(i).y, projectedPointsOnAxes.at(i).z};
    float p2[3] = {projectedPointsOnAxes.at(i+1).x, projectedPointsOnAxes.at(i+1).y, projectedPointsOnAxes.at(i+1).z};
    dis[i] = dist2Point(p1, p2);
    if(dis[i] > maxDistance)
      maxDistance = dis[i];
    meanDistNeighbours += dis[i]; divide++;
  }
  meanDistNeighbours /= divide;
  // if(meanDistNeighbours > maxMeanPointsDistance){
  //   meanDistNeighbours = maxMeanPointsDistance;
  // }

  //Get biggst Set S, where maxDistance between 2 neighbours is < x*meanDistNeighbours
  int cylStart = 0; int cylEnd = projectedPointsOnAxes.size()-1; int cylNPoints = 0;
  int counter = 0; int start = 0;
  if(maxDistance > F_2_PROJ_CYLPOINTS * meanDistNeighbours){
    for(unsigned int i = 0; i < projectedPointsOnAxes.size()-2; i++){
      counter ++;
      if(dis[i] > F_2_PROJ_CYLPOINTS * meanDistNeighbours){

        if(counter > cylNPoints){
          cylStart = start;
          cylEnd = cylStart + counter - 1;
          cylNPoints = counter;
        }
        start = i+1;
        counter = 0;
      }
    }
    if(counter > cylNPoints){
      cylStart = start;
      cylEnd = cylStart + counter - 1;
      cylNPoints = counter;
    }
  }

  if(projectedPointsOnAxes.empty())
    return false;

  // Take cyl with most points
  c->cylinderAxisEnd[0] = projectedPointsOnAxes.at(cylEnd).x;
  c->cylinderAxisEnd[1] = projectedPointsOnAxes.at(cylEnd).y;
  c->cylinderAxisEnd[2] = projectedPointsOnAxes.at(cylEnd).z;
  c->cylinderAxisStart[0] = projectedPointsOnAxes.at(cylStart).x;
  c->cylinderAxisStart[1] = projectedPointsOnAxes.at(cylStart).y;
  c->cylinderAxisStart[2] = projectedPointsOnAxes.at(cylStart).z;

  return true;
}


/**
 * void deleteCylinderIntersectingAxes - Delete multiple cyl detection (or bad intersecting Cyl if deleteBadCylinder == true)
 *
 * @param  {type} bool deleteBadCylinder Delete not only same, but also bad cyl if cylinders are intersecting
 */
void CylinderDetector::deleteCylinderIntersectingAxes(bool deleteBadCylinder){
  //sort Cylinder after CylPoints
  std::sort(v_cylinder.begin(), v_cylinder.end(), [](const Cylinder* c1, const Cylinder * c2){
    return  c1->cylinderPoints->size() > c2->cylinderPoints->size();
  });

  //Check if 2 cyl are same (or 2 cyls are intersecting + 1 is bad)
  for(std::vector<Cylinder*>::iterator it1 = v_cylinder.begin(); it1 != v_cylinder.end();){
    bool iterIt1 = true;
    for(std::vector<Cylinder*>::iterator it2 = it1 + 1; it2 != v_cylinder.end(); ){
      bool iterIt2 = true;
      //test if one Cylinder bad
      bool cyl1Bad = false, cyl2Bad = false;
      if((*it1)->inlierPoints > (*it1)->cylinderPoints->size())
        cyl1Bad = true;
      if((*it2)->inlierPoints > (*it2)->cylinderPoints->size())
        cyl2Bad = true;

      //2 cyls are same || (2 intersecting + min one is bad)
      if(sameCylinder((*(*it1)), (*(*it2))) || (deleteBadCylinder && (cyl1Bad || cyl2Bad) && intersectCylinder((*(*it1)), (*(*it2))))){
        //Check if 1 cyl is better
        bool deleteCylinder1 = true;
        float cost1 = (*it1)->cylinderPoints->size();
        float cost2 = (*it2)->cylinderPoints->size();
        if(cost1 == cost2){
          if((*it1)->radius > (*it2)->radius){
            deleteCylinder1 = true;
          }else{
            deleteCylinder1 = false;
          }
        }else{
          if(cost1 > cost2){
            deleteCylinder1 = false;
          }else{
            deleteCylinder1 = true;
          }
        }

        //Delete worse cyl
        if(deleteCylinder1){
          v_cylinder.erase(it1);
          iterIt1 = false;
          break;
        }else{
          v_cylinder.erase(it2);
          iterIt2 = false;
          continue;
        }
      }
      if(iterIt2)
        ++it2;
    }
    if(iterIt1)
     ++it1;
  }
}


/**
 * bool sameCylinder - Check if 2 Cyl are same (angle between axis < para.sameAngleDeg && intersecting)
 *
 * @param  {type} Cylinder &cyl1 Input Cyl 1
 * @param  {type} Cylinder& cyl2 Input Cyl 2
 * @return {type} bool           true, if same, false if not same
 */
bool CylinderDetector::sameCylinder(Cylinder &cyl1, Cylinder& cyl2){
  //calc angle between 2 cyl axis
  ColumnVector w1(3); w1 << cyl1.axis[0] << cyl1.axis[1] << cyl1.axis[2];
  ColumnVector w2(3); w2 << cyl2.axis[0] << cyl2.axis[1] << cyl2.axis[2];
  float cosAngle = DotProduct(w1, w2)/(w1.NormFrobenius()*w2.NormFrobenius());
  if(cosAngle > 1){
    cosAngle = 1;
  }

  Real angle = acos(cosAngle);
  angle = angle * 180 /M_PI;
  //test if axis same
  Real normDiff = (w1-w2).NormFrobenius();
  //if(angle < para.sameAngleDeg || abs(angle-180) < para.sameAngleDeg){
  if(angle < 5 || abs(angle-180) < 5 || normDiff < 0.001){
    return intersectCylinder(cyl1, cyl2);
  }
  return false;
}



/**
 * bool intersectCylinder - Check if cyl1 and cyl2 intersect (min dis between cylAxis < radius1+radius2)
 *
 * @param  {type} Cylinder &cyl1 Input Cylinder 1
 * @param  {type} Cylinder& cyl2 Input Cylinder 2
 * @return {type}                true intersection; false no intersection
 */
bool CylinderDetector::intersectCylinder(Cylinder &cyl1, Cylinder& cyl2){
  ColumnVector c1_p0(3); c1_p0 << cyl1.cylinderAxisStart[0] << cyl1.cylinderAxisStart[1] << cyl1.cylinderAxisStart[2];
  ColumnVector c1_p1(3); c1_p1 << cyl1.cylinderAxisEnd[0] << cyl1.cylinderAxisEnd[1] << cyl1.cylinderAxisEnd[2];
  ColumnVector c1(3); c1 = c1_p1 - c1_p0;
  ColumnVector c2_p0(3); c2_p0 << cyl2.cylinderAxisStart[0] << cyl2.cylinderAxisStart[1] << cyl2.cylinderAxisStart[2];
  ColumnVector c2_p1(3); c2_p1 << cyl2.cylinderAxisEnd[0] << cyl2.cylinderAxisEnd[1] << cyl2.cylinderAxisEnd[2];
  ColumnVector c2(3); c2 = c2_p1 - c2_p0;
  Real normC1 = c1.NormFrobenius();
  Real normC2 = c2.NormFrobenius();
  c1 /= normC1;
  c2 /= normC2;
  ColumnVector c1Cc2(3);
  crossProduct(c1.Store(), c2.Store(), c1Cc2.Store());
  Real crossNorm2 = c1Cc2.NormFrobenius(); crossNorm2 *= crossNorm2;
  Real minDis = 0;

  // Cylc angle between axis
  ColumnVector w1(3); w1 << cyl1.axis[0] << cyl1.axis[1] << cyl1.axis[2];
  ColumnVector w2(3); w2 << cyl2.axis[0] << cyl2.axis[1] << cyl2.axis[2];
  float cosAngle = DotProduct(w1, w2)/(w1.NormFrobenius()*w2.NormFrobenius());
  if(cosAngle > 1){
    cosAngle = 1.0;
  }
  Real angle = acos(cosAngle);
  angle = angle * 180.0/M_PI;

  //Min distance calculation between 2 cyl axis, if axis are ~parallel
  if(crossNorm2 < 0.0001 || angle < 0.1 || abs(angle-180) < 0.1){
    ColumnVector tmp(3); tmp = c2_p0 - c1_p0;
    Real d0 = DotProduct(c1, tmp);
    tmp = c2_p1 - c1_p0;
    minDis = (((d0*c1) + c1_p0) - c2_p0).NormFrobenius();
    if(minDis < cyl1.radius + cyl2.radius){
      return true;
    }
    return false;
  }

  //Axis arent parallel (calc min distance between 2 skew lines
  ColumnVector a0b0(3); a0b0 = c2_p0 - c1_p0;
  Matrix mC1(3,3);
  mC1.Column(1) = a0b0;
  mC1.Column(2) = c2;
  mC1.Column(3) = c1Cc2;
  Matrix mC2(3,3);
  mC2.Column(1) = a0b0;
  mC2.Column(2) = c1;
  mC2.Column(3) = c1Cc2;
  Real detC1 = mC1.Determinant();
  Real detC2 = mC2.Determinant();
  Real t0 = detC1/crossNorm2;
  Real t1 = detC2/crossNorm2;
  ColumnVector pC1(3); pC1 = c1_p0 + (c1 * t0);
  ColumnVector pC2(3); pC2 = c2_p0 + (c2 * t1);

  //If min distance out of cylinder bounds --> clip to nearest cyl end
  if(t0 < 0)
    pC1 = c1_p0;
  if(t0 > normC1)
    pC1 = c1_p1;

  if(t1 < 0)
    pC2 = c2_p0;
  if(t1 > normC2)
    pC2 = c2_p1;

  if(t0 < 0 || t0 > normC1){
    Real dot0 = DotProduct(c2, (pC1 - c2_p0));
    if(dot0 < 0){dot0 = 0;}
    else if(dot0 > normC2){dot0 = normC2;}
    pC2 = c2_p0 + (c2 * dot0);
  }
  if(t1 < 0 || t1 > normC2){
    Real dot1 = DotProduct(c1, (pC2 - c1_p0));
    if(dot1 < 0){dot1 = 0;}
    else if(dot1 > normC1){dot1 = normC1;}
    pC1 = c1_p0 + (c1 * dot1);
  }

  //Calc min distance
  minDis = (pC1 - pC2).NormFrobenius();
  if(minDis < cyl1.radius + cyl2.radius){
    return true;
  }
  return false;
}

/**
* void crossProduct - crossProduct
*
* @param  {type} Real* a Input Vector A (3D)
* @param  {type} Real* b Input Vector B (3D)
* @param  {type} Real* c Output Vector C (3D)
*/
void crossProduct(Real* a, Real* b, Real* c){
 c[0] = a[1] * b[2] - a[2] * b[1];
 c[1] = a[2] * b[0] - a[0] * b[2];
 c[2] = a[0] * b[1] - a[1] * b[0];
}

double dist2Point(double* p1, double* p2){
  double tmpX = p1[0] - p2[0];
  double tmpY = p1[1] - p2[1];
  double tmpZ = p1[2] - p2[2];

  return sqrt(tmpX * tmpX + tmpY * tmpY + tmpZ * tmpZ);
}
