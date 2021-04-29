#include <detectCylinder/circleDetector.h>
using namespace NEWMAT;


/**
 * CircleDetector - Constructor
 *
 */
CircleDetector::CircleDetector(){

}


/**
 * CircleDetector - Deconstructor
 *
 */
CircleDetector::~CircleDetector(){

}

/**
 * std::vector<Circle3D> detectCylinderCircles - Detect cylinder projected along cylinder axis
 *
 * @param  {type} const std::vector<origPoint3D> &v_inputPoints 3D Possible cylinder points
 * @param  {type} float orthoSysMat[3][3]                       orthnormal coordinate system, cyl axis is column 1 of matrix
 * @return {type} std::vector<Circle3D>                         Detected projected Circles + with middle point in 3D
 */
std::vector<Circle3D> CircleDetector::detectCylinderCircles(const std::vector<origPoint3D> &v_inputPoints, float orthoSysMat[3][3]){
  //clear v_validatedCircles from last search
  if(!v_projectedPoints.empty()){
    v_projectedPoints.clear();
  }
  if(!v_validatedCircles.empty()){
    for(std::vector<Circle3D>::iterator it = v_validatedCircles.begin(); it != v_validatedCircles.end(); ++it){
      (*it).v_origPoints.clear();
    }
    v_validatedCircles.clear();
  }

  //copy input Points
  v_origPoints = v_inputPoints;

  //Test if enough new Points
  if(v_origPoints.empty() || v_origPoints.size() < para.minNumberPointsCircle)
    return v_validatedCircles;

  //Project Points on Multiple Planes along cylinder axis
  projectPointsAlongCylAxis(orthoSysMat);
  //Find all possible circles + validate them
  rhtNew();

  //Test if circles are found if not return empty vector
  if(v_validatedCircles.empty())
    return v_validatedCircles;

  //Project 2D Circle middle point back --> 3D point on cyl axis
  for(unsigned int i = 0; i < v_validatedCircles.size(); i++) {
     float middleBack[3];
     backProjection(v_validatedCircles.at(i).middleP, orthoSysMat, middleBack);
     v_validatedCircles.at(i).middleP[0] = middleBack[0];
     v_validatedCircles.at(i).middleP[1] = middleBack[1];
     v_validatedCircles.at(i).middleP[2] = middleBack[2];
   }

  return v_validatedCircles;
}


/**
 * void projectPointsAlongCylAxis - Project 3D points along cyl axis on plane
 *
 * @param  {type} float orthoSysMat[3][3] orthnormal coordinate system
 */
void CircleDetector::projectPointsAlongCylAxis(float orthoSysMat[3][3]){
  // Create Vectors for projecting
  ColumnVector a(3); a << orthoSysMat[0][2] << orthoSysMat[1][2] << orthoSysMat[2][2];
  ColumnVector b(3); b << orthoSysMat[0][1] << orthoSysMat[1][1] << orthoSysMat[2][1];
  ColumnVector n(3); n << orthoSysMat[0][0] << orthoSysMat[1][0] << orthoSysMat[2][0];

  //Init Index counter + tmp Projection variables
  int index3D = 0;

  //Project Points on origin plane + calc signed distance to seperate Points for different planes later
  for(std::vector<origPoint3D>::const_iterator it = v_origPoints.begin(); it != v_origPoints.end(); ++it){
    ColumnVector p(3); p << (*it).point[0] << (*it).point[1] << (*it).point[2];
    ColumnVector p_proj = DotProduct(p, a) * a + DotProduct(p, b) * b;
    ProjPoint projP = {static_cast<float>(DotProduct(p_proj,a)), static_cast<float>(DotProduct(p_proj,b)), 0 ,index3D}; index3D++;
    v_projectedPoints.push_back(projP);
  }
}



/**
 * void rhtNew - Detects Circle with RHT
 *
 */
void CircleDetector::rhtNew(){
  srand (time(NULL));
  std::vector<CircleCandidate> circlesFound; int c_counter = 0;
  const unsigned int nProjPoints = v_projectedPoints.size();

  //Array for
  bool pointsUsed[nProjPoints];
  unsigned int nC = 0;  //How many points are already used for all circles

  unsigned int t = 0;
  while(t < nProjPoints){
    t++;

    //check if still enough points for one circle detection
    if(v_projectedPoints.size() < para.minNumberPointsCircle)
      break;

    //Select 3 Random points
    unsigned int rIndex = v_projectedPoints.size()*(rand()/(RAND_MAX+1.0));
    ProjPoint p1 = v_projectedPoints.at(rIndex);
    rIndex = v_projectedPoints.size()*(rand()/(RAND_MAX+1.0));
    ProjPoint p2 = v_projectedPoints.at(rIndex);
    rIndex = v_projectedPoints.size()*(rand()/(RAND_MAX+1.0));
    ProjPoint p3 = v_projectedPoints.at(rIndex);

    //Test if points are collinear --> can't lie on one circle
    if(testCollinear2D(p1, p2, p3))
      continue;

    //Calc possible circle
    CircleCandidate c = calculateCircleThrough3Points(p1, p2, p3);
    c.middle.x = round(c.middle.x * para.roundValue) / para.roundValue;
    c.middle.y = round(c.middle.y * para.roundValue) / para.roundValue;
    c.r = round(c.r * para.roundValue) / para.roundValue;
    c.nPoints = 0;
    c.nCircleFound = 0;
    c.cost = 0;
    c.wasValidated = false;

    //Add circle candidate in accumulator if rht start
    if(c_counter == 0){
      circlesFound.push_back(c);
      c_counter++;
      continue;
    }

    //Look if circle was already calculated
    bool found = false;
    for(unsigned int i = 0; i < circlesFound.size(); i++){
      float c_x = circlesFound.at(i).middle.x;
      float c_y = circlesFound.at(i).middle.y;
      float c_r = circlesFound.at(i).r;
      if(c_x > c.middle.x - para.thresRHTXY_m && c_x < c.middle.x + para.thresRHTXY_m
      && c_y > c.middle.y - para.thresRHTXY_m && c_y < c.middle.y + para.thresRHTXY_m
      && c_r > c.r - para.thresRHT_radius && c_r < c.r + para.thresRHT_radius){
        //Circle entry was found
        circlesFound.at(i).nCircleFound++;
        found = true;

        //Check if already was validated --> break
        if(circlesFound.at(i).wasValidated == true){
          break;
        }

        //Wasn't validated + Circle was
        if(circlesFound.at(i).nCircleFound == para.valThres){
          circlesFound.at(i).wasValidated = true;

          //calc circle cost check if enough points + cost high enough
          calcCircleCost(&circlesFound.at(i), para.thresRHT_radius, pointsUsed, nC);
          if(circlesFound.at(i).nPoints < para.minNumberPointsCircle || circlesFound.at(i).cost < para.minCost || circlesFound.at(i).inlierPoints > circlesFound.at(i).nPoints ){
            circlesFound.erase(circlesFound.begin() + i);
            break;
          }

          //if validationg wrong delete circle
          if(!validateAndCorrectCircleNew(circlesFound.at(i))){
            circlesFound.erase(circlesFound.begin() + i);
            break;
          }

          //Validation was true --> Create Circle
          Circle3D val;
          val.r = circlesFound.at(i).r;
          val.middleP[0] = circlesFound.at(i).middle.x;
          val.middleP[1] = circlesFound.at(i).middle.y;
          val.middleP[2] = 0;
          val.nVotingPoints = circlesFound.at(i).nPoints;
          val.cost = circlesFound.at(i).cost;
          val.inlierPoints = 0;
          for(std::vector<ProjPoint>::iterator it = v_projectedPoints.begin(); it != v_projectedPoints.end();) {
            //Calc distance
            double tmpX = circlesFound.at(i).middle.x - (*it).x;
            double tmpY = circlesFound.at(i).middle.y - (*it).y;
            double dis = sqrt(tmpX * tmpX + tmpY * tmpY) - circlesFound.at(i).r;
            if( dis < -para.thresDisCircle2Point){
              val.inlierPoints++;
            }
            if(fabs(dis) < para.thresDisCircle2Point){
              v_origPoints.at((*it).index3D).indexVPossibleCircle = (*it).index3D;
              val.v_origPoints.push_back(v_origPoints.at((*it).index3D));
              pointsUsed[(*it).index2D] = true;
              v_projectedPoints.erase(it);
            }else{
              if(pointsUsed[(*it).index2D])
                nC--;
              pointsUsed[(*it).index2D] = false;
              ++it;
            }
          }

          //save Circle in validate circle + delete from candidate Circle
          v_validatedCircles.push_back(val);
          circlesFound.erase(circlesFound.begin()+i);

          //Test if circle Intersect if yes delete circle
          for(std::vector<CircleCandidate>::iterator it = circlesFound.begin(); it != circlesFound.end();){
            //calc distance between
            float tmpX = (*it).middle.x - val.middleP[0];
            float tmpY = (*it).middle.y - val.middleP[1];
            float dis = sqrt(tmpX * tmpX + tmpY * tmpY);

            if(dis < (*it).r + val.r){
              circlesFound.erase(it);
            }else{
              ++it;
            }
          }
      }
      }
    }
    if(found == false){
      circlesFound.push_back(c);
      c_counter++;
    }
  }
}


/**
 * void backProjection - Projects circle middle point back in 3D coor sys
 *
 * @param  {type} float p_projected[2]    Input: Porjected circle middle point
 * @param  {type} float orthoSysMat[3][3] Input: orthonormal coor sys, with column 1 = cyl axis
 * @param  {type} float back[3]           Output: 3D Circle middle point --> Point on cyl axis
 */
void CircleDetector::backProjection(float p_projected[2], float orthoSysMat[3][3], float back[3]){
  ColumnVector a(3); a << orthoSysMat[0][2] << orthoSysMat[1][2] << orthoSysMat[2][2];
  ColumnVector b(3); b << orthoSysMat[0][1] << orthoSysMat[1][1] << orthoSysMat[2][1];
  ColumnVector n(3); n << orthoSysMat[0][0] << orthoSysMat[1][0] << orthoSysMat[2][0];

  Matrix S(3,3);
  S.Column(1) = a; S.Column(2) = b; S.Column(3) = n;

  ColumnVector p_proj(3);
  p_proj << p_projected[0] << p_projected[1] << 0;
  ColumnVector p_back = (ColumnVector)(S*p_proj);

  back[0] = p_back(1);
  back[1] = p_back(2);
  back[2] = p_back(3);

  return;
}

/**
 * void calcCircleCost - Calculate Circle cost for CircleCandidate c
 *
 * @param  {type} CircleCandidate *c          Circle costs need to be calculated
 * @param  {type} float threshold             Max allowed distance between points and circle border
 * @param  {type} bool *pointsUsed            Points used for circle detection
 * @param  {type} unsigned int &nc            #Points used for circle detection
 */
void CircleDetector::calcCircleCost(CircleCandidate *c, float threshold, bool *pointsUsed, unsigned int &nc){
  unsigned int nPoints = 0;
  unsigned int nPointsInCircle = 0;
  for(unsigned int i =0; i < v_projectedPoints.size(); i++){
    //Calc distance
    double tmpX = c->middle.x - v_projectedPoints.at(i).x;
    double tmpY = c->middle.y - v_projectedPoints.at(i).y;
    double dis = sqrt(tmpX * tmpX + tmpY * tmpY);
    if(dis < c->r - threshold){
      nPointsInCircle++;
    }
    if(dis > c->r - threshold && dis < c->r + threshold){
      nPoints++;
      if(!pointsUsed[v_projectedPoints.at(i).index2D]){
         pointsUsed[v_projectedPoints.at(i).index2D] = true;
         nc++;
       }
    }
  }

  float cost = nPoints / c->r;
  c->cost = cost;
  c->nPoints = nPoints;
  c->inlierPoints = nPointsInCircle;
}


/**
 * bool validateAndCorrectCircleNew - Validate Circle + Use Least Square to correct
 *
 * @param  {type} CircleCandidate& c Circle to validate
 * @param  {type} double thresholdR  Max distance between circle point and circle line
 * @return {type} bool               true if validation success, flase if not
 */
bool CircleDetector::validateAndCorrectCircleNew(CircleCandidate& c, double thresholdR){
  // Find new possible Points + Calc Mean
  std::vector<ProjPoint> possibleCirclePoints;
  Real meanX = 0; Real meanY = 0;
  for (unsigned int i = 0; i < v_projectedPoints.size(); i++) {
    //Calc distance
    double tmpX = c.middle.x - v_projectedPoints.at(i).x;
    double tmpY = c.middle.y - v_projectedPoints.at(i).y;
    double dis = sqrt(tmpX * tmpX + tmpY * tmpY);

    //If distance point to circle line smaller than threshold --> add to possibleCirclePoints
    if(dis > c.r - thresholdR && dis < c.r + thresholdR){
      possibleCirclePoints.push_back(v_projectedPoints.at(i));
      meanX += v_projectedPoints.at(i).x;
      meanY += v_projectedPoints.at(i).y;
    }
  }
  meanX /= possibleCirclePoints.size();
  meanY /= possibleCirclePoints.size();

  //Divde circle in sectors with even arc of a circle
  double deltaTheta = para.lengthOfArc/c.r;
  unsigned int nSegment = ceil((2 * M_PI * c.r)/para.lengthOfArc);
  if(c.r < 1)
    nSegment = ceil((2 * M_PI)/para.lengthOfArc);

  //check if enough circle points to get a positive validated circle
  if(possibleCirclePoints.size() < para.p_nSegment * nSegment){
    return false;
  }

  // Test how many positive circle sectors are avialable
  bool circleSeg[nSegment];
  for(unsigned int i = 0; i < nSegment; i++){
    circleSeg[i] = false;
  }
  unsigned int segmentsValid = 0;
  for (unsigned int i = 0; i < possibleCirclePoints.size(); i++) {
    //Calc Theta to get circle seg
    double tmpX = possibleCirclePoints.at(i).x - c.middle.x;
    double tmpY = c.middle.y - possibleCirclePoints.at(i).y;
    double theta = atan2(tmpX, tmpY);
    if(tmpX < 0)
     theta += 2*M_PI;
    int index = round(theta/deltaTheta);
    if(index > nSegment)
      index = nSegment;
    if(!circleSeg[index]){
      circleSeg[index] = true;
      segmentsValid++;
    }
  }
  //If not enough sectors are positive return false
  if(segmentsValid <  para.p_nSegment * nSegment){
    return false;
  }

  // Least Square
  float middlePoint[2];
  float radius;
  leastSquareCircle(possibleCirclePoints,meanX, meanY, middlePoint, &radius);
  c.middle.x = middlePoint[0];
  c.middle.y = middlePoint[1];
  c.r = radius;
  return true;
}


/**
 * void leastSquareCircle - Caluclate best circle fit in circlePoints
 *  After: A Simple Approach for the Estimation of Circular Arc Center and Its Radius; SAMUEL M. THOMAS, Y. T. CHAN
 *
 * @param  {type} std::vector<ProjPoint> circlePoints Points to fit circle
 * @param  {type} float meanX                         Mean X Value of circlePoints
 * @param  {type} float meanY                         Mean Y Value of circlePoints
 * @param  {type} float *middlePoint                  Output: float[2] = {middleX, middleY}
 * @param  {type} float* radius                       Output: float radius
 */
void CircleDetector::leastSquareCircle(std::vector<ProjPoint> circlePoints, float meanX, float meanY, float *middlePoint, float* radius){
  Real x = 0;
  Real xx = 0;
  Real xxx = 0;

  Real y = 0;
  Real yy = 0;
  Real yyy = 0;

  Real xy = 0;
  Real xxy = 0;
  Real xyy = 0;

  for(std::vector<ProjPoint>::iterator it = circlePoints.begin(); it != circlePoints.end(); it++){
    //-Mean Middle Point so Circle is around 0
    float tmpX = (*it).x - meanX;
    float tmpY = (*it).y - meanY;

    x += tmpX;
    xx += (tmpX * tmpX);
    xxx += (tmpX * tmpX * tmpX);

    y += tmpY;
    yy += (tmpY * tmpY);
    yyy += (tmpY * tmpY * tmpY);

    xy += (tmpX * tmpY);
    xxy += (tmpX * tmpX * tmpY);
    xyy += (tmpX * tmpY * tmpY);
  }
  unsigned int n = circlePoints.size();
  Real a1 = 2 * (x * x - n * xx);
  Real a2 = 2 * (x * y - n * xy);
  Real b1 = a2;
  Real b2 = 2 * (y * y - n * yy);
  Real c1 = xx * x - n * xxx + x * yy - n * xyy;
  Real c2 = xx * y - n * yyy + y * yy - n * xxy;

  Real x_m = (c1*b2 - c2*b1)/(a1*b2-a2*b1);
  Real y_m = (a1*c2 - a2*c1)/(a1*b2-a2*b1);
  Real rr = (xx - 2 * x * x_m + n * x_m * x_m + yy - 2 * y * y_m +n * y_m * y_m) / n;
  middlePoint[0] = x_m + meanX;
  middlePoint[1] = y_m + meanY;
  (*radius) = sqrt(rr);
}


/**
 * bool testCollinear2D - Test if 3 Points (2D) are collinear
 *
 * @param  {type} ProjPoint a Input point 1 (2D)
 * @param  {type} ProjPoint b Input point 2 (2D)
 * @param  {type} ProjPoint c Input point 3 (2D)
 * @return {type}             true, if collinear, false if not
 */
bool testCollinear2D(ProjPoint a, ProjPoint b, ProjPoint c){
  // Area of triangle formula
  float axbx = a.x - b.x;
  float ayby = a.y - b.y;
  float bxcx = b.x - c.x;
  float bycy = b.y - c.y;

  float det = axbx * bycy - bxcx * ayby;

  if(det == 0)
    return true;
  return false;
}


/**
 * CircleCandidate calculateCircleThrough3Points - Calculate Circle through 3 points
 *
 * @param  {type} ProjPoint a       Input circle point 1
 * @param  {type} ProjPoint b       Input circle point 2
 * @param  {type} ProjPoint c       Input circle point 3
 * @return {type} CircleCandidate   Output circle
 */
CircleCandidate calculateCircleThrough3Points(ProjPoint a, ProjPoint b, ProjPoint c){
  // Basic Sbtractions
  float axbx = a.x - b.x;
  float ayby = a.y - b.y;
  float axcx = a.x - c.x;
  float aycy = a.y - c.y;
  float cxax = c.x - a.x;
  float cyay = c.y - a.y;
  float bxax = b.x - a.x;
  float byay = b.y - a.y;

  //  Quad subtractions
  float qaxcx = a.x * a.x - c.x * c.x;
  float qaycy = a.y * a.y - c.y * c.y;
  float qbxax = b.x * b.x - a.x * a.x;
  float qbyay = b.y * b.y - a.y * a.y;

  // Calc Circle middle Point
  CircleCandidate circ;
  circ.middle.x = (-qaxcx * ayby - qaycy * ayby - qbxax * aycy - qbyay * aycy);
  circ.middle.x /= (2 * (cxax * ayby - bxax * aycy));
  circ.middle.y = (-qaxcx * axbx - qaycy * axbx - qbxax * axcx - qbyay * axcx);
  circ.middle.y /= (2 * (cyay * axbx - byay * axcx));

  // Calculate radius
  float trans = -a.x * a.x - a.y * a.y + 2 * circ.middle.x * a.x + 2 * circ.middle.y * a.y;
  circ.r = sqrt(circ.middle.x * circ.middle.x + circ.middle.y * circ.middle.y - trans);

  // set default values
  circ.nPoints = 0;
  circ.cost = 0;

  return circ;
}
