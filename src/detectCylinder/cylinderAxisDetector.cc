#include <detectCylinder/cylinderAxisDetector.h>

using namespace NEWMAT;


/**
 * CylinderAxisDetector - Consturctor
 *
 * @param  {type} std::vector<InputPoint>* v_iP InputPoints + Normals (p[3] + n[3] --> pN will be calculated)
 * @param  {type} float sameAngleDeg            Min Angle between 2 cylinder axis
 * @param  {type} unsigned int nPhi             #Phi for creating accumulator
 * @param  {type} unsigned int nTheta           #Theta for creating accumulator
 * @param  {type} unsigned int windowSize       Length sqaured window for moving window filter for maximum detection in accumulator
 * @return {type} CylinderAxisDetector          CylinderAxisDetector
 */
CylinderAxisDetector::CylinderAxisDetector(std::vector<InputPoint>* v_iP, float sameAngleDeg, unsigned int nPhi, unsigned int nTheta, unsigned int windowSize){
  v_inputPoints = v_iP;
  para.sameAngleDeg = sameAngleDeg;
  para.nPhi = nPhi;
  para.nTheta = nTheta;
  para.windowSize = windowSize;
}


/**
 * ~CylinderAxisDetector - Deconstructor
 *
 */
CylinderAxisDetector::~CylinderAxisDetector(){
  //Delet Accumulator
  delete[] nThetaInPhiRow;
  for(unsigned int j = 0; j < para.nPhi; j++) {
    delete[] accumulator[j];
  }
  delete[] accumulator;

  //Delete Detected Cylinder Axis
  for(std::vector<CylinderAxis*>::iterator it = v_cylinderAxis.begin(); it != v_cylinderAxis.end(); ++it){
    delete (*it);
  }
  v_cylinderAxis.clear();
}


/**
 * void calculateCylAxis - Calculate cylinder axis hypotheses with RHT
 *
 */
void CylinderAxisDetector:: calculateCylAxis(){
  std::cout << "Start detecting cylinder axis" << '\n';

  // Prepare Data
  calcSphereNormals();
  std::cout << "Sperical normals calculated" << '\n';

  //Create Hough Space
  createAccumulator();
  std::cout << "Accumulator was created" << '\n';

  //Randomized Hough Transformation
  detectCylinderAxisRHT();
  std::cout << "RHT finished" << '\n';

  //Sort max
  sortPossibleCylinderAxis();
  std::cout << "Maxima found and sorted" << '\n';
}

/**
 * CylinderAxis getCylAxisHypothese - Get cylinder axis hypothese sorted by votes
 *
 * @param  {type} int index Axis Index (Axis are sorted by votes)
 * @return {type}           Returns axis if index is available, if not return cyl axis [0 0 0] with nVotes -1
 */
CylinderAxis CylinderAxisDetector::getCylAxisHypothese(int index){
  if(index < 0 || v_cylinderAxis.empty() || v_cylinderAxis.size() <= static_cast<unsigned int>(index)){
    CylinderAxis ca;
    ca.axis[0] = 0;
    ca.axis[1] = 0;
    ca.axis[2] = 0;
    ca.nVotes = -1;
    return ca;
  }else{
    return (*v_cylinderAxis.at(index));
  }
}

/**
 * calcSphereNormals - Calc spherical normal points from Input-Points + Input-Normals
 *
 */
void CylinderAxisDetector::calcSphereNormals(){
  Real z[3] = {0, 0, 1};
  Matrix b((*v_inputPoints).size(), 3); int bPointer = 0;
  for(std::vector<InputPoint>::iterator it = v_inputPoints->begin(); it != v_inputPoints->end(); ++it){
    //Get Normal in cartesian coor
    float normals_cart[3];
    normals_cart[0] = (*it).n[0];
    normals_cart[1] = (*it).n[1];
    normals_cart[2] = (*it).n[2];

    //Calc normals in unit sphere coordinates
    float normals_sph[3];
    cart2sph(normals_cart, normals_sph);
    // Calc b entry
    float n[3];
    n[0] = cos(normals_sph[0]) * sin(normals_sph[1]);
    n[1] = sin(normals_sph[0]) * sin(normals_sph[1]);
    n[2] = cos(normals_sph[0]);

    b(bPointer+1, 1) = z[0] - n[0];
    b(bPointer+1, 2) = z[1] - n[1];
    b(bPointer+1, 3) = z[2] - n[2];
    bPointer++;
  }

  //Calc Frobenius Norm
  Real bNorm = b.NormFrobenius();

  //Calc Spherical input norms
  std::vector<InputPoint>::iterator it = v_inputPoints->begin();
  for(unsigned int i = 0; i < v_inputPoints->size(); i++){
    ColumnVector normals_cart(3); normals_cart << (*it).n[0] << (*it).n[1] << (*it).n[2];
    //ColumnVector point_cart(3); point_cart << (*it).p[0] << (*it).p[1] << (*it).p[2];

    ColumnVector b_c(3); b_c = (ColumnVector) b.Row(i+1).t();
    if(bNorm!=0)
      b_c /= bNorm;

    Matrix R(3,3); IdentityMatrix I(3);
    R = I-2*b_c*b_c.t();

    ColumnVector circle_vec = R * normals_cart;
    //ColumnVector circle_vec = R * point_cart;
    if(isnan(circle_vec(1)) || isnan(circle_vec(2)) || isnan(circle_vec(3))){
      v_inputPoints->erase(it);
    }else{
      (*it).pN[0] = circle_vec(1);
      (*it).pN[1] = circle_vec(2);
      (*it).pN[2] = circle_vec(3);
      it++;
    }
  }
}

/**
 * createAccumulator - Create half sphere accumulator
 * Original code for plane detection: Dorit /shapes/akkumulator.cc
 *
 */
void CylinderAxisDetector::createAccumulator(){
  nThetaInPhiRow = new unsigned int[para.nPhi];
  int celCounter = 0;

  //Calc max circumference
  double MAX_A = 2 * M_PI;
  double step = M_PI * 0.5 / para.nPhi;
  double r = sin(0.0);
  double z = cos(0.0);
  double r_next = 0.0;
  double z_next = 0.0;
  double c = 0.0; int counter = 0;
  for(double phi = 0; phi < M_PI/2.0; phi += step){
    r_next = sin(phi+step);
    z_next = cos(phi+step);
    //Calc current circumference
    double a = 2 * M_PI * (r + r_next)/2.0;

    //Calc resolution
    c = (( 2* M_PI * (MAX_A / a)) / (para.nTheta - 1));

    // Berechnung der Felder im Akkumulatorarray
    nThetaInPhiRow[counter] = (1.0 + 2*M_PI/c); // Why +1 ? --> always up round, because circle needs to be closed
    r = r_next;
    z = z_next;
    counter++;
  }
  accumulator = new unsigned int*[para.nPhi];
  for(unsigned int j = 0; j < para.nPhi; j++) {
    accumulator[j] = new unsigned int[nThetaInPhiRow[j]];
    for(unsigned int k = 0; k < nThetaInPhiRow[j]; k++) {
      accumulator[j][k] = 0;
      celCounter++;
    }
  }
}


/**
 * detectCylinderAxisRHT - Calc Cylinder axis hypothese with RHT
 *
 */
void CylinderAxisDetector::detectCylinderAxisRHT(){
  srand (time(NULL));

  //Variables calc Plane
  float p1[3]; float p2[3];
  unsigned int rIndex1;
  unsigned int rIndex2;

  //How often try before stop
  unsigned int t = (unsigned int)(v_inputPoints->size()*25);
  unsigned int counter = 0;

  //rht
  while(counter < t){
    //Get random Points
    rIndex1 = (unsigned int) (v_inputPoints->size()*(rand()/(RAND_MAX+1.0)));
    p1[0] = v_inputPoints->at(rIndex1).pN[0];
    p1[1] = v_inputPoints->at(rIndex1).pN[1];
    p1[2] = v_inputPoints->at(rIndex1).pN[2];
    rIndex2 = (unsigned int) (v_inputPoints->size()*(rand()/(RAND_MAX+1.0)));
    p2[0] = v_inputPoints->at(rIndex2).pN[0];
    p2[1] = v_inputPoints->at(rIndex2).pN[1];
    p2[2] = v_inputPoints->at(rIndex2).pN[2];

    if(dist2Point(p1, p2) < 0.0001) continue; //if ~same point continue

    double phi, theta;
    if(calcOriginPlaneThrough2Points(p1, p2, phi, theta)){
      //Only upper half sphere --> Lower Sphere should never happen because calc Orgin right orientated
      if(phi > M_PI/2){
        std::cerr << "calcOriginPlaneThrough2Points wrong orientated" << '\n';
        continue;
      }

      //Calc Index for accumulator
      int phiIndex = (int)(phi*(((double)para.nPhi*0.9999999)/(M_PI / 2)));
      //For safty
      if(phiIndex < 0 || static_cast<unsigned int>(phiIndex) >= para.nPhi){
        std::cerr << "Wrong Phi Index (RHT Cyl Axis)" << '\n';
        continue;
      }
      int thetaIndexMax = nThetaInPhiRow[phiIndex];
      int thetaIndex = (int)(theta*(((double)thetaIndexMax*0.9999999999)/(2*M_PI)));

      //Accumulate hough space cell
      accumulator[phiIndex][thetaIndex] += 1;


      counter++;
    }
  }
}


/**
 * sortPossibleCylinderAxis - Filter possible cyl axis with moving window + sort cyl axis
 *
 */
void CylinderAxisDetector::sortPossibleCylinderAxis() {
  //moving window filter for detecting strong cyl axis hypothese in accumulator
  unsigned int i_phiEnd = para.nPhi - para.windowSize;
  if(para.nPhi < para.windowSize){
    i_phiEnd = 1;
  }
  for(unsigned int i_phi = 0; i_phi < i_phiEnd; i_phi++) {
    unsigned int i_thetaEnd = nThetaInPhiRow[i_phi] - para.windowSize;
    if(nThetaInPhiRow[i_phi] < para.windowSize)
      i_thetaEnd = 1;
    for(unsigned int i_theta = 0; i_theta < i_thetaEnd; i_theta++) {
      unsigned int maxWindow = 0;
      //use Window to find Maxima
      for(unsigned int iw_phi = i_phi; (iw_phi < (i_phi + para.windowSize)) && (iw_phi < para.nPhi); iw_phi++) {
        for(unsigned int iw_theta = i_theta; (iw_theta < (i_theta + para.windowSize)) && (iw_theta < nThetaInPhiRow[iw_phi]); iw_theta++) {
          if(accumulator[iw_phi][iw_theta] > maxWindow){
            maxWindow = accumulator[iw_phi][iw_theta];
          }
        }
      }
      if(maxWindow == 0)
        continue;

      //Put Window Maxima (if > 1) in Function
      for(unsigned int iw_phi = i_phi; (iw_phi < (i_phi + para.windowSize)) && (iw_phi < para.nPhi); iw_phi++) {
        for(unsigned int iw_theta = i_theta; (iw_theta < (i_theta + para.windowSize)) && (iw_theta < nThetaInPhiRow[iw_phi]); iw_theta++) {
          if(accumulator[iw_phi][iw_theta] < maxWindow){
            accumulator[iw_phi][iw_theta] = 0;
          }
        }
      }
    }
  }

  //Get Cyl Axis
  for(unsigned int j = 0; j < para.nPhi; j++) {
    for(unsigned int k = 0; k < nThetaInPhiRow[j]; k++) {
      if(accumulator[j][k] > 0){
        double phi = (0.5 + j) * (double)((M_PI/2) / para.nPhi);
        double theta = (0.5 + k) * (double)(2.0 * M_PI / nThetaInPhiRow[j]);
        CylinderAxis* cA = new CylinderAxis;
        cA->axis[0] = cos(theta) * sin(phi);
        cA->axis[1] = sin(theta) * sin(phi);
        cA->axis[2] = cos(phi);
        cA->nVotes = accumulator[j][k];
        v_cylinderAxis.push_back(cA);
      }
    }
  }

  //Sort cyl axis after votes
  std::sort(v_cylinderAxis.begin(), v_cylinderAxis.end(), [](const CylinderAxis* v1, const CylinderAxis* v2){
      return v1->nVotes > v2->nVotes;
  });


  //Delete multiple cylinder axis (angle between 2 cyl axis <  para.sameAngleDeg)
  for(std::vector<CylinderAxis*>::iterator it1 = v_cylinderAxis.begin(); it1 != v_cylinderAxis.end();){
    bool incIt1 = true;
    for(std::vector<CylinderAxis*>::iterator it2 = it1 + 1 ; it2 != v_cylinderAxis.end();){
      float* v1 = (*it1)->axis;
      float* v2 = (*it2)->axis;
      //calc angle between
      double theta = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
      theta = acos(theta);
      if(theta > M_PI - para.sameAngleDeg*M_PI/180){
        theta = fabs(theta-M_PI);
      }

      if(theta <= para.sameAngleDeg*M_PI/180){
        if((*it1)->nVotes < (*it2)->nVotes){
          delete (*it1);
          v_cylinderAxis.erase(it1);
          incIt1 = false;
          break;
        }else{
          delete (*it2);
          v_cylinderAxis.erase(it2);
          continue;
        }
      }
      ++it2;
    }
    if(incIt1)
      ++it1;
  }
}


/**
 * void cart2sph - Calculate cartesian coordinates in spherical
 *
 * @param  {type} float* cart   Input: cartesian coordinates (3D)
 * @param  {type} float* sphere Output: spherical coordinates
 */
void cart2sph(float* cart, float* sphere){
  sphere[0] = euclideanNorm3D(cart);
  sphere[1] = atan2(cart[1], cart[0]);
  sphere[2] = acos(cart[2] / sphere[0]);
}


/**
 * float euclideanNorm3D - Calculate euclidean norm sqrt(x^2+y^2+z^2)
 *
 * @param  {type} float* v3d Input Vector 3D
 * @return {type} float      euclidean norm
 */
float euclideanNorm3D(float* v3d){
  float tmpX = v3d[0] * v3d[0];
  float tmpY = v3d[1] * v3d[1];
  float tmpZ = v3d[2] * v3d[2];
  return sqrt(tmpX + tmpY + tmpZ);
}


/**
 * float dist2Point - Calc distance between 2 Points 3D
 *
 * @param  {type} float* p1 Input point p1
 * @param  {type} float* p2 Input point p2
 * @return {type} float     distance between p1 and p2
 */
float dist2Point(float* p1, float* p2){
  float tmpX = p1[0] - p2[0];
  float tmpY = p1[1] - p2[1];
  float tmpZ = p1[2] - p2[2];

  return sqrt(tmpX * tmpX + tmpY * tmpY + tmpZ * tmpZ);
}


/**
 * bool calcOriginPlaneThrough2Points - Calc Origin Plane from 2 Points + origin with plane normal pointing to upper half sphere
 *
 * @param  {type} float* p1     Input point p1
 * @param  {type} float* p2     Input point p2
 * @param  {type} double& phi   Output phi plane normal
 * @param  {type} double& theta Output theta plane normal
 */
bool calcOriginPlaneThrough2Points(float* p1, float* p2, double& phi, double& theta){
  float n[3];
  crossProduct(p1, p2, n);
  if(euclideanNorm3D(n) < 0.000001) return false;
  //Check if plane normal is on upper half sphere, if not invert
  if(n[2] < 0){
    n[0] *= -1;
    n[1] *= -1;
    n[2] *= -1;
  }
  double nd[] = {n[0], n[1], n[2]};
  return cartToPolarUnit(nd, phi, theta);
}


/**
 * void crossProduct - Calculate cross product v1 x v2
 *
 * @param  {type} float* v1    Input vector v1 (3D)
 * @param  {type} float* v2    Input vector v2 (3D)
 * @param  {type} float* v1cv2 Output vector v1 x v2
 */
void crossProduct(float* v1, float* v2, float* v1cv2){
  v1cv2[0] = v2[1] * v1[2] - v2[2] * v1[1];
  v1cv2[1] = v2[2] * v1[0] - v2[0] * v1[2];
  v1cv2[2] = v2[0] * v1[1] - v2[1] * v1[0];
}


/**
 * bool cartToPolarUnit - Convert cartesian coordinates in polar on unit sphere (rho = 1)
 *
 * @param  {type} double n[3]   Input cartesian coordinates
 * @param  {type} double &phi   Output phi
 * @param  {type} double &theta Output theta
 */
bool cartToPolarUnit(double n[3], double &phi, double &theta){
  double norm = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
  n[0] /= norm;
  n[1] /= norm;
  n[2] /= norm;

  phi = acos(n[2]);
  double theta0;

  if(fabs(phi) < 0.0001) {
    theta = 0.0;
  } else if(fabs(M_PI - phi) < 0.0001) {
    theta = 0.0;
  } else {
    if(fabs(n[0]/sin(phi)) > 1.0) {
      if(n[0]/sin(phi) < 0) {
        theta0 = M_PI;
      } else {
        theta0 = 0.0;
      }
    } else {
      theta0 = acos(n[0]/sin(phi));
    }

  double sintheta = n[1]/sin(phi);
  double EPS = 0.0001;

    if(fabs(sin(theta0) - sintheta) < EPS) {
      theta = theta0;
    } else if(fabs( sin( 2*M_PI - theta0 ) - sintheta ) < EPS) {
      theta = 2*M_PI - theta0;
    } else {
      theta = 0;
      std::cout << "Error converting cartesian coordinates into polarr" << std::endl;
      return false;
    }
  }
  return true;
}
