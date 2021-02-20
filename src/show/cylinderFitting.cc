#include "show/cylinderFitting.h"

using namespace NEWMAT;

#define POINT_PRESCISION 0.01 //used for sorting points
#define GUI_PRECISION 0.01     //used for checking if cyl value has changed


CylinderFitting cylFit;

/**
 * FitCylinderFromSelection - Constructor
 *
 */
CylinderFitting::CylinderFitting(){
}

/**
 * ~FitCylinderFromSelection - Deconstructor
 *
 */
CylinderFitting::~CylinderFitting(){
}



//
//  User Interface Funtion
//

/**
 * void fitCylinder - Create new Cylinder + fits it to selected Point
 *
 * @param  {type} int cylinderNumber Cylinder number from GUI
 */
void CylinderFitting::fitCylinderFromUserSelection(int cylinderIndex) {
 //delete Cylinder from generated_Cylinders if Cylinder with same Index exits
 deleteCylinderFromSet(cylinderIndex);

 //create new Cylinder
 Cylinder* tmp = createCylinderFromSelectedPoints(cylinderIndex);

 // Insert Cylinder in vector + update GUI values if generation was successfull
 if(tmp != NULL){
   if(tmp->radius > 0){
     //Add Cylinder To List
     generated_Cylinders.insert(tmp);

     //Set GUI Values
     setCurrentCylParas(tmp);
   }
 }
}


/**
 * bool deleteCylnder - Delete Cylinder with index cylinderIndex
 *
 * @param  {type} int cylinderIndex  Cylinder number from GUI
 * @return {type} bool               true if cyl exits, false if cyl doesn't exists
 */
bool CylinderFitting::deleteCylinder(int cylinderIndex){
  for (std::set<Cylinder *>::iterator it = generated_Cylinders.begin(); it != generated_Cylinders.end(); ++it) {
    if((*it)->cylIndex == cylinderIndex){
      delete (*it);
      generated_Cylinders.erase(it);
      setDefaultCurrentCylParas(cylinderIndex);
      return true;
    }
  }
  return false;
}


/**
 * void CylinderFitting - Delete all Cylinder
 *
 */
void CylinderFitting::deleteAllCylinder(){
  //Check if some cylinders are generated --> if empty return
  if(generated_Cylinders.empty()) return;

  for (std::set<Cylinder *>::iterator it = generated_Cylinders.begin(); it != generated_Cylinders.end();++it) {
    delete (*it);
  }
  generated_Cylinders.clear();
}


/**
 * bool setCurrentCylinder - Set current cylinder
 *
 * @param  {type} int cylinderIndex Cylinder index from GUI
 *
 */
bool CylinderFitting::setCurrentCylinder(int cylinderIndex){
    Cylinder *tmp = getCylinder(cylinderIndex);
    if(tmp==NULL){
      setDefaultCurrentCylParas(cylinderIndex);
      return false;
    }
    //Test if new Cylinder was selected
    setCurrentCylParas(tmp);
    return true;
}


/**
 * void changeCylinderParas - Change all neccessary cylinder paras if new user input
 *
 * @param  {type} double radius     Cylinder radius
 * @param  {type} double length     Cylinder length
 * @param  {type} double* axis      Cylinder axis
 * @param  {type} double* cylStartP Cylinder axis start point
 * @param  {type} double* cylEndP   Cylinder axis end point
 */
void CylinderFitting::changeCylinderParas(double radius, double length, double* axis, double* cylStartP, double* cylEndP){
  if(!userSetCylParas){
    std::cout << "Computer set values" << '\n';
      return;
  }

  Cylinder* cyl = getCylinder(current_cylinder);
  if(cyl == NULL) return;

  if(newGUIValue(radius, cyl->radius)){
    std::cout << "Radius Changed "<< radius << '\n';
    cyl->radius = radius;
    setCurrentCylParas(cyl);
    return;
  }

  float currentLength = dist2Points3D(cyl->cylStartP, cyl->cylEndP);
  if(newGUIValue(length, currentLength)){
    std::cout << "Length Changed "<< length << '\n';

    //calc new cyl Start Point
    float cylMiddleP[3];
    cylMiddleP[0] = cyl->cylStartP[0] + cyl->axis[0] * (currentLength / 2);
    cylMiddleP[1] = cyl->cylStartP[1] + cyl->axis[1] * (currentLength / 2);
    cylMiddleP[2] = cyl->cylStartP[2] + cyl->axis[2] * (currentLength / 2);

    //calc new Cyl start + end Point
    cyl->cylStartP[0] = cylMiddleP[0] - cyl->axis[0] * (length / 2);
    cyl->cylStartP[1] = cylMiddleP[1] - cyl->axis[1] * (length / 2);
    cyl->cylStartP[2] = cylMiddleP[2] - cyl->axis[2] * (length / 2);
    cyl->cylEndP[0] = cylMiddleP[0] + cyl->axis[0] * (length / 2);
    cyl->cylEndP[1] = cylMiddleP[1] + cyl->axis[1] * (length / 2);
    cyl->cylEndP[2] = cylMiddleP[2] + cyl->axis[2] * (length / 2);

    //Set current cylinder paras
    setCurrentCylParas(cyl);
    return;
  }

  if(newGUIValue(cylStartP[0], cyl->cylStartP[0]) || newGUIValue(cylStartP[1], cyl->cylStartP[1]) || newGUIValue(cylStartP[2], cyl->cylStartP[2])){
    std::cout << "Start point changed" << '\n';
    cyl->cylStartP[0] = cylStartP[0];
    cyl->cylStartP[1] = cylStartP[1];
    cyl->cylStartP[2] = cylStartP[2];

    //calc new cyl Axis
    float tmpAxis[3];
    tmpAxis[0] = cyl->cylEndP[0] - cyl->cylStartP[0];
    tmpAxis[1] = cyl->cylEndP[1] - cyl->cylStartP[1];
    tmpAxis[2] = cyl->cylEndP[2] - cyl->cylStartP[2];
    float norm = sqrt(tmpAxis[0] * tmpAxis[0] + tmpAxis[1] * tmpAxis[1] + tmpAxis[2] * tmpAxis[2]);
    cyl->axis[0] = tmpAxis[0] / norm;
    cyl->axis[1] = tmpAxis[1] / norm;
    cyl->axis[2] = tmpAxis[2] / norm;
    cyl->axisUser[0] = tmpAxis[0] / norm;
    cyl->axisUser[1] = tmpAxis[1] / norm;
    cyl->axisUser[2] = tmpAxis[2] / norm;
    setCurrentCylParas(cyl);
    return;
  }

  if(newGUIValue(cylEndP[0], cyl->cylEndP[0]) || newGUIValue(cylEndP[1], cyl->cylEndP[1]) || newGUIValue(cylEndP[2], cyl->cylEndP[2])){
    std::cout << "End point changed" << '\n';
    cyl->cylEndP[0] = cylEndP[0];
    cyl->cylEndP[1] = cylEndP[1];
    cyl->cylEndP[2] = cylEndP[2];

    //calc new cyl Axis
    float tmpAxis[3];
    tmpAxis[0] = cyl->cylEndP[0] - cyl->cylStartP[0];
    tmpAxis[1] = cyl->cylEndP[1] - cyl->cylStartP[1];
    tmpAxis[2] = cyl->cylEndP[2] - cyl->cylStartP[2];
    float norm = sqrt(tmpAxis[0] * tmpAxis[0] + tmpAxis[1] * tmpAxis[1] + tmpAxis[2] * tmpAxis[2]);
    cyl->axis[0] = tmpAxis[0] / norm;
    cyl->axis[1] = tmpAxis[1] / norm;
    cyl->axis[2] = tmpAxis[2] / norm;
    cyl->axisUser[0] = tmpAxis[0] / norm;
    cyl->axisUser[1] = tmpAxis[1] / norm;
    cyl->axisUser[2] = tmpAxis[2] / norm;

    setCurrentCylParas(cyl);
    return;
  }
  std::cout << "User change "<< axis[0] << " " << axis[1] << " " << axis[2] << '\n';
  if(newGUIValue(axis[0], cyl->axisUser[0]) || newGUIValue(axis[1], cyl->axisUser[1]) || newGUIValue(axis[2], cyl->axisUser[2])){
    //calc new cyl Middle Point -> Rotate around cyl middle Point
    float cylMiddleP[3];
    cylMiddleP[0] = cyl->cylStartP[0] + cyl->axis[0] * (length / 2);
    cylMiddleP[1] = cyl->cylStartP[1] + cyl->axis[1] * (length / 2);
    cylMiddleP[2] = cyl->cylStartP[2] + cyl->axis[2] * (length / 2);

    //Set User Axis
    cyl->axisUser[0] = axis[0];
    cyl->axisUser[1] = axis[1];
    cyl->axisUser[2] = axis[2];

    //Norm axis for caluclation
    std::cout << "New Axis user: "<< cyl->axisUser[0] << " "<<cyl->axisUser[1] << " "<< cyl->axisUser[2] << " " <<cyl->axis[0] << " " << cyl->axis[1] << " " << cyl->axis[2]<< '\n';
    float normAxis = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    cyl->axis[0] = axis[0] / normAxis;
    cyl->axis[1] = axis[1] / normAxis;
    cyl->axis[2] = axis[2] / normAxis;

    //calc new Cyl start + end Point
    cyl->cylStartP[0] = cylMiddleP[0] - cyl->axis[0] * (length / 2);
    cyl->cylStartP[1] = cylMiddleP[1] - cyl->axis[1] * (length / 2);
    cyl->cylStartP[2] = cylMiddleP[2] - cyl->axis[2] * (length / 2);
    cyl->cylEndP[0] = cylMiddleP[0] + cyl->axis[0] * (length / 2);
    cyl->cylEndP[1] = cylMiddleP[1] + cyl->axis[1] * (length / 2);
    cyl->cylEndP[2] = cylMiddleP[2] + cyl->axis[2] * (length / 2);

    //Set current cylinder paras
    setCurrentCylParas(cyl);
    return;
  }
}


/**
 * bool newGUIValue - Check if new GUI values
 *
 * @param  {type} double valueGUI      GUI value
 * @param  {type} double cylinderValue current cylinder value
 * @return {type}                      true if new value, false if not
 */
bool CylinderFitting::newGUIValue(double valueGUI, double cylinderValue){
  if(fabs(cylinderValue - valueGUI) >= GUI_PRECISION/2) return true;
  return false;
}

/**
 * void setDefaultCurrentCylParas - Set default current GUI cylinder values
 *
 * @param  {type} int cylinderIndex Cylinder Index (GUI)
 */
void CylinderFitting::setDefaultCurrentCylParas(int cylinderIndex){
  current_cylinder = cylinderIndex;
  current_cylRadius = 0;
  current_cylLength = 0;
  current_cylAxis[0] = 0;
  current_cylAxis[1] = 0;
  current_cylAxis[2] = 0;
  current_cylSPoint[0] = 0;
  current_cylSPoint[1] = 0;
  current_cylSPoint[2] = 0;
  current_cylEPoint[0] = 0;
  current_cylEPoint[1] = 0;
  current_cylEPoint[2] = 0;
}


/**
 * void setCurrentCylParas - Set new GUI cylinder paras
 *
 * @param  {type} Cylinder* cyl Values to set current cylinder paras
 */
void CylinderFitting::setCurrentCylParas(Cylinder* cyl){
  current_cylinder = cyl->cylIndex;
  current_cylRadius = cyl->radius;
  current_cylAxis[0] = cyl->axisUser[0];
  current_cylAxis[1] = cyl->axisUser[1];
  current_cylAxis[2] = cyl->axisUser[2];
  current_cylSPoint[0] = cyl->cylStartP[0];
  current_cylSPoint[1] = cyl->cylStartP[1];
  current_cylSPoint[2] = cyl->cylStartP[2];
  current_cylEPoint[0] = cyl->cylEndP[0];
  current_cylEPoint[1] = cyl->cylEndP[1];
  current_cylEPoint[2] = cyl->cylEndP[2];
  current_cylLength = dist2Points3D(cyl->cylStartP, cyl->cylEndP);
}


/**
 * void saveCylinderToFile - Write generated cylinder to file
 *
 */
void CylinderFitting::saveCylinderToFile(){
  // open the output file
  std::ofstream cylinderOut;
  cylinderOut.open(current_cylinderFile, std::ios::out);

  // if file not found then show error
  if(!cylinderOut){
    std::cerr << "Error creating the cylinder file." << std::endl;
    return;
  }
  //Write cylinder in File
  std::cout << "Save Cylinder" << '\n';
  cylinderOut << "#" << "CylinderIndex;Radius;Axis_x Axis_y lAxis_z; StartAxis_x StartAxis_y StartAxis_z;EndAxis_x EndAxis_y EndAxis_z;OrigPointOnAxis_x OrigPointOnAxis_y OrigPointOnAxis_z"<< std::endl;
  for(std::set<Cylinder*>::iterator it = generated_Cylinders.begin(); it!=generated_Cylinders.end(); ++it){
    Cylinder* cyl = (*it);
    cylinderOut << cyl->cylIndex << ";";
    cylinderOut << cyl->radius << ";";
    cylinderOut << cyl->axis[0] << " "<<cyl->axis[1] << " " << cyl->axis[2]<< ";";
    cylinderOut << cyl->cylStartP[0] << " " <<  cyl->cylStartP[1] << " " <<  cyl->cylStartP[2]<< ";";
    cylinderOut << cyl->cylEndP[0] << " " <<  cyl->cylEndP[1] << " " <<  cyl->cylEndP[2]<< ";";
    cylinderOut << cyl->pAxis[0]<< " " << cyl->pAxis[1]<< " " <<cyl->pAxis[2]<< '\n';
  }
  cylinderOut.clear();
  cylinderOut.close();
}


/**
 * void loadCylindersFromFile - Load cylinder from file
 *
 */
void CylinderFitting::loadCylindersFromFile(){
  // input file stream
  std::ifstream cylinderfile;

  // open the output file
  cylinderfile.open(current_cylinderFile, std::ios::in);

  // if file not found then show error
  if(!cylinderfile){
    std::cerr << "Error creating the selection file." << std::endl;
    return;
  }

  string line;
  while(std::getline(cylinderfile,line, '\n')){
    //Test if comment line
    if(line.at(0) == '#') continue;

    //If no comment line --> cylinder line
    std::stringstream ss1(line); string cylPara;
    uint16_t cylinderNumber = 0; std::vector<float> v_cylParas;
    int paraCounter = 0;
    while(std::getline(ss1,cylPara, ';')){
      if(paraCounter == 0){   // Get Cylindernumver
        cylinderNumber = stoi(cylPara);
      }else if(paraCounter == 1){ //Get radius
        v_cylParas.push_back(stof(cylPara));
      }else{ //Get axis, startAxis, endAxis, pOnAxis
        std::stringstream ss2(cylPara); string cylParaPart;
        while(std::getline(ss2,cylParaPart, ' ')){
          v_cylParas.push_back(stof(cylParaPart));
        }
      }
      paraCounter++;
    }
    //Check if correct number of cyl paras
    if(v_cylParas.size() != 13){
      std::cerr << "Wrong cyl Paras number: "<< v_cylParas.size() << '\n';
      continue;
    }
    //Create Cylinder
    Cylinder* cyl = new Cylinder();
    //check if cylinderNumber unused + valid
    if(cylinderNumber < 0) cylinderNumber = 0;
    while(getCylinder(cylinderNumber) != NULL){
      cylinderNumber++;
    }
    cyl->cylIndex = cylinderNumber;
    cyl->radius = v_cylParas.at(0);
    cyl->axis[0] = v_cylParas.at(1);
    cyl->axis[1] = v_cylParas.at(2);
    cyl->axis[2] = v_cylParas.at(3);
    cyl->axisUser[0] = v_cylParas.at(1);
    cyl->axisUser[1] = v_cylParas.at(2);
    cyl->axisUser[2] = v_cylParas.at(3);
    cyl->cylStartP[0] = v_cylParas.at(4);
    cyl->cylStartP[1] = v_cylParas.at(5);
    cyl->cylStartP[2] = v_cylParas.at(6);
    cyl->cylEndP[0] = v_cylParas.at(7);
    cyl->cylEndP[1] = v_cylParas.at(8);
    cyl->cylEndP[2] = v_cylParas.at(9);
    cyl->pAxis[0] = v_cylParas.at(10);
    cyl->pAxis[1] = v_cylParas.at(11);
    cyl->pAxis[2] = v_cylParas.at(12);
    generated_Cylinders.insert(cyl);

    //Set Current Cylinder Values
    setCurrentCylParas(cyl);
  }

  cylinderfile.clear();
  cylinderfile.close();
}


//
// Create Cylinder / Delete Cylinder
//

/**
 * struct Cylinder* createCylinderFromSelectedPoints -
 *  Allocate memory, create cylinder structure, calculate best Fit
 *
 * @param  {type} int cylIndex       Cylinder number from GUI
 * @return {type} Cylinder*          Pointer of new Created cylinder
 */
struct Cylinder* CylinderFitting::createCylinderFromSelectedPoints(int cylIndex){
  struct Cylinder *retVal;

  //Allocate cylinder structure
  retVal = new Cylinder;

  //Test if some points are selected -_> if not can not create Cylinder
  if(selected_points[0].empty()){
    std::cerr << "No selectedPoints" << '\n';
    free(retVal);
    return NULL;
  }

  //Create new set and insert selected Point
  for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--){
    for (std::set<sfloat*>::iterator it = selected_points[iterator].begin();it != selected_points[iterator].end(); it++) {
      CylPoint* p = new CylPoint;
      p->x=(*it)[0];
      p->y=(*it)[1];
      p->z=(*it)[2];
      retVal->selectedPoints.push_back(p);

      CylPoint* p1 = new CylPoint;
      p1->x=(*it)[0];
      p1->y=(*it)[1];
      p1->z=(*it)[2];
      retVal->cylinderPoints.push_back(p1);
    }
  }
  retVal->cylIndex = cylIndex;

  //Fit Cylinder to Selected Points
  fitCylinderToPoints(retVal);

  return retVal;
}


/**
 * bool CylinderFitting - Delete Cylinder with Index == cylIndex + delete generated_Cylinders entry
 *
 * @param  {type} int cylIndex Cylinder Index
 * @return {type} bool         True if delete successfull, false if no cylinder with cylIndex exits in generated_Cylinders
 */
bool CylinderFitting::deleteCylinderFromSet(int cylIndex){
  for (std::set<Cylinder *>::iterator it = generated_Cylinders.begin();it != generated_Cylinders.end(); it++) {
    if((*it)->cylIndex == cylIndex){
      delete (*it);
      generated_Cylinders.erase(it);
      return true;
    }
  }
  return false;
}


/**
 * struct Cylinder* getCylinder - Get Cylinder with right Index back
 *
 * @param  {type} int cylinderIndex  Cylinder index
 * @return {type} Cylinder*          Cylinder* if cylinder index valid, else NULL
 */
struct Cylinder* CylinderFitting::getCylinder(int cylinderIndex){
  for(std::set<Cylinder*>::iterator it = generated_Cylinders.begin(); it!=generated_Cylinders.end(); ++it){
    if((*it)->cylIndex == cylinderIndex){
      return (*it);
    }
  }
  return NULL;
}

//
// Calculate/Fit Cylinder
//

void CylinderFitting::fitCylinderToPoints(Cylinder *c){
  //First Guess Cylinder Fit
  prepareFittingValues(c);
  std::vector<CylPoint> possibleCylAxis = prepareCylAxisForLS(c, cylParasGUI.axisResFirst);
  fitCylinderLeastSquare(c, possibleCylAxis);
  calcCylinderEndsMinMax(c);
  std::cout << "First guess: r: "<< c->radius  <<" axis: ["<< c->axis[0] << ", "<< c->axis[1] << ", " << c->axis[2]<<"]" << '\n';

  //If auto correct allow --> try to optimize first guess
  if(current_cylAutoCorrect){
    fitCylinderAlongLateralSurfacePoints(c);
    calcCylinderEndsMinMax(c);
    std::cout << "TO Implement auto correct" << '\n';
  }else{
    possibleCylAxis.clear();
    possibleCylAxis = prepareCylAxisForLS(c, cylParasGUI.axisResCone, false, cylParasGUI.angleMaxCone);
    fitCylinderLeastSquare(c, possibleCylAxis);
    calcCylinderEndsMinMax(c);
    std::cout << "Second guess: r: "<< c->radius  <<" axis: ["<< c->axis[0] << ", "<< c->axis[1] << ", " << c->axis[2]<<"]" << '\n';
  }
}


bool CylinderFitting::fitCylinderAlongLateralSurfacePoints(Cylinder* c){
  //Variables to check if grow Left/Right possible
  bool growLeft = true, growRight = true;

  //Cylinder Variables
  float radiusThreshold = cylParasGUI.radiusThreshold;

  float current_cylEps = cylParasGUI.cylEps; int currentTry = 0;
  int searches = 0;
  unsigned int meanSearchPoint = 0;

  //Grow Cylinder
  CylPoint pOnAxesLeft = {(c->cylStartP[0] + c->cylEndP[0])/2, (c->cylStartP[1] + c->cylEndP[1])/2, (c->cylStartP[2] + c->cylEndP[2])/2};
  CylPoint pOnAxesRight = {(c->cylStartP[0] + c->cylEndP[0])/2, (c->cylStartP[1] + c->cylEndP[1])/2, (c->cylStartP[2] + c->cylEndP[2])/2};
  while(growLeft || growRight){
    //calc new Search Middle Points on Cylinder Axis
    pOnAxesLeft.x -= (c->axis[0] * (current_cylEps/2));
    pOnAxesLeft.y -= (c->axis[1] * (current_cylEps/2));
    pOnAxesLeft.z -= (c->axis[2] * (current_cylEps/2));
    pOnAxesRight.x += (c->axis[0] * (current_cylEps/2));
    pOnAxesRight.y += (c->axis[1] * (current_cylEps/2));
    pOnAxesRight.z += (c->axis[2] * (current_cylEps/2));
    currentTry++;

    //grow Left
    if(growLeft){
      //search for new Cylinder Points in small cyl section
      double middlePointSearch[3] = {pOnAxesLeft.x, pOnAxesLeft.y, pOnAxesLeft.z};
      double searchDir[3] = {c->axis[0], c->axis[1], c->axis[2]};
      std::vector<float*> pointsCircleSeg = cylinderAxisSearch(middlePointSearch, searchDir, current_cylEps, c->radius + radiusThreshold, c->radius - radiusThreshold); //TODO MEAN POINTS size
      if(pointsCircleSeg.size() * 2 < meanSearchPoint || pointsCircleSeg.size() < 5 ){
        growLeft = false;
      }else{
        meanSearchPoint += pointsCircleSeg.size();
        if(searches != 0) meanSearchPoint /= 2;
        searches++;
      }

      if(growLeft){
        //Calc Best Cylinder
        Cylinder cylPart;
        cylPart.radius = c->radius;
        cylPart.axis[0] = c->axis[0];
        cylPart.axis[1] = c->axis[1];
        cylPart.axis[2] = c->axis[2];
        for(std::vector<float*>::iterator it = pointsCircleSeg.begin(); it != pointsCircleSeg.end(); ++it){
          CylPoint* tmp = new CylPoint((*it)[0], (*it)[1], (*it)[2]);
          cylPart.cylinderPoints.push_back(tmp);
        }
        prepareFittingValues(&cylPart);
        std::vector<CylPoint> possibleCylAxis = prepareCylAxisForLS(&cylPart, 0.25, false, 1);
        fitCylinderLeastSquare(&cylPart, possibleCylAxis);
        float angle = acos(cylPart.axis[0] * c->axis[0] + cylPart.axis[1] * c->axis[1] + cylPart.axis[2] * c->axis[2]);
        float diff = fabs(cylPart.axis[0] - c->axis[0]) + fabs(cylPart.axis[1] - c->axis[1]) + fabs(cylPart.axis[2] - c->axis[2]);
        if(fabs(cylPart.radius - c->radius) < cylParasGUI.radiusThreshold  && (angle < (cylParasGUI.angleMaxCone * M_PI / 180) || diff < 0.25)){
          for(std::vector<float*>::iterator it = pointsCircleSeg.begin(); it != pointsCircleSeg.end(); ++it){
            CylPoint* tmp = new CylPoint((*it)[0], (*it)[1], (*it)[2]);
            c->cylinderPoints.push_back(tmp);
          }
          c->radius += cylPart.radius; c->radius /= 2;
          c->axis[0] += cylPart.axis[0];
          c->axis[1] += cylPart.axis[1];
          c->axis[2] += cylPart.axis[2];
          float norm = sqrt(c->axis[0] * c->axis[0] + c->axis[1] * c->axis[1] + c->axis[2] * c->axis[2]);
          c->axis[0] /= norm;
          c->axis[1] /= norm;
          c->axis[2] /= norm;
          c->cylStartP[0] = pOnAxesLeft.x;
          c->cylStartP[1] = pOnAxesLeft.y;
          c->cylStartP[2] = pOnAxesLeft.z;
          std::cout << "Start Point: "<< c->cylStartP[0] << ", " << c->cylStartP[1] << ", " << c->cylStartP[2] << '\n';

        }else{
          growLeft = false;
        }
      }
    }

    //grow Right
    if(growRight){
      //search for new Cylinder Points in small cyl section
      double middlePointSearch[3] = {pOnAxesRight.x, pOnAxesRight.y, pOnAxesRight.z};
      std::cout << "Middle Point: "<< middlePointSearch[0] << ", " << middlePointSearch[1] << ", " << middlePointSearch[2] << '\n';
      double searchDir[3] = {c->axis[0], c->axis[1], c->axis[2]};
      std::vector<float*> pointsCircleSeg = cylinderAxisSearch(middlePointSearch, searchDir, current_cylEps, c->radius + radiusThreshold, c->radius - radiusThreshold);
      if(pointsCircleSeg.size() * 2 < meanSearchPoint || pointsCircleSeg.size() < 5){
       std::cout << "Not enough Search Points: "<< pointsCircleSeg.size() << "/ "<< meanSearchPoint << '\n';
        growRight = false;
      }else{
        meanSearchPoint += pointsCircleSeg.size();
        if(searches != 0) meanSearchPoint /= 2;
        searches++;
      }
      if(growRight){
        //Calc Best Cylinder
        Cylinder cylPart;
        cylPart.radius = c->radius;
        cylPart.axis[0] = c->axis[0];
        cylPart.axis[1] = c->axis[1];
        cylPart.axis[2] = c->axis[2];
        for(std::vector<float*>::iterator it = pointsCircleSeg.begin(); it != pointsCircleSeg.end(); ++it){
          CylPoint* tmp = new CylPoint((*it)[0], (*it)[1], (*it)[2]);
          cylPart.cylinderPoints.push_back(tmp);
        }
        prepareFittingValues(&cylPart);
        std::vector<CylPoint> possibleCylAxis = prepareCylAxisForLS(&cylPart, 0.25, false, 1);
        fitCylinderLeastSquare(&cylPart, possibleCylAxis);
        float angle = acos(cylPart.axis[0] * c->axis[0] + cylPart.axis[1] * c->axis[1] + cylPart.axis[2] * c->axis[2]);
        float diff = fabs(cylPart.axis[0] - c->axis[0]) + fabs(cylPart.axis[1] - c->axis[1]) + fabs(cylPart.axis[2] - c->axis[2]);
        if(fabs(cylPart.radius - c->radius) < cylParasGUI.radiusThreshold  && (angle < (cylParasGUI.angleMaxCone * M_PI / 180) || diff < 0.25)){
          for(std::vector<float*>::iterator it = pointsCircleSeg.begin(); it != pointsCircleSeg.end(); ++it){
            CylPoint* tmp = new CylPoint((*it)[0], (*it)[1], (*it)[2]);
            c->cylinderPoints.push_back(tmp);
          }
          c->radius += cylPart.radius; c->radius /= 2;
          c->axis[0] += cylPart.axis[0];
          c->axis[1] += cylPart.axis[1];
          c->axis[2] += cylPart.axis[2];
          float norm = sqrt(c->axis[0] * c->axis[0] + c->axis[1] * c->axis[1] + c->axis[2] * c->axis[2]);
          c->axis[0] /= norm;
          c->axis[1] /= norm;
          c->axis[2] /= norm;
          c->cylEndP[0] = pOnAxesRight.x;
          c->cylEndP[1] = pOnAxesRight.y;
          c->cylEndP[2] = pOnAxesRight.z;
        }else{
          growRight = false;
        }
      }
    }
  }
  c->pAxis[0] = (c->cylStartP[0] + c->cylEndP[0])/2;
  c->pAxis[1] = (c->cylStartP[1] + c->cylEndP[1])/2;
  c->pAxis[2] = (c->cylStartP[2] + c->cylEndP[2])/2;
  return true;
}


/**
 * bool prepareFittingValues - Prepare Values used for Least Square Fitting
 *
 * @param  {type} Cylinder *c Cylinder to fit
 * @return {type}             false if nCylinderPoints < 5, else true
 */
bool CylinderFitting::prepareFittingValues(Cylinder *c){
  //calculate Average
  c->para.average[0] = 0;
  c->para.average[1] = 0;
  c->para.average[2] = 0;
  int nCylinderPoints = c->cylinderPoints.size();

  if(nCylinderPoints<5){
    c->radius = -1;
    std::cout << "Cylinder Fitting not Possible. To few selected Points (>5)" << '\n';
    return false;
  }

  //calculate mean
    for (std::vector<CylPoint*>::iterator it=c->cylinderPoints.begin(); it != c->cylinderPoints.end(); ++it){
      c->para.average[0] += (*it)->x;
      c->para.average[1] += (*it)->y;
      c->para.average[2] += (*it)->z;
    }

    for(int i=0; i<3;i++){
      c->para.average[i] /= nCylinderPoints;
    }
    //X = point-mean (for more stable result)
    Real x[nCylinderPoints][3];
    int pointer = 0;
    for (std::vector<CylPoint*>::iterator it=c->cylinderPoints.begin(); it != c->cylinderPoints.end(); ++it){
      x[pointer][0] = (*it)->x -c->para.average[0];
      x[pointer][1] = (*it)->y -c->para.average[1];
      x[pointer][2] = (*it)->z -c->para.average[2];
      pointer++;
    }
    //calculate mu
    // create on Heap no Stack overflow (Cylinder Points can get big)
    Real** products = new Real*[nCylinderPoints];
    for(int i = 0; i < nCylinderPoints; ++i)
      products[i] = new Real[6];
    for(int i=0; i<nCylinderPoints;i++){
      products[i][0]=x[i][0]*x[i][0];
      products[i][1]=x[i][0]*x[i][1];
      products[i][2]=x[i][0]*x[i][2];
      products[i][3]=x[i][1]*x[i][1];
      products[i][4]=x[i][1]*x[i][2];
      products[i][5]=x[i][2]*x[i][2];

      (*c->para.mu)(1)+=products[i][0];
      (*c->para.mu)(2)+=2*products[i][1];
      (*c->para.mu)(3)+=2*products[i][2];
      (*c->para.mu)(4)+=products[i][3];
      (*c->para.mu)(5)+=2*products[i][4];
      (*c->para.mu)(6)+=products[i][5];
    }
    for(int i=1; i<=6;i++){
      (*c->para.mu)(i)/=nCylinderPoints;
    }
    //calculate F0, F1, F2
    (*c->para.F0) = 0.0; (*c->para.F1) = 0.0; (*c->para.F2) = 0.0; //set all elements to zeros
    for(int i=0;i<nCylinderPoints;i++){
      RowVector delta(6);
      delta(1)=products[i][0]-(*c->para.mu)(1);
      delta(2)=2*products[i][1]-(*c->para.mu)(2);
      delta(3)=2*products[i][2]-(*c->para.mu)(3);
      delta(4)=products[i][3]-(*c->para.mu)(4);
      delta(5)=2*products[i][4]-(*c->para.mu)(5);
      delta(6)=products[i][5]-(*c->para.mu)(6);
      (*c->para.F0)(1,1)+=products[i][0];
      (*c->para.F0)(1,2)+=products[i][1];
      (*c->para.F0)(1,3)+=products[i][2];
      (*c->para.F0)(2,2)+=products[i][3];
      (*c->para.F0)(2,3)+=products[i][4];
      (*c->para.F0)(3,3)+=products[i][5];

      ColumnVector xR(3);
      xR << x[i][0]<<x[i][1]<<x[i][2];
      (*c->para.F1)+= xR*delta;
      (*c->para.F2)+= delta.t()*delta;
  }
  (*c->para.F0)/=nCylinderPoints;
  (*c->para.F0)(2,1)=(*c->para.F0)(1,2);
  (*c->para.F0)(3,1)=(*c->para.F0)(1,3);
  (*c->para.F0)(3,2)=(*c->para.F0)(2,3);
  (*c->para.F1)/=nCylinderPoints;
  (*c->para.F2)/=nCylinderPoints;

  //free Heap
  for(int i = 0; i < nCylinderPoints; ++i)
    delete[] products[i];
  delete[] products;
  return true;
}



/**
 * prepareCylAxisForLS - Calculate possible clyinder axis for Least Square
 *
 * @param  {type} Cylinder *c        Current cylinder
 * @param  {type} float resDeg       Res of phi/theta in deg
 * @param  {type} bool firstGuess    First Least Square guess --> yes sample half sphere, no sample cone
 * @param  {type} float coneAngleDeg Max angle around calculated cyl axis to create cone
 * @return {type}                    Possible cylinder axis
 */
std::vector<CylPoint> CylinderFitting::prepareCylAxisForLS(Cylinder *c, float resDeg, bool firstGuess, float coneAngleDeg){
  std::vector<CylPoint> axis;

  if(firstGuess){
    // Deg 2 Rad
    float res = resDeg * M_PI / 180;
    for(float phi = 0.0; phi < M_PI/2.0; phi += res){
      for(float theta = 0.0; theta < 2 * M_PI; theta += res){
        axis.push_back({cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi)});
      }
    }
  }else{
    //calc Rotationmatrix
    ColumnVector W_old(3); W_old << c->axis[0] << c->axis[1] << c->axis[2];
    ColumnVector polAxes(3); polAxes << 0 << 0 << 1;
    ColumnVector rotAxis(3); crossProduct(W_old.Store(), polAxes.Store(), rotAxis.Store());
    Real c = DotProduct(W_old, polAxes);
    Matrix v(3,3); v << 0 << -rotAxis(3) << rotAxis(2) << rotAxis(3) << 0 << -rotAxis(1) << -rotAxis(2) << rotAxis(1) << 0;
    Matrix I(3,3); I << 1 << 0 << 0
                     << 0 << 1 << 0
                     << 0 << 0 << 1;
    Matrix R(3,3);
    R = I + v + v * v * (1/(1+c));

    //Debug
    std::ofstream cylinderOut;
    cylinderOut.open("/home/sm/Documents/Auswertung/debugeCone.3d", std::ios::out);

    // if file not found then show error
    if(!cylinderOut){
      std::cerr << "Error creating the cylinder file." << std::endl;
      return axis;
    }

    //Sampling
    float coneAngle = coneAngleDeg * M_PI / 180;
    float res = resDeg * M_PI / 180;
    if(cylParasGUI.randomized == true){

      RowVector axis_pole(3);
      for(unsigned int i = 0; i < cylParasGUI.nAxis; i++){
        if(i == 0){
          axis_pole(1) = 0;
          axis_pole(2) = 0;
          axis_pole(3) = 1;
        }else{
          float phi = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/coneAngle));
          float theta = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(2*M_PI)));
          axis_pole(1) = cos(theta) * sin(phi);
          axis_pole(2) = sin(theta) * sin(phi);
          axis_pole(3) = cos(phi);
        }
        RowVector axis_rot = axis_pole * R;
        axis.push_back({static_cast<float>(axis_rot(1)), static_cast<float>(axis_rot(2)), static_cast<float>(axis_rot(3))});
        cylinderOut << axis_rot;
      }
    }else{
      for(float  phi = 0; phi <= coneAngle; phi += res){
        for(float theta = 0; theta <= 2 * M_PI; theta += res){
          RowVector axis_pole(3);
          axis_pole(1) = cos(theta) * sin(phi);
          axis_pole(2) = sin(theta) * sin(phi);
          axis_pole(3) = cos(phi);
          RowVector axis_rot = axis_pole * R;
          axis.push_back({static_cast<float>(axis_rot(1)), static_cast<float>(axis_rot(2)), static_cast<float>(axis_rot(3))});
          cylinderOut << axis_rot;
        }
      }
    }
    cylinderOut.clear();
    cylinderOut.close();
  }
  return axis;
}

/**
 * void fitCylinderLeastSquare - Fit Cylinder with Least Square after:
 * Least Squares Fitting of Data by Linear or Quadratic Structures
 * David Eberly, Geometric Tools, Redmond WA 98052
 *
 * @param  {type} Cylinder *c                     Cylinder to fit
 * @param  {type} std::vector<CylPoint>  axis     resPhi  in Deg
 */
void CylinderFitting::fitCylinderLeastSquare(Cylinder *c, std::vector<CylPoint> axis){
  // Define Cylinder Values
  sfloat minError = std::numeric_limits<float>::infinity();
  RowVector W(3); W<<0.0<<0.0<<0.0; //CylinderAxis
  ColumnVector C(3); C<<0.0<<0.0<<0.0;  //Point on Cylinder Axis
  sfloat rSqr = 0;  //r*r


  //solve least square error function
  for(std::vector<CylPoint>::iterator it = axis.begin(); it != axis.end(); ++it){

    RowVector currentW(3); currentW << (*it).x << (*it).y << (*it).z;
    ColumnVector currentC(3); currentC<<0<<0<<0;
    sfloat currentRSqr=0;

    //calculate error
    IdentityMatrix I(3);
    Matrix P(3,3); P = I - currentW.t()*currentW;
    Matrix S(3,3);
    S << 0<< -currentW(3)<< currentW(2)<< currentW(3)<<0<<-currentW(1)<< -currentW(2) << currentW(1) << 0;

    Matrix A(3,3); A = P * (*c->para.F0) * P ;
    Matrix hatA(3,3); hatA= -(S * A * S ) ;
    Matrix hatAA(3,3); hatAA = hatA * A ;
    sfloat trace_hatAA = hatAA.Trace() ;
    Matrix Q(3,3); Q = hatA/trace_hatAA;
    ColumnVector p(6);
    p <<P(1 , 1) << P(1 , 2) << P(1 , 3) << P(2 , 2) << P(2 , 3) << P(3 , 3);

    ColumnVector alpha(3); alpha = (*c->para.F1) * p;
    ColumnVector beta(3); beta = Q * alpha ;

    sfloat error = (DotProduct(p,((*c->para.F2)*p)) - 4 * DotProduct(alpha, beta) + 4 * DotProduct(beta , ((*c->para.F0) * beta)))/ c->cylinderPoints.size() ;
    currentC = beta;
    currentRSqr = DotProduct(p,(*c->para.mu)) + DotProduct(beta , beta) ;

    //if best Solution save best solution
    if(error < minError){
      minError = error;
      W = currentW ;
      C = currentC ;
      rSqr = currentRSqr ;
    }
 }

 C(1)+= c->para.average[0]; C(2)+= c->para.average[1]; C(3)+= c->para.average[2];
 c->axis[0] = W(1); c->axis[1] = W(2); c->axis[2] = W(3);
 c->axisUser[0] = W(1); c->axisUser[1] = W(2); c->axisUser[2] = W(3);
 c->pAxis[0] = C(1); c->pAxis[1] = C(2); c->pAxis[2] = C(3);
 c->radius = sqrt(rSqr);
}


/**
 * std::vector<float*> cylinderAxisSearch - Search from middlePoint in dir direction
 *
 * @param  {type} double middlePoint[3] Search middlePoint
 * @param  {type} double dir[3]         Search direction
 * @param  {type} float length          Search length
 * @param  {type} float radius          Search radius
 * @return {type} std::vector<float*>   Cylinder points found
 */
 std::vector<float*> CylinderFitting::cylinderAxisSearch(double middlePoint[3], double dir[3], float length, float radiusMax, float radiusMin){
  std::vector<float*> searchPoints;
  // Check if cylinder is ready for search
  if(radiusMax <= 0)
    return searchPoints;

  //TODO: Implement multiple search calls for more than one scan
  //cast octpts for search in Show_BOctTree
  Show_BOctTree<sfloat>* bTree = dynamic_cast<Show_BOctTree<sfloat>*>(octpts[0]);
  if(bTree == NULL){
    std::cout << "Dynamic Cast octpts to Show_BOctTree wasn't possible" << '\n';
    return searchPoints;
  }

  double searchPoint[3];
  searchPoint[0] = middlePoint[0];
  searchPoint[1] = middlePoint[1];
  searchPoint[2] = middlePoint[2];
  double searchDir[3];
  searchDir[0] = dir[0];
  searchDir[1] = dir[1];
  searchDir[2] = dir[2];
  std::cout << "Search: Length: "<< length << "Radius: "<< radiusMax << "/" << radiusMin<< " Middle Point:" << searchPoint[0]<< " "<< searchPoint[1]<< " "<< searchPoint[2]<<'\n';
  return bTree->RangeSearchAlongDir(searchPoint, searchDir, radiusMax, radiusMin, length, 0);
}

/**
 * void calcCylinderEndsMinMax - Calc cylinder ends from min Max Point projected on cyl Axis
 *
 * @param  {type} Cylinder *c Cylinder c
 */
void CylinderFitting::calcCylinderEndsMinMax(Cylinder *c){
   //Project Points on Cylinder Axes
   std::vector<CylPoint> projectedPointsOnAxes;
   double roundValue = 1/POINT_PRESCISION;
   for(std::vector<CylPoint*>::iterator it = c->cylinderPoints.begin(); it!=c->cylinderPoints.end(); ++it){
     ColumnVector c_vec(3); c_vec << c->pAxis[0] << c->pAxis[1] << c->pAxis[2];
     ColumnVector cp_vec(3); cp_vec << (*it)->x - c->pAxis[0] << (*it)->y - c->pAxis[1] << (*it)->z - c->pAxis[2];
     ColumnVector w_vec(3); w_vec <<  c->axis[0]  <<  c->axis[1] <<  c->axis[2];

     Real cp_norm = cp_vec.NormFrobenius();
     Real w_norm =  w_vec.NormFrobenius();
     Real d = DotProduct(cp_vec, w_vec); d /= cp_norm; d /= w_norm;

     Real a1 = cp_norm * w_norm * d;
     ColumnVector projected_c(3); projected_c = a1 * w_vec + c_vec;

     CylPoint projected;
     projected.x = round(projected_c(1) * roundValue) / roundValue;
     projected.y = round(projected_c(2) * roundValue) / roundValue;
     projected.z = round(projected_c(3) * roundValue) / roundValue;
     projectedPointsOnAxes.push_back(projected);
   }

   //sort Points on Axis
   if(c->axis[0] >= c->axis[1] && c->axis[0] >= c->axis[2]){
     std::sort(projectedPointsOnAxes.begin(), projectedPointsOnAxes.end());
   }else if(c->axis[1] >= c->axis[2] && c->axis[1] >= c->axis[0]){
     std::sort(projectedPointsOnAxes.begin(), projectedPointsOnAxes.end(), [](const CylPoint c1, const CylPoint c2){
       if (c1.y != c2.y)
           return c1.y < c2.y;
       if (c1.x != c2.x)
           return c1.x < c2.x;
       return c1.z < c2.z;
     });
   }else{
     std::sort(projectedPointsOnAxes.begin(), projectedPointsOnAxes.end(), [](const CylPoint c1, const CylPoint c2){
       if (c1.z != c2.z)
           return c1.z < c2.z;
       if (c1.x != c2.x)
           return c1.x < c2.x;
       return c1.y < c2.y;
     });
   }
   //Get Min bzw Max for cylinder start/end
   c->cylStartP[0] = projectedPointsOnAxes.at(0).x;
   c->cylStartP[1] = projectedPointsOnAxes.at(0).y;
   c->cylStartP[2] = projectedPointsOnAxes.at(0).z;
   c->cylEndP[0] = projectedPointsOnAxes.at((projectedPointsOnAxes.size())-1).x;
   c->cylEndP[1] = projectedPointsOnAxes.at((projectedPointsOnAxes.size())-1).y;
   c->cylEndP[2] = projectedPointsOnAxes.at((projectedPointsOnAxes.size())-1).z;
}

float dist2Points3D(float p1[3], float p2[3]){
  float tmpX = p2[0] - p1[0];
  float tmpY = p2[1] - p1[1];
  float tmpZ = p2[2] - p1[2];
  return sqrt(tmpX * tmpX + tmpY * tmpY + tmpZ * tmpZ);
}


/**
 * void crossProduct - crossProduct of two 3D Vectors a, b
 *
 * @param  {type} Real* a Vector a
 * @param  {type} Real* b Vector b
 * @param  {type} Real* c Result axb
 */
void crossProduct(Real* a, Real* b, Real* c){
   c[0] = a[1] * b[2] - a[2] * b[1];
   c[1] = a[2] * b[0] - a[0] * b[2];
   c[2] = a[0] * b[1] - a[1] * b[0];
}
