//
//  CalibrationPattern.cpp
//  CameraCalibration
//
//  Created by Joschka van der Lucht on 07.09.15.
//  Copyright (c) 2015 Joschka van der Lucht. All rights reserved.
//

#include "calibration/CalibrationPattern.h"

//#include <vtdGen.h>
//#include <vtdNav.h>
//#include <autoPilot.h>
//#include <bookMark.h>
//#include <XMLModifier.h>
//#include <nodeRecorder.h>
//#include <textIter.h>

using namespace std;
using namespace cv;
//using namespace com_ximpleware;


CalibrationPattern::CalibrationPattern() {

}


CalibrationPattern::~CalibrationPattern() {

}


void CalibrationPattern::readPattern(string path, Settings &s) {
    //TODO
    /*
    if(pattern == Settings::APRIL_A4) {
        //Tag: 33mm x 33 mm
        //Rand: 9mm
        // BxH: 7 x 5 Tags
        int id = 0;
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 7; j++) {
                AprilTag3f tag3f = AprilTag3f(id,Point3f(j * (33 + 9), i * (33 + 9), 0), Point3f(33+j*(33+9),i*(33+9),0), Point3f(33+j*(33+9),33+i*(33+9),0), Point3f(j*(33+9),33+i*(33+9),0));
                patternPoints.push_back(tag3f);
                id++;
            }
        }
    } else if(pattern == Settings::APRIL_A3){
        //Tag: 45mm x 45mm
        //Rand: 11mm
        // BxH: 7 x 5 Tags
        int id = 0;
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 7; j++) {
                AprilTag3f tag3f = AprilTag3f(id, Point3f(j * (45 + 11), i * (45 + 11), 0), Point3f(45+j*(45+11),i*(45+11),0), Point3f(45+j*(45+11),45+i*(45+11),0), Point3f(j*(45+11),45+i*(45+11),0));
                patternPoints.push_back(tag3f);
                id++;
            }
        }
    }else if(pattern == Settings::APRIL_3D){
        //initPattern3Dapril();
        //patternPoints = getPattern3Dapril();
        patternPoints = AprilTag::createAprilTag3fFromFile(path);
    }
     */
    patternPoints = AprilTag::createAprilTag3fFromFile(path, s);

}

vector<AprilTag3f> CalibrationPattern::getPoints() {
    return patternPoints;
}


string CalibrationPattern::patternPointsToString() {
    string pointString = "";
    for(AprilTag3f tag3f : patternPoints){
        pointString.append(tag3f.toString());
    }
    return pointString;
}
