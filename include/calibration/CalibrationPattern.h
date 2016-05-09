//
//  CalibrationPattern.h
//  CameraCalibration
//
//  Created by Joschka van der Lucht on 07.09.15.
//  Copyright (c) 2015 Joschka van der Lucht. All rights reserved.
//

#ifndef __CameraCalibration__CalibrationPattern__
#define __CameraCalibration__CalibrationPattern__

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <map>
#include "calibration/Settings.h"
#include "calibration/AprilTag.h"
#include <fstream>



using namespace std;
using namespace cv;
using namespace AprilTag;

class CalibrationPattern{
    
private:
    vector<AprilTag3f> patternPoints;
    
public:
    
    /**
     *  Constructor
     */
    CalibrationPattern();
    
    /**
     *  Destructor
     */
    ~CalibrationPattern();
    
    /**
     *  read an XML-file and pars points
     *
     *  @param path <#path description#>
     */
    void readPattern(string path, Settings &s);
    
    /**
     *  get the Point map
     *
     *  @return map<int, Point3f> with the values <ID, Point>
     */
    vector<AprilTag3f> getPoints();

    string patternPointsToString();
    
};

#endif /* defined(__CameraCalibration__CalibrationPattern__) */
