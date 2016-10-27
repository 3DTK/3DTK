//
// Created by Joschka van der Lucht on 17.10.16.
//

#ifndef LASERCAL_LASERTOOLS_H
#define LASERCAL_LASERTOOLS_H

#import "Settings.h"
#include "PictureHandler.h"
#include "CalibrationPattern.h"
#include <opencv2/core/core.hpp>
#include "AprilTag2f.h"
#include "AprilTag3f.h"

class LaserTools {
private:
    Settings settings;
    PictureHandler pictureHandler;
    CalibrationPattern pattern;
    std::vector<std::vector<std::vector<Point2f>>> imageListWithPoints;
    std::vector<std::vector<std::vector<Point3f>>> patternListWithPoints;
    std::vector<std::vector<Mat>> rvector;
    std::vector<std::vector<Mat>> tvector;


public:
    LaserTools(Settings &s);
    void detectTags(std::vector<std::vector<int>> planeIDs);
    int computeExtrinsic();
    void printMatrix();
    void calculateEquation();
};



#endif //LASERCAL_LASERTOOLS_H
