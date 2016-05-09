//
// Created by Joschka van der Lucht on 07.05.16.
//

#ifndef CAMERACALIBRATIONTOOL_CHESSBOARD_H
#define CAMERACALIBRATIONTOOL_CHESSBOARD_H


#include <string>
#include <opencv2/core/core.hpp>

namespace Chessboard {

    std::vector<cv::Point3f> readPoint3fChessboardFromFile(std::string path);

    std::vector<cv::Point2f> readPoint2fChessboardFromFile(std::string path);

    std::string toString(std::vector<cv::Point2f> points);

    std::string toString(std::vector<cv::Point3f> points);

};


#endif //CAMERACALIBRATIONTOOL_CHESSBOARD_H
