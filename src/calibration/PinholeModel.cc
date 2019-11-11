#include "calibration/PinholeModel.h"
#include <chrono>
#include <iostream>
#include <boost/filesystem.hpp>

namespace calibration {

PinholeModel::PinholeModel(cv::Mat cameraMatrix, cv::Mat distCoeffs, int width, int height) :
    _cameraMatrix(cameraMatrix),
    _distCoeffs(distCoeffs),
    _width(width),
    _height(height)
{

}

PinholeModel::PinholeModel(const std::string& filename)
{
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);

    fs["Camera_Matrix"] >> _cameraMatrix;
    fs["Distortion_Coefficients"] >> _distCoeffs;
    fs["image_Width"] >> _width;
    fs["image_Height"] >> _height;
}

PinholeModel::~PinholeModel()
{

}

Eigen::Affine3d PinholeModel::computeExtrinsicTransform(std::vector<cv::Point2f> _imagePoints, std::vector<cv::Point3f> _objectPoints)
{
    cv::Mat_<double> rvec, tvec;
    solvePnP(_objectPoints, _imagePoints, _cameraMatrix, _distCoeffs, rvec, tvec, false,
#if CV_MAJOR_VERSION > 2
			cv::SOLVEPNP_ITERATIVE
#else
			CV_ITERATIVE
#endif
			);

    cv:: Mat_<double> rotMatrix;
    Rodrigues(rvec, rotMatrix);

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();

    for (int m = 0; m < 3; m++) {
        for (int n = 0; n < 3; n++) {
            transform(m, n) = rotMatrix.at<double>(m, n);
        }
        transform(m, 3) = tvec.at<double>(m, 0);
    }
    transform(3, 3) = 1;

    return transform;
}

} // namespace calibration
