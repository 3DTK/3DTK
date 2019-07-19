#ifndef CALIBRATION_PINHOLEMODEL_H
#define CALIBRATION_PINHOLEMODEL_H

#include "calibration/CameraModel.h"

namespace calibration
{

class PinholeModel : public CameraModel
{
public:
    PinholeModel(cv::Mat cameraMatrix, cv::Mat distCoeffs, int width, int height);
    PinholeModel(const std::string& filename);
    ~PinholeModel();

public:
    Eigen::Affine3d computeExtrinsicTransform(std::vector<cv::Point2f> _imagePoints, std::vector<cv::Point3f> _objectPoints);

private:
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    int _width;
    int _height;
};

} // namespace calibration

#endif // CALIBRATION_PINHOLEMODEL_H
