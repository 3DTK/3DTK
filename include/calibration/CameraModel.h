#ifndef CALIBRATION_CAMERAMODEL_H
#define CALIBRATION_CAMERAMODEL_H

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace calibration
{

class CameraModel
{
public:
    virtual ~CameraModel() {};

public:
    virtual Eigen::Affine3d computeExtrinsicTransform(std::vector<cv::Point2f> _imagePoints, std::vector<cv::Point3f> _objectPoints) = 0;
};

} // namespace calibration

#endif // CALIBRATION_CAMERAMODEL_H
