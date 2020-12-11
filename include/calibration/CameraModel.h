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

    /**
     * similar to OpenCV projectPoints
     * @return
     */
    virtual std::vector<cv::Point3f>forwardprojection();

    virtual std::vector<cv::Point3f>backwardprojection();

    /**
     * Camera parameters to double vector
     * @return
     */
    virtual std::vector<double> parametersToVector();

    virtual void vectorToParameters();

    /**
     * calibrate Camera
     * @return
     */
    virtual void calibrate();

    /**
     *  OpenCV YAML file format
     * @param path
     */
    virtual void saveParametersToFile(std::string &path);
    virtual void loadParametersToFile(std::string &path);

    virtual void wirteParametersTo(cv::FileStorage &fs);
    virtual void fromParametersFrom(cv::FileStorage &fs);



};

} // namespace calibration

#endif // CALIBRATION_CAMERAMODEL_H
