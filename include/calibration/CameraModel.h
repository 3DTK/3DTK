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
    std::vector<cv::Point3f>forwardprojection();

    std::vector<cv::Point3f>backwardprojection();

    /**
     * Camera parameters to double vector
     * @return
     */
    std::vector<double> parametersToVector();

    void vectorToParameters();

    /**
     * calibrate Camera
     * @return
     */
    void calibrate();

    /**
     *  OpenCV YAML file format
     * @param path
     */
    void saveParametersToFile(std::string &path);
    void loadParametersToFile(std::string &path);

    void wirteParametersTo(cv::FileStorage &fs);
    void fromParametersFrom(cv::FileStorage &fs);



};

} // namespace calibration

#endif // CALIBRATION_CAMERAMODEL_H
