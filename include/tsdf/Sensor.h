#ifndef SENSOR_H_
#define SENSOR_H_

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Sensor
{
public:
    Sensor(double minRange, double maxRange);
    virtual ~Sensor();

public:
    void setPose(const Eigen::Affine3d& pose) { _pose = pose; _poseInverse = pose.inverse(); }
    const Eigen::Affine3d& getPose() { return _pose; }
    Eigen::Vector3d getTranslation() { return _pose.translation(); }
    Eigen::Matrix3d getRotation() { return _pose.rotation(); }

    double getMinRange() { return _minRange; }
    double getMaxRange() { return _maxRange; }
    unsigned int getSize() { return _data.size(); }

    void setData(const std::vector<double>& data) { _data = data; }
    const std::vector<double> getData() { return _data; }

    void setMask(const std::vector<bool>& mask) { _mask = mask; }
    const std::vector<bool>& getMask() { return _mask; }

    virtual void backProject(std::vector<Eigen::Vector4d>& coords, std::vector<int>& indices) = 0;

protected:
    Eigen::Affine3d _pose;
    Eigen::Affine3d _poseInverse;
    double _minRange;
    double _maxRange;
    std::vector<double> _data;
    std::vector<bool> _mask;
};

#endif
