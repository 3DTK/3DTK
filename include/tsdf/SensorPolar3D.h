#ifndef SENSOR_POLAR_3D_H
#define SENSOR_POLAR_3D_H

#include "tsdf/Sensor.h"

class SensorPolar3D : public Sensor
{
public:
  SensorPolar3D(unsigned int beams, double lineAngle, double minLineAngle, unsigned int lines, double frameAngle, double minFrameAngle, double minRange=0.0, double maxRange=INFINITY);
  ~SensorPolar3D();

public:
  void backProject(std::vector<Eigen::Vector4d>& coords, std::vector<int>& indices);

private:
  unsigned int _beams;
  double _lineAngle;
  double _minLineAngle;
  unsigned int _lines;
  double _frameAngle;
  double _minFrameAngle;
  std::vector<std::vector<int> > _indexLUT;
};

#endif
