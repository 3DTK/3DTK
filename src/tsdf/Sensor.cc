#ifndef SENSOR_H
#define SENSOR_H

#include "tsdf/Sensor.h"

Sensor::Sensor(double minRange, double maxRange) :
    _pose(Eigen::Affine3d::Identity()),
    _minRange(minRange),
    _maxRange(maxRange)
{
}

Sensor::~Sensor()
{
}

#endif
