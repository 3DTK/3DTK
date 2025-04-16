#include "tsdf/SensorPolar3D.h"
#include <math.h>
#include <string.h>
#include <iostream>

using namespace std;

SensorPolar3D::SensorPolar3D(unsigned int beams, double lineAngle, double minLineAngle, unsigned int lines, double frameAngle, double minFrameAngle, double minRange, double maxRange) :
    Sensor(minRange, maxRange),
    _beams(beams),
    _lineAngle(lineAngle),
    _minLineAngle(minLineAngle),
    _lines(lines),
    _frameAngle(frameAngle),
    _minFrameAngle(minFrameAngle)
{
    _data.resize(_beams * _lines);
    _mask.resize(_beams * _lines);
    std::fill(_mask.begin(), _mask.end(), true);

    _indexLUT = vector<vector<int> >(_lines, vector<int>(_beams));

    for (unsigned int m = 0; m < _lines; m++) {
        for (unsigned int n = 0; n < _beams; n++) {
            _indexLUT[m][n] = m * _beams + n;
        }
    }
}

SensorPolar3D::~SensorPolar3D()
{
}

void SensorPolar3D::backProject(std::vector<Eigen::Vector4d>& coords, std::vector<int>& indices)
{
    indices.resize(coords.size());
    std::fill(indices.begin(), indices.end(), -1);

    for (unsigned int i = 0; i < coords.size(); i++) {
        Eigen::Vector4d coord = _poseInverse.matrix() * coords[i];

        double phi = atan2(coord(2), coord(0)) - M_PI;
        if(phi > M_PI) phi -= M_PI;
        if(phi < -M_PI) phi += M_PI;

        double theta = acos(coord(1) / coord.head<3>().norm());
        if(coord(2) > 0) theta = -theta;

        unsigned int phiIndex = round((phi - _minFrameAngle) / _frameAngle);
        unsigned int thetaIndex = round((theta - _minLineAngle) / _lineAngle);

        if (phiIndex >= 0 && phiIndex < _lines && thetaIndex >= 0 && thetaIndex < _beams) {
            indices[i] = _indexLUT[phiIndex][thetaIndex];
        }
    }
}
