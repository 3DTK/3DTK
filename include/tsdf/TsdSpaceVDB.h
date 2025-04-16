#ifndef TSDSPACEVDB_H
#define TSDSPACEVDB_H

#include "tsdf/Sensor.h"
#include "TsdVoxelVDB.h"
#include <openvdb/openvdb.h>

class TsdSpaceVDB
{
public:
    TsdSpaceVDB(const double voxelSize);
    virtual ~TsdSpaceVDB();

public:
    void setMaxTruncation(const float val) { _maxTruncation = val; }
    double getMaxTruncation() const { return _maxTruncation; }

    void integrate(Sensor* sensor);

private:
    double _voxelSize;
    float _maxTruncation;
    openvdb::Grid<openvdb::tree::Tree4<TsdVoxelVDB, 5, 4, 3>::Type>::Ptr _grid;
};

#endif
