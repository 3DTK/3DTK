#include "tsdf/TsdVoxelVDB.h"

void TsdVoxelVDB::update(const float sd, const float maxTruncation) {
    const unsigned char incWeight = 1;
    const unsigned char maxWeight = 32;

    float tsd = std::min(sd / maxTruncation, (float) incWeight);

    _weight += incWeight;

    _weight = std::min(_weight, maxWeight);
    _tsd = (_tsd * (_weight - incWeight) + tsd) / _weight;
}
