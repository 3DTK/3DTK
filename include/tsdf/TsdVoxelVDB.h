#ifndef TSDVOXELVDB_H
#define TSDVOXELVDB_H

#include <openvdb/openvdb.h>

class TsdVoxelVDB
{
public:
    TsdVoxelVDB() : _tsd(0), _weight(0) {}
    TsdVoxelVDB(float tsd) : _tsd(tsd), _weight(0) {}
    TsdVoxelVDB(float tsd, unsigned char weight) : _tsd(tsd), _weight(weight) {}
    ~TsdVoxelVDB() {}

public:
    TsdVoxelVDB& operator=(const TsdVoxelVDB& voxel) {
        _tsd = voxel._tsd;
        _weight = voxel._weight;
        return *this;
    }

    bool operator==(const TsdVoxelVDB& voxel) const { return _tsd == voxel._tsd; }
    bool operator< (const TsdVoxelVDB& voxel) const { return _tsd <  voxel._tsd; }
    bool operator> (const TsdVoxelVDB& voxel) const { return _tsd >  voxel._tsd; }

    TsdVoxelVDB operator+(const TsdVoxelVDB& voxel) const { return TsdVoxelVDB(_tsd + voxel._tsd, _weight); }
    TsdVoxelVDB operator-(const TsdVoxelVDB& voxel) const { return TsdVoxelVDB(_tsd - voxel._tsd, _weight); }
    TsdVoxelVDB operator-() const { return TsdVoxelVDB(-_tsd, _weight); }

public:
    void update(const float sd, const float maxTruncation);

public:
    float _tsd;
    unsigned char _weight;
};

inline std::ostream& operator<<(std::ostream& ostr, const TsdVoxelVDB& voxel)
{
    ostr << voxel._tsd;
    return ostr;
}

inline TsdVoxelVDB Abs(const TsdVoxelVDB& voxel)
{
    return TsdVoxelVDB(openvdb::math::Abs(voxel._tsd), voxel._weight);
}

#endif
