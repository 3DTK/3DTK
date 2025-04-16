#include "tsdf/TsdSpaceVDB.h"
#include <cstring>
#include <cmath>
#include <omp.h>
#include <openvdb/tools/LevelSetFilter.h>

using namespace std;

TsdSpaceVDB::TsdSpaceVDB(const double voxelSize) :
    _voxelSize(voxelSize)
{
    _maxTruncation = 3 * voxelSize;

    _grid = openvdb::Grid<openvdb::tree::Tree4<TsdVoxelVDB, 5, 4, 3>::Type>::create(TsdVoxelVDB(_maxTruncation, 0));
    _grid->setTransform(openvdb::math::Transform::createLinearTransform(voxelSize));
    _grid->setGridClass(openvdb::GRID_LEVEL_SET);
}

TsdSpaceVDB::~TsdSpaceVDB(void)
{
}

void TsdSpaceVDB::integrate(Sensor* sensor)
{
    Eigen::Vector3d sensorTranslation = sensor->getTranslation();

    cout << "Sensor position: " << sensorTranslation(0) << " " << sensorTranslation(1) << " " << sensorTranslation(2) << endl;

    double limits[3][2];
    for (int i = 0; i < 3; i++) {
        limits[i][0] = int((sensorTranslation[i] - sensor->getMaxRange()) / _voxelSize / 8) * 8;
        limits[i][1] = int(1 + (sensorTranslation[i] + sensor->getMaxRange()) / _voxelSize / 8) * 8;
    }

    openvdb::Grid<openvdb::tree::Tree4<TsdVoxelVDB, 5, 4, 3>::Type>::Accessor gridAccessor = _grid->getAccessor();

    for (int x = limits[0][0]; x <= limits[0][1]; x+=8)
        for (int y = limits[1][0]; y <= limits[1][1]; y+=8)
            for (int z = limits[2][0]; z <= limits[2][1]; z+=8) {
                vector<Eigen::Vector4d> coords(512);
                vector<openvdb::Coord> xyz(512);

                int a = 0;
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 8; j++)
                        for (int k = 0; k < 8; k++) {
                            coords[a][0] = (x + i + 0.5) * _voxelSize;
                            coords[a][1] = (y + j + 0.5) * _voxelSize;
                            coords[a][2] = (z + k + 0.5) * _voxelSize;
                            coords[a][3] = 1;

                            xyz[a] = openvdb::Coord(x + i, y + j, z + k);
                            a++;
                        }

                vector<int> idx(512);
                sensor->backProject(coords, idx);

                for (int b = 0; b < 512; b++) {
                    int index = idx[b];

                    if (index >= 0 && sensor->getMask()[index] && sensor->getData()[index] > 1.0) {
                        float distance = (sensorTranslation - coords[b].head<3>()).norm();
                        float sd = sensor->getData()[index] - distance;

                        if (sd >= -_maxTruncation) {
                            openvdb::Coord c = xyz[b];

                            if (gridAccessor.isValueOn(c) || sd <= _maxTruncation) {
                                TsdVoxelVDB voxel = gridAccessor.getValue(c);
                                voxel.update(sd, _maxTruncation);

                                if (std::abs(voxel._tsd) == _maxTruncation) {
                                    gridAccessor.setActiveState(c, false);
                                } else {
                                    gridAccessor.setValue(c, voxel);
                                }
                            }
                        }
                    }
                }
            }

    cout << "Voxel count: " << _grid->activeVoxelCount() << endl;


    openvdb::FloatGrid::Ptr tsdf = openvdb::createLevelSet<openvdb::FloatGrid>(_voxelSize, 3);
    openvdb::FloatGrid::Accessor tsdfAccessor = tsdf->getAccessor();

    for (openvdb::Grid<openvdb::tree::Tree4<TsdVoxelVDB, 5, 4, 3>::Type>::ValueOnCIter it = _grid->cbeginValueOn();
         it.test(); ++it) {
        //if (it->_weight > 2)
        tsdfAccessor.setValue(it.getCoord(), it->_tsd);
    }

    tsdf->pruneGrid();

    openvdb::io::File file("test.vdb");
    openvdb::GridPtrVec grids;
    grids.push_back(tsdf);
    file.write(grids);
    file.close();
}
