#include "slam6d/scan.h"
#include <cppunit/extensions/HelperMacros.h>
#include <openvdb/openvdb.h>
#include <openvdb/Types.h>
#include <boost/program_options.hpp>
#include <openvdb/tools/VolumeToMesh.h>
#include <chrono>
#include <boost/algorithm/string.hpp>

using namespace std;

int main(int argc, char **argv)
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("input,i", boost::program_options::value<std::string>(), "input file")
            ("output,o", boost::program_options::value<std::string>()->default_value("mesh.obj"), "output file")
            ("bbox,b", boost::program_options::value<std::string>()->default_value("0,0,0,10,10,10"), "bounding box")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    openvdb::initialize();

    openvdb::io::File file(vm["input"].as<std::string>());
    file.open();
    openvdb::GridPtrVecPtr myGrids = file.getGrids();
    file.close();

    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(myGrids->at(0));
    openvdb::BoolGrid::Ptr mask = openvdb::BoolGrid::create();

    std::string bboxString = vm["bbox"].as<std::string>();

    std::vector<std::string> strs;
    boost::split(strs, bboxString, boost::is_any_of(","));

    if (strs.size() != 6) {
        cout << "Failed to parse bounding box!" << endl;
    }

    float bboxValues[6];
    for (int i = 0; i < 6; i++) {
        bboxValues[i] = std::stof(strs.at(i));
    }

    openvdb::Coord min(bboxValues[0], bboxValues[1], bboxValues[2]);
    openvdb::Coord max(bboxValues[3], bboxValues[4], bboxValues[5]);
    openvdb::CoordBBox bbox(min, max);
    mask->fill(bbox, true, true);

    cout << "Mask size: " << mask->activeVoxelCount() << endl;

    openvdb::tools::VolumeToMesh mesher(grid->getGridClass() == openvdb::GRID_LEVEL_SET ? 0.0 : 0.01);
    mesher.setSurfaceMask(mask);

    auto start = chrono::high_resolution_clock::now();

    mesher(*grid);

    cout << "Meshing: " << chrono::duration<double, milli> (chrono::high_resolution_clock::now() - start).count() << " ms" << endl;

    openvdb::tools::PolygonPoolList& polygonPoolList = mesher.polygonPoolList();

    openvdb::Index64 numQuads = 0;
    for (openvdb::Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
        numQuads += polygonPoolList[n].numQuads();
    }

    cout << "Number of points: " << mesher.pointListSize() << endl;
    cout << "Number of quads: " << numQuads << endl;

    std::fstream f(vm["output"].as<std::string>(), std::fstream::out);

    for (openvdb::Index64 n = 0, N = mesher.pointListSize(); n < N; ++n) {
        const openvdb::Vec3s& p = mesher.pointList()[n];

        f << "v " << p[0] << " " << p[1] << " " << p[2] << endl;
    }

    for (openvdb::Index64 n = 0, N = mesher.polygonPoolListSize(); n < N; ++n) {
        const openvdb::tools::PolygonPool& polygons = polygonPoolList[n];
        for (openvdb::Index64 i = 0, I = polygons.numQuads(); i < I; ++i) {
            const openvdb::Vec4I& quad = polygons.quad(i);

            f << "f " << quad[0] + 1 << " " << quad[1] + 1 << " " << quad[2] + 1 << " " << quad[3] + 1 << endl;
        }
    }

    return 0;
}
