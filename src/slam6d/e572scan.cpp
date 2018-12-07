//
// Created by Joschka van der Lucht on 20.11.18.
// tool to convert .e57 files to scans
//

#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include "../../3rdparty/e57-3d-imgfmt/include/E57Foundation.h"
#include <slam6d/globals.icc>

namespace po = boost::program_options;

void readPoints(const std::string inputPath, const std::string outputPath, double scale, int start, int end, double minDist, double maxDist);

int main(int argc, char *argv[]){
    std::string extension = ".e57";
    std::string inputPath = "";
    std::string outputPath ="";
    int start = 0;
    int end = -1;
    double scale = 1.0;
    double minDist = 0.0;
    double maxDist = -1.0;

    try {
        po::options_description generic("Generic options");
        generic.add_options()
                ("help,h", "output this help message");

        po::options_description output("Output options");
        output.add_options()
                ("min, m",po::value<double>(&minDist)->default_value(0.0), "min distance")
                ("max, M",po::value<double>(&maxDist)->default_value(-1.0), "max distance")
                ("scale", po::value<double>(&scale)->default_value(1.0), "scale the point cloud")
                ("output,o", po::value<std::string>(&outputPath)->default_value("."), "output directory");
        po::options_description input("Input options");
        input.add_options()
                ("start,s", po::value<int>(&start)->default_value(0), "start scan")
                ("end,e", po::value<int>(&end)->default_value(-1), "end scan");

        po::options_description hidden("Hidden options");
        hidden.add_options()
                ("input", po::value(&inputPath), "input .e57 file");

        // all options
        po::options_description all;
        all.add(generic).add(output).add(hidden).add(input);

        // options visible with --help
        po::options_description cmdline_options;
        cmdline_options.add(generic).add(output).add(input);

        // positional argument
        po::positional_options_description pd;
        pd.add("input", 1);

        // process options
        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                options(all).positional(pd).run(), vm);
        po::notify(vm);
        // display help
        if (vm.count("help")) {
            std::cout << "Helper tool to convert .e57 files to scan files\n"
                      << cmdline_options;
            exit(0);
        }
        if(inputPath.length() == 0){
            std::cerr << "Missing input file!\n" << inputPath << std::endl;
            std::cout << cmdline_options;
            exit(1);
        }else{
            // find the last occurrence of '.'
            size_t pos = inputPath.find_last_of(".");
            // make sure the poisition is valid
            std::string ex = "";
            if (pos != std::string::npos) {
                ex = inputPath.substr(pos + 1);
            }
            if(!extension.compare(ex)){
                std::cout << "Input file is not a .e57 file!" << std::endl;
                exit(1);
            }
        }
        //if(outputPath.length() == 0){
        //    std::cerr << "Missing output directory!" << std::endl;
        //    std::cout << cmdline_options;
        //    exit(1);
        //}
    }catch (std::exception& e){
        std::cerr << "Error while parsing settings: " << e.what() << std::endl;
        exit(1);
    }
    readPoints(inputPath, outputPath, scale, start, end, minDist, maxDist);
    return 0;
};

// transmat 3x3 rotation, x,y,z translation, mat output
void setTransform(double *transmat, double *mat, double x, double y, double z) {
    transmat[0] = mat[4];
    transmat[1] = -mat[7];
    transmat[2] = -mat[1];
    transmat[3] = 0.0;

    transmat[4] = -mat[5];
    transmat[5] = mat[8];
    transmat[6] = mat[2];
    transmat[7] = 0.0;

    transmat[8] = -mat[3];
    transmat[9] = mat[6];
    transmat[10] = mat[0];
    transmat[11] = 0.0;

    // translation
    transmat[12] =  -y;
    transmat[13] =  z;
    transmat[14] =  x;
    transmat[15] = 1;
}

void QuatToMatrix4RightHand(const double * qaut, const double * t, double * mat){
    mat[0] = 1 - 2 * qaut[1] * qaut[1] - 2 * qaut[2] * qaut[2];
    mat[1] = 2 * (qaut[0] * qaut[1] - qaut[3] * qaut[2]);
    mat[2] = 2.0 * (qaut[0] * qaut[2] + qaut[3] * qaut[1]);
    mat[3] = t[0];

    mat[4] = 2 * (qaut[0] * qaut[1] + qaut[3] * qaut[2]);
    mat[5] = 1.0 - 2.0 * qaut[0] * qaut[0] - 2.0 * qaut[2] * qaut[2];
    mat[6] = 2.0 * (qaut[1] * qaut[2] - qaut[3] * qaut[0]);
    mat[7] = t[1];

    mat[8] = 2 * (qaut[0] * qaut[2] - qaut[3] * qaut[1]);
    mat[9] = 2.0 * (qaut[1] * qaut[2] + qaut[3] * qaut[0]);
    mat[10] = 1.0 - 2.0 * qaut[0] * qaut[0] - 2.0 * qaut[1] * qaut[1];
    mat[11] = t[2];

    mat[12] = 0;
    mat[13] = 0;
    mat[14] = 0;
    mat[15] = 1;

}

bool writePose(double *translation, double *rotation, int scanid, std::string path){
    double inMatrix [16];
    //TODO selbst bauen QuatToMatrix4( rotation, translation, inMatrix);
    double t [3];
    double r [3];
    double tMatrix[16];
    double roation [9];

    QuatToMatrix4RightHand( rotation, translation, inMatrix);

    roation[0] = inMatrix[0];
    roation[1] = inMatrix[1];
    roation[2] = inMatrix[2];
    roation[3] = inMatrix[4];
    roation[4] = inMatrix[5];
    roation[5] = inMatrix[6];
    roation[6] = inMatrix[8];
    roation[7] = inMatrix[9];
    roation[8] = inMatrix[10];
    setTransform(tMatrix, roation, translation[0], translation[1], translation[2]);
    Matrix4ToEuler(tMatrix, r, t);

    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << scanid;
    std::string filename = path + "/scan" + ss.str() + ".pose";
    boost::filesystem::ofstream file (filename);
    file << t[0] << " " << t[1] << " " << t[2] << "\n" << deg(r[0]) << " " << deg(r[1]) << " " << deg(r[2]) << " ";
//    file << translation[0] << " " << translation[1] << " " << translation[2] << "\n" << deg(r[0]) << " " << deg(r[1]) << " " << deg(r[2]) << " ";
    file.close();
    return true;
}

void readPoints(const std::string inputPath, const std::string outputPath, double scale, int start, int end, double minDist, double maxDist){
    try {
        //TODO check if RGB is available
        /// Read file from disk
        e57::ImageFile imf(inputPath, "r");
        e57::StructureNode root = imf.root();

        /// Make sure vector of scans is defined and of expected type.
        /// If "/data3D" wasn't defined, the call to root.get below would raise an exception.
        if (!root.isDefined("/data3D")) {
            std::cout << "File doesn't contain 3D images" << std::endl;
            exit(0);
        }
        e57::Node n = root.get("/data3D");
        if (n.type() != e57::E57_VECTOR) {
            std::cout << "bad file" << std::endl;
            exit(1);
        }

        /// The node is a vector so we can safely get a VectorNode handle to it.
        /// If n was not a VectorNode, this would raise an exception.
        e57::VectorNode data3D(n);

        /// Print number of children of data3D.  This is the number of scans in file.
        int64_t scanCount = data3D.childCount();
        std::cout << "Number of scans in file:" << scanCount << std::endl;

        /// For each scan, print out first 4 points in either Cartesian or Spherical coordinates.
        if(start >= scanCount) exit(1);
        if(end >= scanCount) end = scanCount-1;
        if(end > 0 && end < start) exit(1);
        if(start < 0) start = 0;

        for (int scanIndex = start; scanIndex < scanCount; scanIndex++) {
            /// Get scan from "/data3D", assume its a Structure (else get exception)
            e57::StructureNode scan(data3D.get(scanIndex));
            std::cout << "got:" << scan.pathName() << std::endl;
            e57::StringNode name (scan.get("name"));
            std::cout << "name: " << name.value() << std::endl;
            e57::StringNode desc (scan.get("description"));
            std::cout << "description: " << desc.value() <<std::endl;
            /// Get "points" field in scan.  Should be a CompressedVectorNode.
            e57::CompressedVectorNode points(scan.get("points"));
            std::cout << "point count: " <<points.childCount() << std::endl;

            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << scanIndex;
            std::string filename = outputPath + "/scan" + ss.str() + ".3d";
            boost::filesystem::ofstream file (filename);

            double tr[3];
            double rq[4];
            // Get pose
            if(scan.isDefined("pose")){
                e57::StructureNode pose(scan.get("pose"));
                if(scan.isDefined("pose") && pose.isDefined("translation")){
                    e57::StructureNode translation (pose.get("translation"));
                    e57::FloatNode tx (translation.get("x"));
                    e57::FloatNode ty (translation.get("y"));
                    e57::FloatNode tz (translation.get("z"));
                    tr[0] = tx.value()*scale;
                    tr[1] = ty.value()*scale;
                    tr[2] = tz.value()*scale;
                    std::cout << "translation: x=" << tr[0] << " y=" << tr[1] << " z=" << tr[2] << std::endl;
                } else{
                    tr[0] = 0;
                    tr[1] = 0;
                    tr[2] = 0;
                }
                if(scan.isDefined("pose") && pose.isDefined("rotation")){
                    e57::StructureNode rotation (pose.get("rotation"));
                    e57::FloatNode rw (rotation.get("w"));
                    e57::FloatNode rx (rotation.get("x"));
                    e57::FloatNode ry (rotation.get("y"));
                    e57::FloatNode rz (rotation.get("z"));
                    rq[0] = rx.value();
                    rq[1] = ry.value();
                    rq[2] = rz.value();
                    rq[3] = rw.value();
                    std::cout << "rotation: w=" << rw.value() << " x=" << rx.value() << " y=" << ry.value() << " z=" << rz.value() << std::endl;
                }else{
                    rq[0] = 0;
                    rq[1] = 0;
                    rq[2] = 0;
                    rq[3] = 0;
                }
            } else {
                tr[0] = 0;
                tr[1] = 0;
                tr[2] = 0;
                rq[0] = 0;
                rq[1] = 0;
                rq[2] = 0;
                rq[3] = 0;
            }

            if(writePose(tr, rq, scanIndex, outputPath)){
                std::cout << "write pose..." << std::endl;
            }


            /// Call subroutine in this file to print the points
            //printSomePoints(imf, points);
            /// Need to figure out if has Cartesian or spherical coordinate system.
            /// Interrogate the CompressedVector's prototype of its records.
            e57::StructureNode proto(points.prototype());

            /// The prototype should have a field named either "cartesianX" or "sphericalRange".
            if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ")) {
                //TODO not working
                /// Make a list of buffers to receive the xyz values.
                std::vector<e57::SourceDestBuffer> destBuffers;
                int64_t columnIndex[10];
                destBuffers.push_back(e57::SourceDestBuffer(imf, "columnIndex", columnIndex, 10, true));

                /// Create a reader of the points CompressedVector, try to read first block of 4 columnIndex
                /// Each call to reader.read() will fill the xyz buffers until the points are exhausted.
                e57::CompressedVectorReader reader = points.reader(destBuffers);
                unsigned gotCount = reader.read();
                std::cout << "  got first " << gotCount << " points" << std::endl;

                /// Print the coordinates we got
                for (unsigned i=0; i < gotCount; i++)
                    std::cout << "  " << i << ". columnIndex=" << columnIndex[i] << std::endl;
            }else if (proto.isDefined("sphericalRange")) {
                /// Make a list of buffers to receive the xyz values.
                const int N = points.childCount();
                std::vector<e57::SourceDestBuffer> destBuffers;

                double * range = (double *) malloc(sizeof(double)*N);
                double * elevation = (double *) malloc(sizeof(double)*N);
                double * azimuth = (double *) malloc(sizeof(double)*N);;
                int * r = (int *) malloc(sizeof(int)*N);
                int * g = (int *) malloc(sizeof(int)*N);
                int * b = (int *) malloc(sizeof(int)*N);
                destBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalRange", range, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalAzimuth", azimuth, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalElevation", elevation, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "colorRed", r, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "colorGreen", g, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "colorBlue", b, N, true));

                /// Create a reader of the points CompressedVector, try to read first block of N points
                /// Each call to reader.read() will fill the xyz buffers until the points are exhausted.
                e57::CompressedVectorReader reader = points.reader(destBuffers);
                unsigned gotCount = reader.read();
                std::cout << "write scan...0 % \r";
                std::cout.flush();

                /// convert the coordinates to cartesian and write to file
                for (unsigned i=0; i < gotCount; i++) {
                    if(minDist <= range[i]*scale && (maxDist < 0 || range[i]*scale <= maxDist)){
                        double cartesian [3];
                        double polar[3];
                        double rgb[3];
                        polar[2] = range[i]*scale;
                        polar[1] = azimuth[i];
                        polar[0] = elevation[i];
                        toCartesianWithElevation(polar, cartesian);
                        rgb[0] = r[i];
                        rgb[1] = g[i];
                        rgb[2] = b[i];
//                    file << cartesian[0] << " " << cartesian[1] << " " << cartesian[2] << " " << rgb[0] << " " << rgb[1] << " " << rgb[2] << "\n";
                        /// convert to 3DTK coordinate system
                        file << (cartesian[1]*(-1)) << " " << cartesian[2] << " " << cartesian[0] << " " << rgb[0] << " " << rgb[1] << " " << rgb[2] << "\n";
                    }
                    std::cout << "write scan..." << ((i+1)*100.0f/gotCount) << " % \r";
                    std::cout.flush();
                }
                free(range);
                free(elevation);
                free(azimuth);
                free(r);
                free(g);
                free(b);
            } else
                std::cout << "Error: couldn't find either Cartesian or spherical points in scan! Other formats not implemented." << std::endl;
            std::cout << std::endl << std::endl;
            file.close();
            if(end >= 0 && scanIndex == end) break;
        }

        imf.close();
    } catch(e57::E57Exception& ex) {
        ex.report(__FILE__, __LINE__, __FUNCTION__);
        exit(1);
    } catch (std::exception& ex) {
        std::cerr << "Got an std::exception, what=" << ex.what() << std::endl;
        exit(1);
    } catch (...) {
        std::cerr << "Got an unknown exception" << std::endl;
        exit(1);
    }
}
