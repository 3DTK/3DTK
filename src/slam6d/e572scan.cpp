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

void readPoints(const std::string inputPath);

int main(int argc, char *argv[]){
    std::string extension = ".e57";
    std::string inputPath = "";
    std::string outputPath ="";
    double scale = 1.0;
    int xDirection = 1;

    try {
        po::options_description generic("Generic options");
        generic.add_options()
                ("help,h", "output this help message");

        po::options_description output("Output options");
        output.add_options()
                ("scale,s", po::value<double>(&scale)->default_value(1.0), "scale the point cloud")
                ("invertX,x", "invert x-axes to convert from right to left handed system")
                ("output,o", po::value<std::string>(&outputPath), "output directory");

        po::options_description hidden("Hidden options");
        hidden.add_options()
                ("input", po::value(&inputPath), "input .e57 file");

        // all options
        po::options_description all;
        all.add(generic).add(output).add(hidden);

        // options visible with --help
        po::options_description cmdline_options;
        cmdline_options.add(generic).add(output);

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
        if(vm.count("invertX")){
            std::cout << "invert x axis." << std::endl;
            xDirection = -1;
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
    readPoints(inputPath);
    return 0;
};

bool writePose(double *translation, double *rotation, int scanid){
    double mat [16];
    QuatToMatrix4( rotation, translation, mat);
    double t [3];
    double r [3];
    Matrix4ToEuler(mat, r, t);
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << scanid;
    std::string filename = "scan" + ss.str() + ".pose";
    boost::filesystem::ofstream file (filename);
    file << t[0] << " " << t[1] << " " << t[2] << "\n"
            << r[0] << " " << r[1] << " " << r[2] << " ";
    file.close();
    return true;
}

void readPoints(const std::string inputPath){
    try {
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
        for (int scanIndex = 0; scanIndex < scanCount; scanIndex++) {
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
            // Get pose
            e57::StructureNode pose(scan.get("pose"));

            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << scanIndex;
            std::string filename = "scan" + ss.str() + ".3d";
            boost::filesystem::ofstream file (filename);

            double tr[3];
            double rq[4];
            if(pose.isDefined("translation")){
                e57::StructureNode translation (pose.get("translation"));
                e57::FloatNode tx (translation.get("x"));
                e57::FloatNode ty (translation.get("y"));
                e57::FloatNode tz (translation.get("z"));
                tr[0] = tx.value();
                tr[1] = ty.value();
                tr[2] = tz.value();
                std::cout << "translation x:" << tx.value() << " y:" << ty.value() << " z:" << tz.value() << std::endl;
            } else{
                tr[0] = 0;
                tr[1] = 0;
                tr[2] = 0;
            }
            if(pose.isDefined("rotation")){
                e57::StructureNode rotation (pose.get("rotation"));
                e57::FloatNode rw (rotation.get("w"));
                e57::FloatNode rx (rotation.get("x"));
                e57::FloatNode ry (rotation.get("y"));
                e57::FloatNode rz (rotation.get("z"));
                rq[0] = rx.value();
                rq[1] = ry.value();
                rq[2] = rz.value();
                rq[3] = rw.value();
                std::cout << "rotation w:" << rw.value() << " x:" << rx.value() << " y:" << ry.value() << " z:" << rz.value() << std::endl;
            }else{
                rq[0] = 0;
                rq[1] = 0;
                rq[2] = 0;
                rq[3] = 0;
            }
            if(writePose(tr, rq, scanIndex)){
                std::cout << "write pose" << std::endl;
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

                std::cout << "??? not implemented yet" << std::endl;
                /// Make a list of buffers to receive the xyz values.
                //TODO get all points
                const int N = 200000 ;
                std::vector<e57::SourceDestBuffer> destBuffers;
                double range[N];
                double eleavtion[N];
                double azimuth[N];
                int r[N];
                int g[N];
                int b[N];
                destBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalRange", range, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalAzimuth", azimuth, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalElevation", eleavtion, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "colorRed", r, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "colorGreen", g, N, true));
                destBuffers.push_back(e57::SourceDestBuffer(imf, "colorBlue", b, N, true));

                /// Create a reader of the points CompressedVector, try to read first block of N points
                /// Each call to reader.read() will fill the xyz buffers until the points are exhausted.
                e57::CompressedVectorReader reader = points.reader(destBuffers);
                unsigned gotCount = reader.read();
                std::cout << "  got first " << gotCount << " points" << std::endl;

                /// Print the coordinates we got
                for (unsigned i=0; i < gotCount; i++) {
//                    std::cout << "  " << i << ". range=" << range[i] << " eleavtion=" << eleavtion[i] << " azimuth="
//                              << azimuth[i] << " r=" << r[i] << " g=" << g[i] << " b=" << b[i] << std::endl;
                    double cartesian [3];
                    double polar[3];
                    double rgb[3];
                    polar[0] = range[i];
                    polar[1] = azimuth[i];
                    polar[2] = eleavtion[i];
                    //TODO convert to cartesian (use toCartesian()) and write down to scan file. Also export RGB.
                    toCartesian(polar, cartesian);
                    rgb[0] = r[i];
                    rgb[1] = g[i];
                    rgb[2] = b[i];
//                    std::cout << "\t" << cartesian[0] << " " << cartesian[1] << " " << cartesian[2] << std::endl;
                    file << cartesian[0] << " " << cartesian[1] << " " << cartesian[2] << " " << rgb[0] << " " << rgb[1] << " " << rgb[2] << "\n";
                }
            } else
                std::cout << "Error: couldn't find either Cartesian or spherical points in scan" << std::endl;
            std::cout << std::endl;
            file.close();
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
