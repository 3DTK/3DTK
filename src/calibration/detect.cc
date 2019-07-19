//
// Created by Joschka van der Lucht on 06.03.18.
//

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include "calibration/Detector.h"
#include "calibration/AprilTagDetector.h"
#include "calibration/ChessboardDetector.h"

#include <dirent.h>

namespace po = boost::program_options;
using namespace cv;

namespace std {
ostream& operator<<(ostream &os, const vector<string> &vec)
{
    for (auto item : vec) {
        os << item << " ";
    }
    return os;
}
}

void findAllFilesByExtension(const boost::filesystem::path& root, const std::string& ext, std::vector<std::string>& ret)
{
    if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) {
        return;
    }

    boost::filesystem::recursive_directory_iterator it(root);
    boost::filesystem::recursive_directory_iterator endit;

    while (it != endit) {
        if (boost::filesystem::is_regular_file(*it) && boost::iequals(it->path().extension().string(), ext)) {
            ret.push_back(boost::filesystem::canonical(it->path()).string());
        }
        ++it;
    }
}

int main(int argc, const char *argv[]) {

    std::string inputPath;
    std::string outputPath;
    std::string patternType;
    std::vector<std::string> extensions;
    std::string tagFamily;
    float decimate;
    float blur;
    int hamming;
    bool refineEdges;
    int threads;
    bool debug;
    int x;
    int y;
    bool adaptiveThreshold;
    bool normalizeImage;
    bool filterQuads;
    bool fastCheck;

    // Declare supported options
    po::variables_map vm;
    po::options_description generic("Generic options");
    po::options_description input("Input options");
    po::options_description output("Output options");
    po::options_description apriltag("AprilTag options");
    po::options_description chessboard("Chessboard options");
    po::options_description hidden("Hidden options");

    generic.add_options()
            ("help,h", "produce help message");
    hidden.add_options()
            ("input,i", po::value<std::string>(&inputPath)->required(), "input path with pictures");
    input.add_options()
            ("patterntype,p", po::value<std::string>(&patternType)->default_value("apriltag"),
             "set the type of used pattern, default 'apriltag', allowed 'apriltag' or 'chessboard'")
            ("force", "run pattern detection even if a '.detections' file was found.")
            ("extensions,e", po::value<std::vector<std::string> >(&extensions)->multitoken()->default_value(
                 std::vector<std::string> {".jpg", ".jpeg", ".jpe", ".png", ".tiff", ".tif", ".ppm", ".pgm", ".bmp"}),
             "filename extensions of input image files");
    output.add_options()
            ("output,o", po::value<std::string>(&outputPath), "output path, by default is "
                                                              "same as input");
    apriltag.add_options()
            ("tagfamily,f", po::value<std::string>(&tagFamily)->default_value("tag36h11"), "set AprilTag family, "
                                                                                           "default 'tag36h11'")
            ("decimate,d", po::value<float>(&decimate)->default_value(0.0), "decimate input image by this factor")
            ("blur,b", po::value<float>(&blur)->default_value(0.0), "apply low-pass blur to input; negative sharpens")
            ("hamming,h", po::value<int>(&hamming)->default_value(0), "detect tags with up to this many bit errors")
            ("refine-edges", po::value<bool>(&refineEdges)->default_value(true), "spend more time trying to align edges of tags")
            ("threads,t", po::value<int>(&threads)->default_value(4), "set thread count for AprilTag detection")
            ("debug", po::value<bool>(&debug)->default_value(false), "enable debug output for AprilTag detector");
    chessboard.add_options()
            ("board-x,x", po::value<int>(&x)->default_value(0), "chessboard bord size x value")
            ("board-y,y", po::value<int>(&y)->default_value(0), "chessboard bord size y value")
            ("adaptive-threshold", po::value<bool>(&adaptiveThreshold)->default_value(true),
             "use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level")
            ("normalize-image", po::value<bool>(&normalizeImage)->default_value(true),
             "normalize the image gamma before applying fixed or adaptive thresholding")
            ("filter-quads", po::value<bool>(&filterQuads)->default_value(true),
             "use additional criteria (like contour area, perimeter, square-like shape) to filter out false quads")
            ("fast-check", po::value<bool>(&fastCheck)->default_value(false),
             "run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found");

    po::options_description all;
    all.add(generic).add(hidden).add(input).add(output).add(apriltag).add(chessboard);

    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(output).add(apriltag).add(chessboard);

    po::positional_options_description pd;
    pd.add("input", 1);

    po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vm);

    try {
        po::notify(vm);
    } catch (const po::required_option &e) {
        if (vm.count("help")) {
            std::cout << cmdline_options << std::endl;
            return 1;
        } else {
            std::cout << all << std::endl;
            throw e;
        }
    }
    if (vm.count("help")) {
        std::cout << all << "\n";
        return 1;
    }
    if (vm.count("threads") && threads < 1) {
        std::cerr << "Number of threads must > 0! use -- help for more information" << std::endl;
        std::cout << cmdline_options << std::endl;
        return -1;
    }
    if (vm.count("output")) {
        outputPath = vm["output"].as<std::string>();
    } else {
        outputPath = inputPath;
    }
    bool forceRecompute = false;
    if (vm.count("force")) {
        forceRecompute = true;
    }

    // Check if input path is valid
    if (!boost::filesystem::exists(boost::filesystem::path(inputPath)) ||
            !boost::filesystem::is_directory(boost::filesystem::path(inputPath))) {
        std::cerr << "Given input path is invalid!"<< std::endl;
        return -1;
    }

    // Create pattern detector based on command line options
    calibration::Detector* detector;
    if (vm["patterntype"].as<std::string>().compare("apriltag") == 0) {
        if (!tagFamily.compare("tag36h11") && !tagFamily.compare("tag25h9") && !tagFamily.compare("tag16h5")) {
            std::cerr
                    << "Unknown tag family! Supported families are tag36h11,tag25h9 and tag16h5."
                    << std::endl;
            std::cout << cmdline_options << std::endl;
            return 1;
        }

        detector = new calibration::AprilTagDetector(std::vector<AprilTag::AprilTag3f>(), tagFamily, decimate, blur, hamming, refineEdges, threads, debug);
    } else if (vm["patterntype"].as<std::string>().compare("chessboard") == 0) {
        if (x <= 2 || y <= 2) {
            std::cerr << "Board size is invalid! use -- help for more information" << std::endl;
            std::cerr << "Both board size x and board size y of the chessboard should be bigger than 2!" << std::endl;
            std::cout << cmdline_options << std::endl;
            return -1;
        }

        detector = new calibration::ChessboardDetector(cv::Size(x, y), -1, adaptiveThreshold, normalizeImage, filterQuads, fastCheck);
    } else {
        std::cerr << "Patterntype is invalid! use -- help for more information" << std::endl;
        std::cout << cmdline_options << std::endl;
        return -1;
    }

    // Find image files in input directory
    std::vector<std::string>pathList;
    for (const std::string& ext : extensions) {
        findAllFilesByExtension(boost::filesystem::path(inputPath), ext, pathList);
    }
    if (pathList.size() == 0){
        std::cerr << "No pictures found on " << inputPath << std::endl;
        return 1;
    } else {
        std::cout << "\n" << pathList.size() << " pictures found" << std::endl;
    }

    // Run pattern detector on all input images and write detection files
    int i = 0;
    for (std::string imageFile : pathList) {
        std::cout << "\nread picture: " << imageFile << " " << ++i << "/" << pathList.size() << std::endl;

        // skip pattern detection if a detections file already exists
        if (!forceRecompute && boost::filesystem::exists(imageFile + ".detections")) {
            std::cout << "Skipping detection because '.detections' file already exists." << std::endl;
            continue;
        }

        Mat image = imread(imageFile);

        if (image.rows < 1 || image.cols < 1) {
            std::cerr << "Can't read picture!\npath: " << imageFile << "\n"<< std::endl;
            return -1;
        }

        if (detector->detect(image)) {
            detector->writeDetectionsToFile(imageFile + ".detections");
            std::cout << "Time to detect: " << detector->getDetectionTimeMilliSec() << " ms" << std::endl;
            std::cout << "Found " << detector->getImagePoints().size() << " image points." << std::endl;
        } else {
            std::cout << "No pattern detected!" << std::endl;
        }
    }

    delete detector;
}
