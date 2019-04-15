//
// Created by Joschka van der Lucht on 06.03.18.
//

#include <boost/program_options.hpp>
#include "calibration/Detector.h"
#include <dirent.h>

namespace po = boost::program_options;
using namespace cv;

int main(int argc, const char *argv[]) {

    std::string inputPath = "";
    std::string outputPath = "";
    std::string patternType = "";
    std::string tagfam = "";
    int x = 0;
    int y = 0;
    int t = 4;
    bool debug = false;
    int piccounter = 0;
    std::vector<std::string>pathlist=std::vector<std::string>();

    //Declare supported options
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
            ("patterntype,p", po::value<std::string>(&patternType)->default_value("apriltag"), "set the type of used"
                    " pattern, default 'apriltag', allowed 'apriltag' or 'chessboard'");
    output.add_options()
            ("output,o", po::value<std::string>(&outputPath), "output path, by default is "
                    "same as input");
    apriltag.add_options()
            ("tagfamily,f", po::value<std::string>(&tagfam)->default_value("tag36h11"), "set AprilTag family, "
                    "default 'tag36h11'")
            ("threads,t", po::value<int>(&t)->default_value(4), "set tread count for AprilTag detection")
            ("debug,d", po::value<bool>(&debug)->default_value(false), "set debug level for AprilTag detector");
    chessboard.add_options()
            ("board-x,x", po::value<int>(&x), "chessboard bord size x value")
            ("board-y,y", po::value<int>(&y), "chessboard bord size y value");

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
    if (vm.count("patterntype")) {
        if (vm["patterntype"].as<std::string>().compare("apriltag") == 0) {
            patternType = "apriltag";
            if (vm.count("tagfamily")) {
                if (vm["tagfamily"].as<std::string>().compare("tag36h11")) {
                    tagfam = "tag36h11";
                } else if (vm["tagfamily"].as<std::string>().compare("tag25h9")) {
                    tagfam = "tag25h9";
                } else if (vm["tagfamily"].as<std::string>().compare("tag16h5")) {
                    tagfam = "tag16h5";
                } else {
                    std::cerr
                            << "Unknown tag family! Supported families are tag36h11, tag36h10, tag25h9, tag25h7 and tag16h5."
                            << std::endl;
                    std::cout << cmdline_options << std::endl;
                    return 1;
                }
            }
        } else if (vm["patterntype"].as<std::string>().compare("chessboard") == 0) {
            patternType = "chessboard";
            if (x == 0 || y == 0) {
                std::cerr << "Need board size x,y > 0 for chessboard!" << std::endl;
                std::cout << cmdline_options << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Patterntype is invalid! use -- hlep for more information" << std::endl;
            std::cout << cmdline_options << std::endl;
            return 1;
        }
    }
    if (vm.count("threads") && t < 1) {
        std::cerr << "Count of threads must >0! use -- hlep for more information" << std::endl;
        std::cout << cmdline_options << std::endl;
        return 1;
    }
    if(vm.count("output")){
        outputPath = vm["output"].as<std::string>();
    }else{
        outputPath = inputPath;
    }

    //start reading pictures
    inputPath = inputPath + "/";
    DIR *dir;
    struct dirent *dirzeiger;
    if ((dir = opendir(inputPath.c_str())) != NULL) {
        while ((dirzeiger = readdir(dir)) != NULL) {
            std::stringstream pic;
            pic << inputPath << (*dirzeiger).d_name;
            std::string picstring = pic.str();
            std::string filetype = "";
            std::transform(picstring.begin(), picstring.end(), picstring.begin(), ::tolower);
            size_t found = std::string::npos;
            filetype = ".jpg";
            found = picstring.find(filetype);
            if (found == std::string::npos) {
                filetype = ".jpeg";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".jpe";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".png";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".tiff";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".tif";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".ppm";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".pgm";
                found = picstring.find(filetype);
            }
            if (found == std::string::npos) {
                filetype = ".bmp";
                found = picstring.find(filetype);
            }
            size_t existingdetection = picstring.find("detections");
            if (found != std::string::npos && existingdetection == std::string::npos) {
                pathlist.push_back(pic.str());
                std::cout << pic.str() << std::endl;
                piccounter++;
            }
        }
        if(piccounter == 0){
            std::cerr << "No pictures found on " << inputPath << std::endl;
            return 1;
        }else {
            std::cout << "\n" << piccounter << " pictures found" << std::endl;
        }
    }else{
        std::cerr << "Given input path is unvalid!"<< std::endl;
        return 1;
    }

    //start detection
    calibration::Detector detector = calibration::Detector();
    int i = 0;
    for(std::string pic: pathlist){
        std::cout << "\nread picture: " << pic << " " << ++i << "/" << piccounter << std::endl;
        //read picture
        Mat imageCV = imread(pic, CV_LOAD_IMAGE_GRAYSCALE);
        image_u8_t *image;
        if (imageCV.rows > 0 && imageCV.cols > 0) {
            image = image_u8_create(imageCV.cols, imageCV.rows);
            for (int y = 0; y < imageCV.rows; y++) {
                uchar* row = imageCV.ptr<uchar>(y);
                for (int x = 0; x < imageCV.cols; x++) {
                    image->buf[y * image->stride + x] = row[x];
                }
            }
        }
        if (image == NULL) {
            std::cerr << "Can't read picture!\npath: " << pic << "\n"<< std::endl;
            return 1;
        }
        //detect aprilTags
        if(patternType.compare("apriltag")==0){
            std::vector<AprilTag::AprilTag2f> points = std::vector<AprilTag::AprilTag2f>();
            detector.detectAprilTag(image, &points);
            detector.writeApilTagDetectionsToFile((pic+".detections"), points);
        //detect chessboard
        }else if(patternType.compare("chessboard")==0){
            std::vector<cv::Point2f> points = std::vector<cv::Point2f>();
            detector.detectChessboard(imageCV, &points, cv::Size(x,y));
            std::cout << "points.size(): " << points.size() << std::endl;
            detector.writeChessboardDetectionsToFile((pic+".detections"), points);
        }
    }
}
