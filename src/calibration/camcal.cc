//
//  main.cpp
//  CameraCalibration
//
//  Created by Joschka van der Lucht on 02.07.15.
//  Copyright (c) 2015 Joschka van der Lucht. All rights reserved.
//

#include <iostream>
#include "calibration/Settings.h"
#include "calibration/CalibrationToolbox.h"
#include <dirent.h>
#include <boost/program_options.hpp>
#include <random>

namespace po = boost::program_options;
using namespace cv;


void createA4File(std::string pfad){
    //Tag: 33mm x 33 mm
    //Rand: 9mm
    // BxH: 7 x 5 Tags
    std::vector<AprilTag3f> apriltags;
    int id = 0;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 7; j++) {
            AprilTag3f tag3f = AprilTag3f(id,Point3f(j * (33 + 9), i * (33 + 9), 0), Point3f(33+j*(33+9),i*(33+9),0), Point3f(33+j*(33+9),33+i*(33+9),0), Point3f(j*(33+9),33+i*(33+9),0));
            apriltags.push_back(tag3f);
            id++;
        }
    }

    std::fstream f;
    f.open((pfad + "aprilA4.txt").c_str(), std::ios::out);
    f << "#APRIL_2D" << std::endl;
    for (AprilTag3f tag : apriltags) {
        f << tag.toString();
    }
    f.close();
}

void createA3File(std::string pfad){
    //Tag: 45mm x 45mm
    //Rand: 11mm
    // BxH: 7 x 5 Tags
    std::vector<AprilTag3f> apriltags;
    int id = 0;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 7; j++) {
            AprilTag3f tag3f = AprilTag3f(id, Point3f(j * (45 + 11), i * (45 + 11), 0), Point3f(45+j*(45+11),i*(45+11),0), Point3f(45+j*(45+11),45+i*(45+11),0), Point3f(j*(45+11),45+i*(45+11),0));
            apriltags.push_back(tag3f);
            id++;
        }
    }
    std::fstream f;
    f.open((pfad + "aprilA3.txt").c_str(), std::ios::out);
    f << "#APRIL_2D" << std::endl;
    for (AprilTag3f tag : apriltags) {
        f << tag.toString();
    }
    f.close();
}



int main(int argc, const char * argv[]) {

    //Timestamp für die Bennenung der Datei mit dem output
    time_t Zeitstempel;
    tm *nun;
    Zeitstempel = time(0);
    nun = localtime(&Zeitstempel);
    std::stringstream timestream;
    timestream<< nun->tm_mday <<"_"<< nun->tm_mon+1 << "__" << nun->tm_hour << "_" << nun->tm_min;

    Settings settings;
    settings.calibrationPattern = Settings::APRILTAG;
    settings.pattern = Settings::APRIL_3D;
    settings.decimate =1;
    settings.debug = false;
    settings.patternPath = "";
    std::string dirpath = "";
    std::string dateiendung = "";
    std::string xmlFileName = "";
    std::string estMatFile = "";
    int picCount;
    bool onlydetect = false;
    bool extrinsic = false;
    settings.debug = false;
    int bx = 0;
    int by = 0;
    float s = 0;

    // Declare the supported options.
    po::variables_map vm;
    po::options_description desc("Allowed options");
    try {
        desc.add_options()
                ("help,h", "produce help message")

                ("patterntype", po::value<std::string>(),
                 "set Patterntype, default APRILTAG, allowed APRILTAG, CHESSBOARD, CIRCLES_GRID; use FROM_FILES to read point pairs (2d 3d) from file")

                ("board-x", po::value<int>(&bx), "set board size x value")
                ("board-y", po::value<int>(&by), "set board size y value")
                ("square", po::value<float>(&s), "set square size")

                ("threads,t", po::value<int>(&settings.threads)->default_value(4),
                 "set threads count for AprilTag detection")

                ("blur", po::value<float>(&settings.blur)->default_value((float) 0.0), "set blur level for AprilTag")

                ("pictures-for-estimation,e",
                 po::value<int>(&settings.picturesForDstimateInitial3DCameraMatrix)->default_value(40),
                 "set the number of Pictures to use for the initial camera matrix")

                ("debug", "set AprilTag to debug mode")

                ("refine-edges", po::value<bool>(&settings.refine_edges)->default_value(true),
                 "set AprilTag refine-edges")

                ("tagFamily", po::value<std::string>(&settings.tagFamily)->default_value("tag36h11"), "set AprilTag TagFamily")

                ("path-pictures,P", po::value<std::string>(&dirpath), "set path to pictures; if use FROM_FILES, path to this files")

                ("read-pictures-form-file,F", po::value<std::string>(&dirpath), "read pictures filename form .txt file")

                ("path-pattern,S", po::value<std::string>(&settings.patternPath), "set path to file with pattern coordinates")

                ("picturetype,T", po::value<std::string>(&dateiendung), "default read all files with png, jpeg, jpg, jpe, tif, tiff, ppm, pgm, bpm")

                ("output-filename,O",
                 po::value<std::string>(&xmlFileName)->default_value("calibration" + timestream.str() + ".xml"),
                 "set name for outputfile")

                ("initial-camera-matrix,M", po::value<std::string>(&estMatFile), "xml-file with initial camera matrix")

                ("no-calibration", "only detect tags and create .detections file")

                ("extrinsic,E","only estimate extrinsic")

                ("visualize", po::value<std::string>(&settings.visualizePath), "output folder for additional debug images");

        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 1;
        }
        if(vm.count("debug")){
            settings.debug = true;
        }
        if(vm.count("no-calibration")){
            onlydetect = true;
        }
        if(vm.count("extrinsic")){
            extrinsic = true;
        }

        // APRILTAG, CHESSBOARD, CIRCLES_GRID
        if (vm.count("patterntype")) {
            if(vm["patterntype"].as<std::string>().compare("APRILTAG") == 0){
                settings.calibrationPattern = Settings::APRILTAG;
            }else if(vm["patterntype"].as<std::string>().compare("CHESSBOARD") == 0){
                settings.calibrationPattern = Settings::CHESSBOARD;
            }else if(vm["patterntype"].as<std::string>().compare("CIRCLES_GRID") == 0){
                settings.calibrationPattern = Settings::CIRCLES_GRID;
            }else if(vm["patterntype"].as<std::string>().compare("FROM_FILES") == 0){
                settings.calibrationPattern = Settings::FROM_FILES;
            }else{
                std::cerr << "patterntype is invalid!, use --help for more information" << std::endl;
                return 1;
            }
        }
        settings.visualize = vm.count("visualize");
    }catch (const boost::program_options::required_option & e) {
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 1;
        } else {
            std::cout << desc << std::endl;
            throw e;
        }
    }
    settings.squareSize = s;
    settings.boardSize = Size(bx,by);

    if(settings.calibrationPattern == Settings::CHESSBOARD){
        if((bx == 0 || by == 0) || settings.squareSize == 0){
            std::cout << "for calibration with CHESSBOARD need boardSize != 0 and squareSize != 0 "<< std::endl;
            return 1;
        }
    }
    if(settings.calibrationPattern == Settings::CIRCLES_GRID){
        std::cout << "not implemented" << std::endl;
        return 1;
    }
    if(settings.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID){
        std::cout << "not implemented" << std::endl;
        return 1;
    }
    if(settings.calibrationPattern == Settings::APRILTAG && settings.patternPath.length() == 0 && (!onlydetect || extrinsic)){
        std::cout << "for calibration with APRILTAGS need pattern file, use --help for more information" <<std::endl;
        return 1;
    }
    if(settings.calibrationPattern != Settings::FROM_FILES) {
        int counter = 0;
        if(vm.count("path-pictures")) {
            dirpath = dirpath + "/";
            // read pictures from path
            std::cout << "read pictures: " << std::endl;
            DIR *dir;
            struct dirent *dirzeiger;
            if ((dir = opendir(dirpath.c_str())) != NULL) {
                while ((dirzeiger = readdir(dir)) != NULL) {
                    std::stringstream pic;
                    pic << dirpath << (*dirzeiger).d_name;
                    std::string picstring = pic.str();
                    std::string defaultdendung = "";
                    std::transform(picstring.begin(), picstring.end(), picstring.begin(), ::tolower);
                    size_t found = std::string::npos;
                    if (dateiendung.length() != 0) {
                        std::transform(dateiendung.begin(), dateiendung.end(), dateiendung.begin(), ::tolower);
                        found = picstring.find(dateiendung);
                    } else {
                        defaultdendung = ".jpg";
                        found = picstring.find(defaultdendung);
                        if (found == std::string::npos) {
                            defaultdendung = ".jpeg";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".jpe";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".png";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".tiff";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".tif";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".ppm";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".pgm";
                            found = picstring.find(defaultdendung);
                        }
                        if (found == std::string::npos) {
                            defaultdendung = ".bmp";
                            found = picstring.find(defaultdendung);
                        }
                    }
                    size_t endung = picstring.find("detections");
                    if (found != std::string::npos && endung == std::string::npos) {
                        settings.picturePath.push_back(pic.str());
                        std::cout << pic.str() << std::endl;
                        counter++;
                    }
                }
                std::cout << "\n" << counter << " pictures read" << std::endl;
            }
        }else if(vm.count("read-pictures-form-file")){
            std::cout << "read pictures from file: " << dirpath << std::endl;
            //TODO read pics from file
            std::ifstream file;
            file.open(dirpath.c_str(), std::ios::in);
            std::string line = "";
            while (getline(file, line))
            {
                //string line = "";
                //getline(file, line);
                std::cout << "picture " << line << std::endl;
                settings.picturePath.push_back(line);
                line ="";
                counter ++;
            }
            file.close();
        }
        if (counter == 0) {
            std::cout << "no pictures found! pleas check path, use --help for more information" << std::endl;
            return 1;
        }
    }else{
        std::cout << "read files: " << std::endl;
        DIR *dir;
        dirpath = dirpath + "/";
        struct dirent *dirzeiger;
        int counter = 0;
        if ((dir = opendir(dirpath.c_str())) != NULL) {
            while ((dirzeiger = readdir(dir)) != NULL) {
                std::stringstream pic;
                pic << dirpath << (*dirzeiger).d_name;
                std::string picstring = pic.str();
                std::string defaultdendung = ".correspondence";
                std::transform(picstring.begin(), picstring.end(), picstring.begin(), ::tolower);
                size_t found = std::string::npos;
                found = picstring.find(defaultdendung);
                if (found != std::string::npos) {
                    settings.picturePath.push_back(pic.str());
                    std::cout << pic.str() << std::endl;
                    counter++;
                }
            }
            std::cout << "\n" << counter << " files read" << std::endl;
        }
        if(counter == 0){
            std::cout << "no files found! pleas check path, use --help for more information" << std::endl;
            return 1;
        }
    }
    if(extrinsic && estMatFile.length()== 0){
        std::cout << "for parameter --extrinsic need also --initial-camera-matrix" << std::endl;
        return 1;
    }
    //if(settings.visualize && onlydetect && estMatFile.length()== 0){
    //    cout << "for parameter --visualize with --no-calibration need --initial-camera-matrix" << endl;
    //    return 1;
    //}
    settings.estimationXML = "ESTIMATION"+xmlFileName; /** dirpath + "/ESTIMATION"+xmlFileName; **/
    settings.outputFileName = xmlFileName; /** dirpath + "/" + xmlFileName; **/


    if(estMatFile.length() != 0){
        settings.estFromInput = true;
        cv::FileStorage fs(estMatFile, cv::FileStorage::READ);
        fs["Camera_Matrix"] >> settings.estCameraMatrix;
        fs["Distortion_Coefficients"] >> settings.estDistCoeff;
        std::cout << "camera matrix: " << std::endl << settings.estCameraMatrix << std::endl;
    }else{
        settings.estFromInput = false;
    }

    std::cout << "start detection" << std::endl;
    CalibrationToolbox calTool = CalibrationToolbox(settings);
    if(extrinsic){
        std::cout << "start extrinsic estimation" << std::endl;
        calTool.computeExtrinsic();
    } else if(!onlydetect) {
        std::cout << "start calibration" << std::endl;
        calTool.calibrate();
    }
    if(settings.visualize && (extrinsic || !onlydetect)){
        calTool.visualize(false);
    }else if(settings.visualize){
        calTool.visualize(true);
    }

    return 0;
}


