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


void createA4File(string pfad){
    //Tag: 33mm x 33 mm
    //Rand: 9mm
    // BxH: 7 x 5 Tags
    vector<AprilTag3f> apriltags;
    int id = 0;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 7; j++) {
            AprilTag3f tag3f = AprilTag3f(id,Point3f(j * (33 + 9), i * (33 + 9), 0), Point3f(33+j*(33+9),i*(33+9),0), Point3f(33+j*(33+9),33+i*(33+9),0), Point3f(j*(33+9),33+i*(33+9),0));
            apriltags.push_back(tag3f);
            id++;
        }
    }

    fstream f;
    f.open((pfad + "aprilA4.txt").c_str(), ios::out);
    f << "#APRIL_2D" << endl;
    for (AprilTag3f tag : apriltags) {
        f << tag.toString();
    }
    f.close();
}

void createA3File(string pfad){
    //Tag: 45mm x 45mm
    //Rand: 11mm
    // BxH: 7 x 5 Tags
    vector<AprilTag3f> apriltags;
    int id = 0;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 7; j++) {
            AprilTag3f tag3f = AprilTag3f(id, Point3f(j * (45 + 11), i * (45 + 11), 0), Point3f(45+j*(45+11),i*(45+11),0), Point3f(45+j*(45+11),45+i*(45+11),0), Point3f(j*(45+11),45+i*(45+11),0));
            apriltags.push_back(tag3f);
            id++;
        }
    }
    fstream f;
    f.open((pfad + "aprilA3.txt").c_str(), ios::out);
    f << "#APRIL_2D" << endl;
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
    stringstream timestream;
    timestream<< nun->tm_mday <<"_"<< nun->tm_mon+1 << "__" << nun->tm_hour << "_" << nun->tm_min;

    Settings settings;
    settings.calibrationPattern = Settings::APRILTAG;
    settings.pattern = Settings::APRIL_3D;
    settings.decimate =1;
    settings.debug = false;
    settings.patternPath = "";
    string dirpath = "";
    string dateiendung = "";
    string xmlFileName = "";
    string estMatFile = "";
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

                ("patterntype", po::value<string>(),
                 "set Patterntype, default APRILTAG, allowed APRILTAG, CHESSBOARD, CIRCLES_GRID; use FROM_FILES to read point pairs (2d 3d) from file")

                ("board-x", po::value<int>(&bx), "set board size x value")
                ("board-y", po::value<int>(&by), "set board size y value")
                ("square", po::value<float>(&s), "set square size")

                ("threads,t", po::value<int>(&settings.threads)->default_value(4),
                 "set threads count for AprilTag detection")

                ("blur", po::value<float>(&settings.blur)->default_value((float) 0.8), "set blur level for AprilTag")

                ("pictures-for-estimation,e",
                 po::value<int>(&settings.picturesForDstimateInitial3DCameraMatrix)->default_value(40),
                 "set the number of Pictures to use for the initial camera matrix")

                ("debug", "set AprilTag to debug mode")

                ("refine-edges", po::value<bool>(&settings.refine_edges)->default_value(true),
                 "set AprilTag refine-edges")

                ("refine-decodes", po::value<bool>(&settings.refine_decodes)->default_value(true),
                 "set AprilTag refine-decodes")

                ("refine-pose", po::value<bool>(&settings.refine_pose)->default_value(true),
                 "set AprilTag refine-pose")

                ("tagFamily", po::value<string>(&settings.tagFamily)->default_value("tag36h11"), "set AprilTag TagFamily")

                ("path-pictures,P", po::value<string>(&dirpath), "set path to pictures; if use FROM_FILES, path to this files")

                ("path-pattern,S", po::value<string>(&settings.patternPath), "set path to file with pattern coordinates")

                ("picturetype,T", po::value<string>(&dateiendung), "default read all files with png, jpeg, jpg, jpe, tif, tiff, ppm, pgm, bpm")

                ("output-filename,O",
                 po::value<string>(&xmlFileName)->default_value("calibration" + timestream.str() + ".xml"),
                 "set name for outputfile")

                ("initial-camera-matrix,M", po::value<string>(&estMatFile), "xml-file with initial camera matrix")

                ("no-calibration", "only detect tags and create .detections file")

                ("extrinsic,E","only estimate extrinsic")

                ("visualize", po::value<string>(&settings.visualizePath), "output folder for additional debug images");

        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            cout << desc << "\n";
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
            if(vm["patterntype"].as<string>().compare("APRILTAG") == 0){
                settings.calibrationPattern = Settings::APRILTAG;
            }else if(vm["patterntype"].as<string>().compare("CHESSBOARD") == 0){
                settings.calibrationPattern = Settings::CHESSBOARD;
            }else if(vm["patterntype"].as<string>().compare("CIRCLES_GRID") == 0){
                settings.calibrationPattern = Settings::CIRCLES_GRID;
            }else if(vm["patterntype"].as<string>().compare("FROM_FILES") == 0){
                settings.calibrationPattern = Settings::FROM_FILES;
            }else{
                cerr << "patterntype is invalid!, use --help for more information" << endl;
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
            cout << "for calibration with CHESSBOARD need boardSize != 0 and squareSize != 0 "<< endl;
            return 1;
        }
    }
    if(settings.calibrationPattern == Settings::CIRCLES_GRID){
        cout << "not implemented" << endl;
        return 1;
    }
    if(settings.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID){
        cout << "not implemented" << endl;
        return 1;
    }
    if(settings.calibrationPattern == Settings::APRILTAG && settings.patternPath.length() == 0){
        cout << "for calibration with APRILTAGS need pattern file, use --help for more information" <<endl;
        return 1;
    }
    if(settings.calibrationPattern != Settings::FROM_FILES) {
        // read pictures from path
        cout << "read pictures: " << endl;
        DIR *dir;
        dirpath = dirpath + "/";
        struct dirent *dirzeiger;
        int counter = 0;
        if ((dir = opendir(dirpath.c_str())) != NULL) {
            while ((dirzeiger = readdir(dir)) != NULL) {
                stringstream pic;
                pic << dirpath << (*dirzeiger).d_name;
                string picstring = pic.str();
                string defaultdendung = "";
                std::transform(picstring.begin(), picstring.end(), picstring.begin(), ::tolower);
                size_t found = string::npos;
                if (dateiendung.length() != 0) {
                    std::transform(dateiendung.begin(), dateiendung.end(), dateiendung.begin(), ::tolower);
                    found = picstring.find(dateiendung);
                } else {
                    defaultdendung = ".jpg";
                    found = picstring.find(defaultdendung);
                    if (found == string::npos) {
                        defaultdendung = ".jpeg";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".jpe";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".png";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".tiff";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".tif";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".ppm";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".pgm";
                        found = picstring.find(defaultdendung);
                    }
                    if (found == string::npos) {
                        defaultdendung = ".bmp";
                        found = picstring.find(defaultdendung);
                    }
                }
                size_t endung = picstring.find("detections");
                if (found != string::npos && endung == string::npos) {
                    settings.picturePath.push_back(pic.str());
                    cout << pic.str() << endl;
                    counter++;
                }
            }
            cout << "\n" << counter << " pictures read" << endl;
        }
        if (counter == 0) {
            cout << "no pictures found! pleas check path, use --help for more information" << endl;
            return 1;
        }
    }else{
        cout << "read pictures: " << endl;
        DIR *dir;
        dirpath = dirpath + "/";
        struct dirent *dirzeiger;
        int counter = 0;
        if ((dir = opendir(dirpath.c_str())) != NULL) {
            while ((dirzeiger = readdir(dir)) != NULL) {
                stringstream pic;
                pic << dirpath << (*dirzeiger).d_name;
                string picstring = pic.str();
                string defaultdendung = ".correspondence";
                std::transform(picstring.begin(), picstring.end(), picstring.begin(), ::tolower);
                size_t found = string::npos;
                found = picstring.find(defaultdendung);
                if (found != string::npos) {
                    settings.picturePath.push_back(pic.str());
                    cout << pic.str() << endl;
                    counter++;
                }
            }
            cout << "\n" << counter << " files read" << endl;
        }
        if(counter == 0){
            cout << "no files found! pleas check path, use --help for more information" << endl;
            return 1;
        }
    }
    if(extrinsic && estMatFile.length()== 0){
        cout << "for parameter --extrinsic nee also --initial-camera-matrix" << endl;
        return 1;
    }
    settings.estimationXML = dirpath + "/ESTIMATION"+xmlFileName;
    settings.outputFileName =dirpath + "/" + xmlFileName;

    if(estMatFile.length() != 0){
        settings.estFromInput = true;
        cv::FileStorage fs(estMatFile, cv::FileStorage::READ);
        fs["Camera_Matrix"] >> settings.estCameraMatrix;
        fs["Distortion_Coefficients"] >> settings.estDistCoeff;
        std::cout << "camera matrix: " << std::endl << settings.estCameraMatrix << std::endl;
    }else{
        settings.estFromInput = false;
    }

    cout << "start detection" << endl;
    CalibrationToolbox calTool = CalibrationToolbox(settings);
    if(extrinsic){
        cout << "start extrinsic estimation" << endl;
        calTool.computeExtrinsic();
    } else if(!onlydetect) {
        cout << "start calibration" << endl;
        calTool.calibrate();
    }

    return 0;
}


