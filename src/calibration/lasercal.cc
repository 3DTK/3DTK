#include <iostream>
#include <boost/program_options.hpp>
#include <dirent.h>
#include "calibration/Settings.h"
#include "calibration/LaserTools.h"

namespace po = boost::program_options;

int main(int argc, const char *argv[]) {
    std::cout << "Laser to camera calibration tool" << std::endl;
    Settings settings;
    settings.calibrationPattern = Settings::APRILTAG;
    settings.tagFamily = "tag36h11";
    settings.threads = 4;
    settings.blur = 0.8;
    settings.refine_decodes = true;
    settings.refine_edges = true;
    settings.refine_pose = true;
    settings.pattern = Settings::APRIL_3D;
    settings.decimate = 1;
    settings.debug = false;
    settings.patternPath = "";
    settings.debug = false;
    std::string dirpath = "";
    string estMatFile = "";
    int counter = 0;

    po::variables_map vm;
    po::options_description desc("Allowed options");
    try {
        desc.add_options()
                ("help,h", "produce help message")

                ("path-pictures,P", po::value<std::string>(&dirpath), "set path to pictures")

                ("path-pattern,S", po::value<string>(&settings.patternPath),
                 "set path to file with pattern coordinates")

                ("initial-camera-matrix,M", po::value<string>(&estMatFile), "xml-file with initial camera matrix");

        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            cout << desc << "\n";
            return 1;
        }
    } catch (const boost::program_options::required_option &e) {
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 1;
        } else {
            std::cout << desc << std::endl;
            throw e;
        }
    }

    // read pics
    dirpath = dirpath + "/";
    cout << "read pictures: " << endl;
    DIR *dir;
    struct dirent *dirzeiger;
    if ((dir = opendir(dirpath.c_str())) != NULL) {
        while ((dirzeiger = readdir(dir)) != NULL) {
            stringstream pic;
            pic << dirpath << (*dirzeiger).d_name;
            string picstring = pic.str();
            string defaultdendung = "";
            std::transform(picstring.begin(), picstring.end(), picstring.begin(), ::tolower);
            size_t found = string::npos;
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
    if(estMatFile.length() != 0){
        settings.estFromInput = true;
        cv::FileStorage fs(estMatFile, cv::FileStorage::READ);
        fs["Camera_Matrix"] >> settings.estCameraMatrix;
        fs["Distortion_Coefficients"] >> settings.estDistCoeff;
        std::cout << "camera matrix: " << std::endl << settings.estCameraMatrix << std::endl;
    }else{
        cout << "camera matrix requiered";
        return 1;
    }

    /////// FIXME feste Werte!
    vector<vector<int>> ids;
    vector<int> planeOne;
    vector<int> planeTwo;
    planeOne.push_back(204);
    planeOne.push_back(206);
    planeOne.push_back(208);
    planeOne.push_back(210);

    planeTwo.push_back(205);
    planeTwo.push_back(207);
    planeTwo.push_back(209);
    planeTwo.push_back(211);

    ids.push_back(planeOne);
    ids.push_back(planeTwo);
    ////////////

    LaserTools laserTools = LaserTools(settings);
    laserTools.detectTags(ids);
    laserTools.computeExtrinsic();
    //laserTools.printMatrix();
    laserTools.calculateEquation();

    return 0;
}