#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

int main (int argc, char* argv[])
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("calibration,c", boost::program_options::value<std::string>()->required(), "calibration file")
            ("output,o", boost::program_options::value<std::string>()->required(), "output file with plane equations in the form ax + by + cz = 1")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    boost::program_options::notify(vm);

    Mat cameraMatrix;
    Mat distCoeffs;
    Mat extrinsics;

    std::string calibrationFile(vm["calibration"].as<std::string>());

    FileStorage fs(calibrationFile, FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs["Extrinsic_Parameters"] >> extrinsics;

    std::cout << "camera matrix: " << std::endl << cameraMatrix << std::endl;
    std::cout << "distortion coefficients: " << std::endl << distCoeffs << std::endl;
    std::cout << std::endl;

    std::string outputFile(vm["output"].as<std::string>());

    std::cout << "writing output: " << outputFile << std::endl << std::endl;

    std::ofstream out(outputFile);

    for (int i = 0; i < extrinsics.rows; i++) {
        Mat_<double> rvec(3,1);
        for (int j = 0; j < 3; j++) { rvec.at<double>(j, 0) = extrinsics.at<double>(i, j); }

        Mat_<double> rotMatrix;
        Rodrigues(rvec, rotMatrix);
        Mat p1 = rotMatrix * Mat(Vec3d(1, 0, 0));
        Mat p2 = rotMatrix * Mat(Vec3d(0, 1, 0));

        Mat_<double> tvec(3,1);
        for (int j = 0; j < 3; j++) { tvec.at<double>(j, 0) = extrinsics.at<double>(i, 3 + j); }

        Mat normal = p1.cross(p2);
        double distance = -((normal.at<double>(0, 0) * tvec.at<double>(0, 0)) + (normal.at<double>(1, 0) * tvec.at<double>(1, 0)) + (normal.at<double>(2, 0) * tvec.at<double>(2, 0)));

        std::cout << "extrinsic parameters nr. " << i << ":" << std::endl;

        std::cout << "normal: " << std::endl << normal << std::endl;
        std::cout << "distance: " << distance << std::endl;

        Mat planeEquation = -normal / distance;
        std::cout << "ax + by + cz = 1: " << std::endl << planeEquation << std::endl << std::endl;

        out << planeEquation.at<double>(0, 0) << " " << planeEquation.at<double>(1, 0) << " " << planeEquation.at<double>(2, 0) << std::endl;
    }

    out.close();

    return 0;
}
