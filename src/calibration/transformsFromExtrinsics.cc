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
using namespace std;

int main (int argc, char* argv[])
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("calibration,c", boost::program_options::value<string>()->required(), "calibration file")
            ("output,o", boost::program_options::value<string>()->required(), "output file with plane equations in the form ax + by + cz = 1")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return EXIT_SUCCESS;
    }

    boost::program_options::notify(vm);

    Mat cameraMatrix;
    Mat distCoeffs;
    Mat extrinsics;

    string calibrationFile(vm["calibration"].as<string>());

    FileStorage fs(calibrationFile, FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs["Extrinsic_Parameters"] >> extrinsics;

    cout << "camera matrix: " << endl << cameraMatrix << endl;
    cout << "distortion coefficients: " << endl << distCoeffs << endl;
    cout << endl;

    string outputFile(vm["output"].as<string>());

    cout << "writing output: " << outputFile << endl << endl;

    ofstream out(outputFile);

    for (int i = 0; i < extrinsics.rows; i++) {
        Mat_<double> rvec(3,1);
        for (int j = 0; j < 3; j++) { rvec.at<double>(j, 0) = extrinsics.at<double>(i, j); }

        Mat_<double> rotMatrix;
        Rodrigues(rvec, rotMatrix);
        Mat p1 = rotMatrix * Mat(Vec3d(1, 0, 0));
        Mat p2 = rotMatrix * Mat(Vec3d(0, 1, 0));

        Mat_<double> tvec(3,1);
        for (int j = 0; j < 3; j++) { tvec.at<double>(j, 0) = extrinsics.at<double>(i, 3 + j); }

        Mat_<double> transform = Mat_<double>::zeros(4,4);
        for (int m = 0; m < 3; m++) {
            for (int n = 0; n < 3; n++) {
                transform.at<double>(m, n) = rotMatrix.at<double>(m, n);
            }
            transform.at<double>(m, 3) = tvec.at<double>(m, 0);
        }
        transform.at<double>(3, 3) = 1;

        cout << "extrinsic parameters nr. " << i << ":" << endl;

        cout << "transform: " << endl << transform << endl;
        cout << "inverse transform: " << endl << transform.inv() << endl;

        out << i;
        for (int m = 0; m < 4; m++) {
            for (int n = 0; n < 4; n++) {
                out << " " << transform.at<double>(m, n);
            }
        }
        out << endl;
    }

    out.close();

    return 0;
}
