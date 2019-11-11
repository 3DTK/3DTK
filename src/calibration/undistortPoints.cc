#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION > 2
#include <opencv2/calib3d.hpp>
#else
#include <opencv2/calib3d/calib3d.hpp>
#endif
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;

Point2f undistortPoint(const Point2f& point, const Mat& _cameraMatrix, const Mat& _distCoeffs)
{
    vector<Point2f> in;
    in.push_back(point);
    vector<Point2f> out(1);

    undistortPoints(Mat(in), Mat(out), _cameraMatrix, _distCoeffs);

    return Point2f(_cameraMatrix.at<double>(0, 0) * out.at(0).x + _cameraMatrix.at<double>(0, 2), _cameraMatrix.at<double>(1, 1) * out.at(0).y + _cameraMatrix.at<double>(1, 2));
}

int main (int argc, char* argv[])
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("input,i", boost::program_options::value<string>()->required(), "input file with points")
            ("calibration,c", boost::program_options::value<string>()->required(), "calibration file")
            ("output,o", boost::program_options::value<string>()->required(), "output file with points")
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

    string calibrationFile(vm["calibration"].as<string>());

    FileStorage fs(calibrationFile, FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;

    cout << "camera matrix: " << endl << cameraMatrix << endl;
    cout << "distortion coefficients: " << endl << distCoeffs << endl;

    string inputFile(vm["input"].as<string>());
    string outputFile(vm["output"].as<string>());

    cout << "reading input file: " << inputFile << endl;

    vector<Point2f> points;

    ifstream in(inputFile);
    string line;
    while(getline(in, line)) {
        istringstream iss(line);

        float x, y;
        iss >> x >> y;

        points.push_back(Point2f(x, y));
    }
    in.close();

    cout << "writing output file: " << outputFile << endl;

    ofstream out(outputFile);
    for(Point2f p : points) {
        Point2f u = undistortPoint(p, cameraMatrix, distCoeffs);

        out << u.x << " " << u.y << endl;
    }
    out.close();

    return 0;
}
