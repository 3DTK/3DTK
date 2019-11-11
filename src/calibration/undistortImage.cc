#include <iostream>
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION > 2
#include <opencv2/calib3d.hpp>
#else
#include <opencv2/calib3d/calib3d.hpp>
#endif

using namespace std;
using namespace cv;

int main (int argc, char* argv[])
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("input,i", boost::program_options::value<string>()->required(), "input image")
            ("calibration,c", boost::program_options::value<string>()->required(), "calibration file")
            ("output,o", boost::program_options::value<string>()->required(), "output image")
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

    cout << "reading input image: " << inputFile << endl;

    Mat colorImage = imread(inputFile);
    Mat undistortedImage;
    undistort(colorImage, undistortedImage, cameraMatrix, distCoeffs);

    cout << "writing output image: " << outputFile << endl;

    imwrite(outputFile, undistortedImage);

    return 0;
}
