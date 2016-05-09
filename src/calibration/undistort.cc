#include <iostream>
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main (int argc, char* argv[])
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("input,i", boost::program_options::value<std::string>()->required(), "input file")
            ("calibration,c", boost::program_options::value<std::string>()->required(), "calibration file")
            ("output,o", boost::program_options::value<std::string>()->required(), "output file")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    boost::program_options::notify(vm);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    std::string calibrationFile(vm["calibration"].as<std::string>());

    cv::FileStorage fs(calibrationFile, cv::FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;

    std::cout << "camera matrix: " << std::endl << cameraMatrix << std::endl;
    std::cout << "distortion coefficients: " << std::endl << distCoeffs << std::endl;

    std::string inputFile(vm["input"].as<std::string>());
    std::string outputFile(vm["output"].as<std::string>());

    std::cout << "Reading: " << inputFile << std::endl;

    cv::Mat colorImage = cv::imread(inputFile);
    cv::Mat undistortedImage;
    cv::undistort(colorImage, undistortedImage, cameraMatrix, distCoeffs);

    std::cout << "Writing: " << outputFile << std::endl;

    cv::imwrite(outputFile, undistortedImage);

    return 0;
}
