//
// Created by Joschka van der Lucht on 06.03.18.
//

#include <boost/program_options.hpp>

namespace po = boost::program_options;

void main(int argc, const char * argv[]){

    std::string inputPath="";
    std::string outputPath="";
    std::string patternType="";
    std::string tagfam="";
    int x = 0;
    int y = 0;
    int t = 4;
    //Declare supported options
    po::variables_map vm;
    po::options_description desc ("Allowed options");
    try {
        desc.add_options()
                ("help,h", "produce help message")
                ("input,i", po::value<std::string>(&inputPath), "input path with pictures")
                ("output,o", po::value<std::string>(&outputPath)->default_value(inputPath), "output path, by default is "
                "same as input")
                ("patterntype,p", po::value<std::string>(&patternType)->default_value("apriltag"),"set the type of used"
                        " pattern, default 'apriltag', allowed 'apriltag' or 'chessboard'")
                ("tagfamily,f", po::value<std::string>(&tagfam)->default_value("tag36h11"), "set AprilTag family, "
                        "default 'tag36h11'")
                ("board-x,x", po::value<int>(&x), "chessboard bord size x value")
                ("board-y,y", po::value<int>(&y), "chessboard bord size y value")
                ("threads,t", po::value<int>(&t)->default_value(4), "set tread count for AprilTag detection")
                ;
    }


}