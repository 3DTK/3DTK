//
// Created by Joschka van der Lucht on 06.03.18.
//

#include <boost/program_options.hpp>
#include "calibration/Detector.h"

namespace po = boost::program_options;

int main(int argc, const char * argv[]){

    std::string inputPath="";
    std::string outputPath="";
    std::string patternType="";
    std::string tagfam="";
    int x = 0;
    int y = 0;
    int t = 4;
    //Declare supported options
    po::variables_map vm;
    po::options_description generic("Generic options");
    po::options_description input("Input options");
    po::options_description output("Output options");
    po::options_description apriltag("AprilTag options");
    po::options_description chessboard("Chessboard options");
    po::options_description hidden("Hidden options");

    generic.add_options()
            ("help,h", "produce help message")
            ;
    hidden.add_options()
            ("input,i", po::value<std::string>(&inputPath)->required(), "input path with pictures")
            ;
    input.add_options()
            ("patterntype,p", po::value<std::string>(&patternType)->default_value("apriltag"),"set the type of used"
                    " pattern, default 'apriltag', allowed 'apriltag' or 'chessboard'")
            ;
    output.add_options()
            ("output,o", po::value<std::string>(&outputPath), "output path, by default is "
                    "same as input")
            ;
    apriltag.add_options()
            ("tagfamily,f", po::value<std::string>(&tagfam)->default_value("tag36h11"), "set AprilTag family, "
                    "default 'tag36h11'")
            ("threads,t", po::value<int>(&t)->default_value(4), "set tread count for AprilTag detection")
            ;
    chessboard.add_options()
            ("board-x,x", po::value<int>(&x), "chessboard bord size x value")
            ("board-y,y", po::value<int>(&y), "chessboard bord size y value")
            ;

    po::options_description all;
    all.add(generic).add(hidden).add(input).add(output).add(apriltag).add(chessboard);

    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input).add(output).add(apriltag).add(chessboard);

    po::positional_options_description pd;
    pd.add("input", 1);

    po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vm);

    try {
        po::notify(vm);
    }catch (const po::required_option &e){
        if (vm.count("help")){
            std::cout << cmdline_options << std::endl;
            return 1;
        }else{
            std::cout << all << std::endl;
            throw e;
        }
    }
    if (vm.count("help")) {
        std::cout << all << "\n";
        return 1;
    }
    if(vm.count("patterntype")){
        if(vm["patterntype"].as<std::string>().compare("apriltag") == 0){
            patternType = "apriltag";
        }else if(vm["patterntype"].as<std::string>().compare("chessboard") == 0){
            patternType = "chessboard";
            if(x == 0 || y == 0){
                std::cerr<< "Need board size x,y > 0 for chessboard!" << std::endl;
                std::cout << cmdline_options << std::endl;
                return 1;
            }
        }else{
            std::cerr << "Patterntype is invalid! use -- hlep for more information" << std::endl;
            std::cout << cmdline_options << std::endl;
            return 1;
        }
    }
    if(vm.count("threads") && t<1){
        std::cerr << "Count of threads must >0! use -- hlep for more information" << std::endl;
        std::cout << cmdline_options << std::endl;
        return 1;
    }


}