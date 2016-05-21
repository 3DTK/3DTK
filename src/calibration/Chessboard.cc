//
// Created by Joschka van der Lucht on 07.05.16.
//

#include <sstream>
#include <fstream>
#include <iostream>
#include "calibration/Chessboard.h"

using namespace std;
using namespace cv;

vector<Point3f> Chessboard::readPoint3fChessboardFromFile(string path){
    vector<Point3f> points;
    ifstream file;
    file.open(path, ios_base::in);
    if(!file){
        cerr << "can't read: " << path << endl;
    }
    else{
        string line;
        bool header = true;
        while (getline(file, line)){
            if(!header){//neue ID
                char* fEnd;
                float x,y,z;
                x = strtof (line.c_str(), &fEnd);
                y = strtof (fEnd, &fEnd);
                z = strtof (fEnd, NULL);
                points.push_back(cv::Point3f(x,y,z));
            }else if(header && line.find("#CESSBOARD")!=std::string::npos) {
                //HEADER
                header = false;
            }else{
                cout << "invalid document!" << endl;
                return points;
            }
            line = "";
        }
    }
    return points;
}

vector<Point2f> Chessboard::readPoint2fChessboardFromFile(string path){
    vector<Point2f> points;
    ifstream file;
    file.open(path, ios_base::in);
    if(!file){
        cerr << "can't read: " << path << endl;
    }
    else{
        string line;
        bool header = true;
        while (getline(file, line)){
            if(!header){//neue ID
                char* fEnd;
                float x,y,z;
                x = strtof (line.c_str(), &fEnd);
                y = strtof (fEnd, NULL);
                points.push_back(cv::Point2f(x,y));
            } else if(header && line.find("#CHESSBOARD")!=std::string::npos) {
                //HEADER
                header = false;
            } else{
                cout << "invalid document!" << endl;
                return points;
            }
            line = "";
        }
    }
    return points;
}

string Chessboard::toString(vector<cv::Point2f> points){
    stringstream pointsstream;
    for(Point2f point2f: points){
        pointsstream << point2f.x << " " << point2f.y << endl;
    }
    return pointsstream.str();
}

string Chessboard::toString(vector<cv::Point3f> points){
    stringstream pointsstream;
    for(Point3f point3f: points){
        pointsstream << point3f.x << " " << point3f.y << " "<< point3f.z << endl;
    }
    return pointsstream.str();
}
