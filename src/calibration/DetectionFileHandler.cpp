//
// Created by Joschka van der Lucht on 10.12.20.
//
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "calibration/DetectionFileHandler.h"

using namespace std;
using namespace cv;

namespace calibration{
    void DetectionFileHandler::readDetectionsFromFile(string &path){
        ifstream f;
        map<string, Point3f> detection;
        f.open(path, ios_base::in);
        if(!f){
            cerr << "can't read: " << path << endl;
            return;
        }else{
            string line;
            int linecounter = 0;
            bool versionCheck = false;
            while (getline(f, line)){
                linecounter ++;
                if(line.find('#') != string::npos){
                    if(line.find(DETECTION_FILE_VERSION) != string::npos){
                        versionCheck = true;
                    }
                }else if(!versionCheck){
                    cerr << "unsupported file version" << endl;
                    return;
                }else{
                    //read id and x y z
                    vector<string> splitedLine;
                    boost::algorithm::split(splitedLine, line, boost::is_any_of("\t "), boost::token_compress_on);
                    if(splitedLine.size() != 4){
                        cerr << "read detections failed at line " << linecounter << endl;
                        return;
                    }
                    string id = splitedLine[0];
                    float x = stof(splitedLine[1]);
                    float y = stof(splitedLine[2]);
                    float z = stof(splitedLine[3]);
                    detection.insert(pair<string, Point3f>(id, Point3f(x,y,z)));
                }
            }

        }
        _detections.push_back(detection);
    }

    void DetectionFileHandler::readDetectionsFromDirectory(string &path){
        boost::filesystem::path boostPath(path);
        vector<string>fileList;
        vector<map<string, Point3f>> detections;

        if (!boost::filesystem::exists(boostPath) || !boost::filesystem::is_directory(boostPath)) {
            return;
        }
        boost::filesystem::directory_iterator it(boostPath);
        boost::filesystem::directory_iterator endit;

        while (it != endit) {
            if (boost::filesystem::is_regular_file(*it) && boost::iequals(it->path().extension().string(), ".detections")) {
                fileList.push_back(boost::filesystem::canonical(it->path()).string());
            }
            ++it;
        }
        for(string f : fileList){
            readDetectionsFromFile(f);
        }
    }

    void DetectionFileHandler::matchImageToObjectPoints(map<string, Point3f>& imageIdPoints,
                                         map<string, Point3f>& objectIdPoints,
                                            vector<Point3f>&imagePoints,
                                            vector<Point3f>&objectPoints,
                                            vector<Point3f>&imageEstPoints,
                                            vector<Point3f>&objectEstPoints){
        for(pair<string, Point3f> p : objectIdPoints){
            string id = p.first.substr(p.first.size() - 3); //Object ID without "Est"
            auto imageIdPointPair = imageIdPoints.find(id);
            if(imageIdPointPair != imageIdPoints.end()){
                if(imageIdPointPair->first.find("Est") != string::npos){
                    objectEstPoints.push_back(p.second);
                    imageEstPoints.push_back(imageIdPointPair->second);
                }
                objectPoints.push_back(p.second);
                imagePoints.push_back(imageIdPointPair->second);
            }
        }
    }

    void DetectionFileHandler::writeDetectionsToFile(string &path, map<string, Point3f>& points,
                                      string *comment){
        fstream f;
        f.open(path, ios::out);
        f << "#Detection file" << endl;
        f << "#" << DETECTION_FILE_VERSION <<  endl;
        f << "#created by 3tdk" << endl;
        if(comment != nullptr && !comment->empty()){
            f << "#" << &comment << endl;
        }
        f << "#ID x y z" << endl;
        for(tuple<string, Point3f> t : points){
            Point3f p = get<1>(t);
            f << get<0>(t) << " " << p.x << " " << p.y << " " << p.x << endl;
        }
        f.close();
    }

    void DetectionFileHandler::writeDetectionsToFile(string &path, map<string, Point3f>& points){
        writeDetectionsToFile(path, points, nullptr);
    }
}