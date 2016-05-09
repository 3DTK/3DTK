//
// Created by Joschka van der Lucht on 22.03.16.
//


#include "calibration/AprilTag.h"
using namespace AprilTag;
using namespace std;

std::vector<AprilTag3f> AprilTag::createAprilTag3fFromFile(std::string path, Settings &s){
    vector<AprilTag3f> tagVector;
    ifstream patternDatei;
    patternDatei.open(path, ios_base::in);
    if(!patternDatei){
        cerr << "can't read: " << path << endl;
    }
    else{
        string line;
        bool header = true;
        while (getline(patternDatei, line)){
            if(!header && line.find('#')!=std::string::npos){
                //neue ID
                int id = stoi(line.substr(1));
                vector<cv::Point3f> points;
                //die vier Eckpunkte zu der ID
                for(int i=0; i < 4; i++){
                    line = "";
                    if(getline(patternDatei, line)){
                        char* fEnd;
                        float x,y,z;
                        x = strtof (line.c_str(), &fEnd);
                        y = strtof (fEnd, &fEnd);
                        z = strtof (fEnd, NULL);
                        points.push_back(cv::Point3f(x,y,z));
                    }
                    else{
                        cout << "invalid document!" << endl;
                        return tagVector;
                    }
                }
                //fertiger AprilTag kommt in vector
                tagVector.push_back(AprilTag3f(id, points[0],points[1],points[2],points[3]));
            }else if(header && line.find("#APRIL_3D")!=std::string::npos) {
                header = false;
                s.pattern = Settings::APRIL_3D;
                //Hier wird die Anfangszeile mit "#APRIL_3D" gesucht und alles vorher ignoriert
                //continue;
            } else if(header && line.find("#APRIL_2D")!=std::string::npos) {
            header = false;
            s.pattern = Settings::APRIL_2D;
            //Hier wird die Anfangszeile mit "#APRIL_2D" gesucht und alles vorher ignoriert
            //continue;
            } else if(header && line.find("#estimation IDs")!=std::string::npos) {
                getline(patternDatei, line);
                while (line.find("#END estimation IDs")== std::string::npos){
                    int id = stoi(line.substr(1));
                    s.estIDs.push_back(id);
                    getline(patternDatei, line);
                }
            }
            line = "";
        }
    }
    return tagVector;
}

std::vector<AprilTag2f> AprilTag::createAprilTag2fFromFile(std::string path){
    vector<AprilTag2f> tagVector;
    ifstream patternDatei;
    patternDatei.open(path, ios_base::in);
    if(!patternDatei){
        cerr << "can't read: " << path << endl;
    }
    else{
        string line;
        bool header = true;
        while (getline(patternDatei, line)){
            if(!header && line.find('#')!=std::string::npos){
                //neue ID
                int id = stoi(line.substr(1));
                vector<cv::Point2f> points;
                //die vier Eckpunkte zu der ID
                for(int i=0; i < 4; i++){
                    line = "";
                    if(getline(patternDatei, line)){
                        char* fEnd;
                        float x,y;
                        x = strtof (line.c_str(), &fEnd);
                        y = strtof (fEnd, NULL);
                        points.push_back(cv::Point2f(x,y));
                    }
                    else{
                        cout << "invalid document!" << endl;
                        return tagVector;
                    }
                }
                //fertiger AprilTag kommt in vector
                tagVector.push_back(AprilTag2f(id, points[0],points[1],points[2],points[3]));
            }else if(header && line.find("#APRIL_2D")!=std::string::npos) {
                header = false;
                //Hier wird die Anfangszeile mit "#APRIL_3D" gesucht und alles vorher ignoriert
                //continue;
            }
            line = "";
        }
    }
    return tagVector;
}

AprilTag3f::AprilTag3f(){
    this->id = 0;
    this->point1 = cv::Point3f(0,0,0);
    this->point2 = cv::Point3f(0,0,0);
    this->point3 = cv::Point3f(0,0,0);
    this->point4 = cv::Point3f(0,0,0);
}

AprilTag3f::AprilTag3f(int id){
    this->id = id;
    this->point1 = cv::Point3f(0,0,0);
    this->point2 = cv::Point3f(0,0,0);
    this->point3 = cv::Point3f(0,0,0);
    this->point4 = cv::Point3f(0,0,0);
}

AprilTag3f::AprilTag3f(int id, cv::Point3f leftup, cv::Point3f leftdown, cv::Point3f rightdown, cv::Point3f rightup){
    this->id = id;
    this->point1 = leftup;
    this->point2 = leftdown;
    this->point3 = rightdown;
    this->point4 = rightup;
}

AprilTag3f::~AprilTag3f(){

}

int AprilTag3f::compair(AprilTag2f aprilTag2f){
    if(this->id == aprilTag2f.id) return 0;
    if(this->id > aprilTag2f.id) return 1;
    if(this->id < aprilTag2f.id) return -1;
    return 0;
}

int AprilTag3f::compair(AprilTag3f aprilTag3f){
    if(this->id == aprilTag3f.id && this->point1 == aprilTag3f.point1 && this->point2 == aprilTag3f.point2 && this->point3 == aprilTag3f.point3 && this->point4 == aprilTag3f.point4) return 0;
    if(this->id != aprilTag3f.id) return -1;
    return 0;
}

std::string AprilTag3f::toString(){
    std::stringstream s;
    s << "#" << this->id << "\n" << this->point1.x << " " << point1.y << " " << point1.z << "\n"<< this->point2.x << " " << point2.y << " " << point2.z << "\n"<< this->point3.x << " " << point3.y << " " << point3.z << "\n"<< this->point4.x << " " << point4.y << " " << point4.z << "\n";
    return s.str();
}


AprilTag2f::AprilTag2f(){
    this->id = 0;
    this->point1 = cv::Point2f(0,0);
    this->point2 = cv::Point2f(0,0);
    this->point3 = cv::Point2f(0,0);
    this->point4 = cv::Point2f(0,0);
}

AprilTag2f::AprilTag2f(int id){
    this->id = id;
    this->point1 = cv::Point2f(0,0);
    this->point2 = cv::Point2f(0,0);
    this->point3 = cv::Point2f(0,0);
    this->point4 = cv::Point2f(0,0);
}

AprilTag2f::AprilTag2f(int id, cv::Point2f leftup, cv::Point2f leftdown, cv::Point2f rightdown, cv::Point2f rightup){
    this->id = id;
    this->point1 = leftup;
    this->point2 = leftdown;
    this->point3 = rightdown;
    this->point4 = rightup;
}

AprilTag2f::~AprilTag2f(){

}


int AprilTag2f::compair(AprilTag2f aprilTag2f){
    if(this->id == aprilTag2f.id && this->point1 == aprilTag2f.point1 && this->point2 == aprilTag2f.point2 && this->point3 == aprilTag2f.point3 && this->point4 == aprilTag2f.point4) return 0;
    if(this->id != aprilTag2f.id) return -1;
    return 0;
}

int AprilTag2f::compair(AprilTag3f aprilTag3f){
    if(this->id == aprilTag3f.id) return 0;
    if(this->id > aprilTag3f.id) return 1;
    if(this->id < aprilTag3f.id) return -1;
    return 0;
}

std::string AprilTag2f::toString(){
    std::stringstream s;
    s << "#" << this->id << "\n" << this->point1.x << " " << point1.y <<"\n"<< this->point2.x << " " << point2.y << "\n"<< this->point3.x << " " << point3.y <<"\n"<< this->point4.x << " " << point4.y <<"\n";
    return s.str();
}
