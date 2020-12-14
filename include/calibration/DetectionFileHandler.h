//
// Created by Joschka van der Lucht on 10.12.20.
//

#ifndef INC_3DTK_DETECTIONFILEHANDLER_H
#define INC_3DTK_DETECTIONFILEHANDLER_H
#include <opencv2/opencv.hpp>

#define DETECTION_FILE_VERSION "V2.0"

namespace calibration{
    class DetectionFileHandler{
    public:
        std::vector<std::map<std::string, cv::Point3f>>& getDetections(){return _detections;};
        void readDetectionsFromFile(std::string &path);
        void readDetectionsFromDirectory(std::string &path);
        static void matchImageToObjectPoints(std::map<std::string, cv::Point3f>& imageIdPoints,
                                         std::map<std::string, cv::Point3f>& objectIdPoints,
                                         std::vector<cv::Point3f>&imagePoints,
                                         std::vector<cv::Point3f>&objectPoints,
                                         std::vector<cv::Point3f>&imageEstPoints,
                                         std::vector<cv::Point3f>&objectEstPoints);

        static void writeDetectionsToFile(const std::string &path, std::map<std::string, cv::Point3f>& points);
        static void writeDetectionsToFile(const std::string &path, std::map<std::string, cv::Point3f>& points,
                                      std::string *comment);

    private:
        std::vector<std::map<std::string, cv::Point3f>> _detections;
    };
}
#endif //INC_3DTK_DETECTIONFILEHANDLER_H
