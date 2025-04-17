#pragma once

#include "calibration/Detector.h"

#include "cctag/CCTagMarkersBank.hpp"
#include "cctag/Params.hpp"

namespace calibration {

class CCTagDetector : public Detector {
public:
    CCTagDetector(size_t nCrowns);
    ~CCTagDetector() = default;
    bool detect(const cv::Mat& image) noexcept(false);
    void writeDetectionsToFile(const std::string& path);
    void readDetectionsFromFile(const std::string& path);
private:
    cctag::Parameters _params;
    cctag::CCTagMarkersBank _bank;
};

}
