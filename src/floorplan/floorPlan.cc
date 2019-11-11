/*
 * floorPlan.cc
 *
 *  Created on: Apr 29, 2013
 *      Author: rdumitru
 */

//==============================================================================
//  Includes.
//==============================================================================
#include "floorplan/floorPlan.h"

// User includes.
#include <floorplan/util.h>
#include <floorplan/rotation3d.h>
#include <floorplan/point3d.h>

// OpenCV includes.
#include <opencv2/opencv.hpp>

// C++ includes.
#include <iostream>
#include <stdexcept>
using namespace std;

//==============================================================================
//  Global variables.
//==============================================================================
extern bool quiet;

//==============================================================================
//  Static field initialization.
//==============================================================================
const double floorplan::FloorPlan::STD_DEV_MUL          = 2.0;
const double floorplan::FloorPlan::ANGLE_RES            = 0.1;

const double floorplan::FloorPlan::HOUGH_RHO            = 0.1;
const double floorplan::FloorPlan::HOUGH_THETA          = CV_PI / 720.0;
const int    floorplan::FloorPlan::HOUGH_THRESH         = 10;
const double floorplan::FloorPlan::HOUGH_MIN_LINE_LEN   = 15.0;
// TODO make these scale with the resolution of the image

const double floorplan::FloorPlan::MAX_ANGLE            = 180.0;
const double floorplan::FloorPlan::SNAP_ANGLE_THRESH    = 5.0;

//==============================================================================
//  Class implementation.
//==============================================================================
vector<cv::Vec4i> floorplan::FloorPlan::extractWallLines() {
    if (!quiet) cout << endl << "== Extracting wall lines..." << endl;

    vector<cv::Vec4i> lines;
    cv::HoughLinesP(this->thresh, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESH, HOUGH_MIN_LINE_LEN / this->scale);

    cv::Mat wallImg(this->thresh.rows, this->thresh.cols, CV_8UC3);
    wallImg = cv::Scalar(0);

    for (size_t it = 0; it < lines.size(); ++it) {
        cv::Vec4i ln = lines[it];
        cv::line(wallImg, cv::Point(ln[0], ln[1]), cv::Point(ln[2], ln[3]), cv::Scalar(0, 0, 255), 1,
#if CV_MAJOR_VERSION > 2
			cv::LINE_AA
#else
			CV_AA
#endif
		);
    }
    cv::imshow("lineImg", wallImg);
    cv::waitKey();

    return lines;
}

vector<int> floorplan::FloorPlan::computeOrientationHist() {
    if (!quiet) cout << endl << "== Computing orientation histogram..." << endl;

    int nrBuckets = ceil(MAX_ANGLE / ANGLE_RES);
    vector<int> orHist(nrBuckets, 0);

    for (size_t it = 0; it < this->walls.size(); ++it) {
        double dX = (double) this->walls[it].val[2] - this->walls[it].val[0];
        double dY = (double) this->walls[it].val[3] - this->walls[it].val[1];
        double angle = normalizedDegAngle(rad2deg(atan2(dY, dX)));
        if (angle > MAX_ANGLE) angle -= MAX_ANGLE;

        // XXX or floor
        int bucket = round(angle / ANGLE_RES);
        orHist[bucket]++;
    }

    return orHist;
}

void floorplan::FloorPlan::correctWallLines() {
    if (!quiet) cout << endl << "== Correcting walls..." << endl;

    if (this->walls.empty()) {
        throw runtime_error("no walls have been detected");
    }

    vector<double> domOr;
    vector<int> orHist = this->computeOrientationHist();

//    vector<int> hist = orHist;
//    for (size_t it = 0; it < hist.size(); ++it) {
//        cout << hist[it] << endl;
//    }

    // Compute mean and standard deviation so we know when to threshold.
    cv::Scalar mean, stdDev;
    cv::meanStdDev(orHist, mean, stdDev);

    // What is above the threshold is a dominant orientation.
    size_t maxIdx = (size_t) (max_element(orHist.begin(), orHist.end()) - orHist.begin());
    size_t secondMaxIdx = (maxIdx + ((size_t) round(90.0 / ANGLE_RES))) % ((size_t) round(MAX_ANGLE / ANGLE_RES));

    int steps = 2 * round(SNAP_ANGLE_THRESH / ANGLE_RES);
    int currIdx = (secondMaxIdx + orHist.size() - (steps / 2)) % orHist.size();
    for (int i = 0; i < steps; ++i) {
        if (orHist[currIdx] > orHist[secondMaxIdx]) {
            secondMaxIdx = currIdx;
        }

        currIdx = (currIdx + 1) % orHist.size();
    }

    domOr.push_back(maxIdx * ANGLE_RES);
    domOr.push_back(secondMaxIdx * ANGLE_RES);

    if (!quiet) cout << endl << "== Dominant orientations: " << endl;
    for (size_t it = 0; it < domOr.size(); ++it) {
        cout << "\t" << domOr[it] << endl;
    }

    // Adjust the lines.
    for (size_t it = 0; it < this->walls.size(); ++it) {
        double dX = (double) this->walls[it].val[2] - this->walls[it].val[0];
        double dY = (double) this->walls[it].val[3] - this->walls[it].val[1];
        double angle = normalizedDegAngle(rad2deg(atan2(dY, dX)));
        if (angle > MAX_ANGLE) angle -= MAX_ANGLE;

        // Go through possible dominant orientations.
        int bestIdx = -1;
        double bestDelta = numeric_limits<double>::max();
        for (size_t jt = 0; jt < domOr.size(); ++jt) {
            double delta = min(fabs(angle - domOr[jt]), fabs(angle - domOr[jt] - MAX_ANGLE));
            if (delta < bestDelta) {
                bestIdx = jt;
                bestDelta = delta;
            }
        }

        if (bestDelta > SNAP_ANGLE_THRESH) {
            continue;
        }

        // Rotate the line.
        double rotAngle = deg2rad(domOr[bestIdx] - angle);

        Rotation3d rot(0.0, 0.0, rotAngle);
        Point3d pts[2];
        pts[0].x = this->walls[it].val[0];
        pts[0].y = this->walls[it].val[1];
        pts[1].x = this->walls[it].val[2];
        pts[1].y = this->walls[it].val[3];

        Point3d center = pts[0];
        center += pts[1];
        center /= 2.0;

        pts[0].rotate(center, rot);
        pts[1].rotate(center, rot);

        this->walls[it].val[0] = pts[0].x;
        this->walls[it].val[1] = pts[0].y;
        this->walls[it].val[2] = pts[1].x;
        this->walls[it].val[3] = pts[1].y;
    }

    cv::Mat wallImg(this->thresh.rows, this->thresh.cols, CV_8UC3);
    wallImg = cv::Scalar(0);

    for (size_t it = 0; it < this->walls.size(); ++it) {
        cv::Vec4i ln = this->walls[it];
        cv::line(wallImg, cv::Point(ln[0], ln[1]), cv::Point(ln[2], ln[3]), cv::Scalar(0, 0, 255), 1,
#if CV_MAJOR_VERSION > 2
			cv::LINE_AA
#else
			CV_AA
#endif
		);
    }
    cv::imshow("correctedLineImg", wallImg);
    cv::waitKey();
}

floorplan::FloorPlan::FloorPlan(const cv::Mat &grayPlan, const double &scale) {
    if (!quiet) cout << endl << "== Initializing floor plan..." << endl;

    if (grayPlan.type() != CV_8UC1) {
        throw runtime_error("image type expected to be CV_8UC1");
    }

    this->grayImg = grayPlan;
    this->scale = scale;

    // Compute mean and standard deviation.
    cv::Scalar mean, stdDev;
    cv::meanStdDev(this->grayImg, mean, stdDev);
    if (!quiet) cout << endl << "== Mean: " << mean[0] << ";\t StdDev: " << stdDev[0] <<"..." << endl;

    // Threshold the image using mean + coef * stdDev.
    cv::threshold(this->grayImg, this->thresh, mean[0] + (STD_DEV_MUL * stdDev[0]), 255, cv::THRESH_BINARY);
    cv::imshow("threshImg", this->thresh);
    cv::waitKey();

    // Detect the walls.
    this->walls = this->extractWallLines();

    // Snap walls to dominant orientations.
    this->correctWallLines();
}

floorplan::FloorPlan::FloorPlan(const FloorPlan &other) {
    this->grayImg = other.grayImg;
    this->thresh  = other.thresh;
    this->walls   = other.walls;
    this->scale   = other.scale;
}

floorplan::FloorPlan::~FloorPlan() {

}
