/**
 * @file plane3d.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 15 Apr 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/labeledPlane3d.h"
#include "model/util.h"

#include <math.h>

#include <set>
#include <limits>
#include <stdexcept>
#include <iostream>
#include <fstream>
using namespace std;

//==============================================================================
//  Static member initializations
//==============================================================================
const std::string model::LabeledPlane3d::RED            = "180 0 0";
const std::string model::LabeledPlane3d::GREEN          = "0 180 0";
const std::string model::LabeledPlane3d::BLUE           = "0 0 180";
const std::string model::LabeledPlane3d::WHITE          = "255 255 255";

const double model::LabeledPlane3d::LINE_EPSILON        = 0.2;

const double model::LabeledPlane3d::SOBEL_SCALE         = 1.0;
const double model::LabeledPlane3d::SOBEL_DELTA         = 0.0;
const int model::LabeledPlane3d::SOBEL_KERNEL           = 3;

const double model::LabeledPlane3d::CANNY_THRESH1       = 40.0;
const double model::LabeledPlane3d::CANNY_THRESH2       = 120.0;

const double model::LabeledPlane3d::HOUGH_RHO           = 1.0;
const double model::LabeledPlane3d::HOUGH_THETA         = CV_PI / 180.0;
const int model::LabeledPlane3d::HOUGH_THRESH           = 80;
const double model::LabeledPlane3d::HOUGH_MIN_LINE_LEN  = 15.0;

const double model::LabeledPlane3d::MIN_EMPTY_AREA      = 0.67;
const double model::LabeledPlane3d::MIN_TOTAL_AREA      = 0.0075;
const double model::LabeledPlane3d::MAX_TOTAL_AREA      = 0.400;

//==============================================================================
//  Class implementation
//==============================================================================
model::LabeledPlane3d::LabeledPlane3d() : Plane3d() {}

model::LabeledPlane3d::LabeledPlane3d(const Point3d& pt, const Vector3d& normal) :
        Plane3d(pt, normal)
{

}

model::LabeledPlane3d::LabeledPlane3d(const Point3d& pt, const Vector3d& normal, const std::vector<Point3d>& hull) :
        Plane3d(pt, normal, hull)
{}

model::LabeledPlane3d::LabeledPlane3d(const LabeledPlane3d& other) : Plane3d(other) {
    this->patches= other.patches;
    this->depthMap = other.depthMap;
    this->depthImg = other.depthImg;
    this->correctedDepthImg = other.correctedDepthImg;
    this->depthMapDistances = other.depthMapDistances;
}

model::LabeledPlane3d::~LabeledPlane3d() {}

model::LabeledPlane3d& model::LabeledPlane3d::operator=(const LabeledPlane3d& other) {
    if (this != &other) {
        Plane3d::operator =(other);
        this->patches= other.patches;
        this->depthMap = other.depthMap;
        this->depthImg = other.depthImg;
        this->correctedDepthImg = other.correctedDepthImg;
        this->depthMapDistances = other.depthMapDistances;
    }

    return *this;
}

void model::LabeledPlane3d::detectEdges(cv::Mat& canny, cv::Mat& hSobel, cv::Mat& vSobel, cv::Mat& combined) const {
    int imgHeight = static_cast<int>(this->depthMap.size());
    int imgWidth  = static_cast<int>(this->depthMap.front().size());

    if (imgHeight == 0 || imgWidth == 0) {
        throw runtime_error("please initialize the patches and depth map for labeled plane");
    }

    // apply Canny edge detection so we can see the lines contained in the image
    cv::Canny(this->depthImg, canny, CANNY_THRESH1, CANNY_THRESH2);

    // apply Sobel on same image so we get better defined horizontal and vertical lines
    cv::Sobel(canny, hSobel, CV_8UC1, 0, 1, SOBEL_KERNEL, SOBEL_SCALE, SOBEL_DELTA, cv::BORDER_DEFAULT);
    cv::Sobel(canny, vSobel, CV_8UC1, 1, 0, SOBEL_KERNEL, SOBEL_SCALE, SOBEL_DELTA, cv::BORDER_DEFAULT);

    // combine all of them for even better edges
    cv::addWeighted(canny, 0.5, hSobel, 0.5, 0, combined);
    cv::threshold(combined, combined, 1, MAX_IMG_VAL, CV_8UC1);
    cv::addWeighted(combined, 0.5, vSobel, 0.5, 0, combined);
    cv::threshold(combined, combined, 1, MAX_IMG_VAL, CV_8UC1);

    cv::imwrite("./img/edgeCanny.png", canny);
    cv::imwrite("./img/edgeSobelHoriz.png", hSobel);
    cv::imwrite("./img/edgeSobelVert.png", vSobel);
    cv::imwrite("./img/edgeCombined.png", combined);
}

void model::LabeledPlane3d::computeLines(std::vector<int>& verticalResult, std::vector<int>& horizontalResult) const {

    if (this->depthMap.size() != this->patches.size() ||
            this->depthMap.front().size() != this->patches.front().size())
    {
        throw runtime_error("depth map and patch matrix must match in sizes");
    }

    int imgHeight = static_cast<int>(this->depthMap.size());
    int imgWidth  = static_cast<int>(this->depthMap.front().size());

    if (imgHeight == 0 || imgWidth == 0) {
        throw runtime_error("please initialize the patches and depth map for labeled plane");
    }

    // clear the output
    verticalResult.clear();
    horizontalResult.clear();

    // compute the Canny edges normally, and using sobel on x and y
    cv::Mat canny, hSobel, vSobel, combined;
    this->detectEdges(canny, hSobel, vSobel, combined);

    // detect all the proper lines from the image
    vector<cv::Vec4i> lines;
    cv::HoughLinesP(combined, lines, HOUGH_RHO, HOUGH_THETA, HOUGH_THRESH, HOUGH_MIN_LINE_LEN);

    // convert to color image so we can draw some colored lines on it
    cv::cvtColor(combined, combined, CV_GRAY2BGR, 3);
    cv::Mat imgLines      = combined.clone();
    cv::Mat imgFinalLines = combined.clone();

    // filter out the vertical and the horizontal lines
    // XXX use sets so that the numbers are ordered automatically
    set<int> horizontalSet, verticalSet;
    int curr;

    for (vector<cv::Vec4i>::iterator it = lines.begin(); it != lines.end(); ++it) {
        int dX = it->val[2] - it->val[0];
        int dY = it->val[3] - it->val[1];

        double angle = atan2(static_cast<double>(dY), static_cast<double>(dX));

        if (((angle < +M_PI_2 + LINE_EPSILON) && (angle > +M_PI_2 - LINE_EPSILON)) ||
                ((angle < -M_PI_2 + LINE_EPSILON) && (angle > -M_PI_2 - LINE_EPSILON)))
        {
            // detect and add the vertical lines
            curr = static_cast<int>(round((it->val[2] + it->val[0]) / 2.0));
            verticalSet.insert(curr);
            cv::line(imgLines, cv::Point(curr, 0), cv::Point(curr, imgLines.rows-1), cv::Scalar(0, 0, MAX_IMG_VAL), 1);

        } else if (((angle < +0.0 + LINE_EPSILON) && (angle > +0.0 - LINE_EPSILON)) ||
                ((angle < +M_PI + LINE_EPSILON) && (angle > +M_PI - LINE_EPSILON)) ||
                ((angle < -M_PI + LINE_EPSILON) && (angle > -M_PI - LINE_EPSILON)))
        {
            // detect and add the horizontal lines
            curr = static_cast<int>(round((it->val[3] + it->val[1]) / 2.0));
            horizontalSet.insert(curr);
            cv::line(imgLines, cv::Point(0, curr), cv::Point(imgLines.cols-1, curr), cv::Scalar(MAX_IMG_VAL, 0, 0), 1);
        }
    }

    // insert the edges as lines
    horizontalSet.insert(0);
    horizontalSet.insert(imgHeight - 1);
    cv::line(imgLines, cv::Point(0, 0), cv::Point(imgLines.cols-1, 0), cv::Scalar(MAX_IMG_VAL, 0, 0), 1);
    cv::line(imgLines, cv::Point(0, imgHeight - 1), cv::Point(imgLines.cols-1, imgHeight - 1), cv::Scalar(MAX_IMG_VAL, 0, 0), 1);

    verticalSet.insert(0);
    verticalSet.insert(imgWidth - 1);
    cv::line(imgLines, cv::Point(0, 0), cv::Point(0, imgLines.rows-1), cv::Scalar(0, 0, MAX_IMG_VAL), 1);
    cv::line(imgLines, cv::Point(imgWidth - 1, 0), cv::Point(imgWidth - 1, imgLines.rows-1), cv::Scalar(0, 0, MAX_IMG_VAL), 1);

    cv::imwrite("./img/edgeLines.png", imgLines);
    if (!quiet) cout << "** Total lines: " << verticalSet.size() << " vertical, " << horizontalSet.size() << " horizontal" << endl;

    horizontalResult.assign(horizontalSet.begin(), horizontalSet.end());
    verticalResult.assign(verticalSet.begin(), verticalSet.end());
}

void model::LabeledPlane3d::computeOpeningCandidates(std::vector<CandidateOpening>& result) const {
    if (!quiet) cout << endl << "== Computing opening candidates for surface centered at " << this->pt << endl;

    // make sure the result is empty
    result.clear();

    if (this->depthMap.size() != this->patches.size() ||
            this->depthMap.front().size() != this->patches.front().size())
    {
        throw runtime_error("depth map and patch matrix must match in sizes");
    }

    int imgHeight = static_cast<int>(this->depthMap.size());
    int imgWidth  = static_cast<int>(this->depthMap.front().size());

    if (imgHeight == 0 || imgWidth == 0) {
        throw runtime_error("please initialize the patches and depth map for labeled plane");
    }

    // convert the sets to vectors
    vector<int> horizontal, vertical;
    this->computeLines(vertical, horizontal);

    // we require the total wall area to discard a few small candidates
    double wCentimetersWall = this->patches[0][0].first.distance(this->patches[0][imgWidth - 1].first);
    double hCentimetersWall = this->patches[0][0].first.distance(this->patches[imgHeight - 1][0].first);
    double wallArea = wCentimetersWall * hCentimetersWall;

    // put temporary candidates in here
    std::vector<CandidateOpening> candidates;

    // count the number of discarded small planes
    unsigned int discarded = 0;

    // there is no other way of determining all the rectangles in the image
    // u - upper, l - lower, h - horizontal, v - vertical
    for (vector<int>::iterator y1 = horizontal.begin(); y1 != horizontal.end() - 1; ++y1) {
        for (vector<int>::iterator y2 = y1 + 1; y2 != horizontal.end(); ++y2) {
            for (vector<int>::iterator x1 = vertical.begin(); x1 != vertical.end() - 1; ++x1) {
                for (vector<int>::iterator x2 = x1 + 1; x2 != vertical.end(); ++x2) {

                    // do not add really small candidates that are smaller than a certain percentage of the whole wall
                    double wCentiMeters = this->patches[0][*x2].first.distance(this->patches[0][*x1].first);
                    double hCentiMeters = this->patches[*y2][0].first.distance(this->patches[*y1][0].first);
                    double area = wCentiMeters * hCentiMeters;

                    if (area < MIN_TOTAL_AREA * wallArea || area > MAX_TOTAL_AREA * wallArea) {
                        discarded++;
                        continue;
                    }

                    // create edges for candidate opening
                    int tempEdges[4] = {*y1, *y2, *x1, *x2};
                    vector<int> edges(tempEdges, tempEdges + 4);

                    // compute the hull
                    vector<Point3d> hull;
                    hull.push_back(this->patches[*y1][*x1].first);
                    hull.push_back(this->patches[*y1][*x2].first);
                    hull.push_back(this->patches[*y2][*x2].first);
                    hull.push_back(this->patches[*y2][*x1].first);

                    // compute the normal application point
                    Point3d pt(0.0, 0.0, 0.0);
                    for (vector<Point3d>::iterator it = hull.begin(); it != hull.end(); ++it) {
                        pt += *it;
                    }
                    pt /= hull.size();

                    CandidateOpening candidate(pt, this->normal, hull, vector<double>(), edges);
                    candidate.normal = candidate.computeAverageNormal();
                    candidates.push_back(candidate);
                }
            }
        }
    }

    if (!quiet) cout << "** Total opening candidates: " << candidates.size()  << endl;
    if (!quiet) cout << "** Discarded openings relative size to wall: " << discarded << endl;
    discarded = 0;

    // determine the features of each candidate opening
    for (vector<CandidateOpening>::iterator it = candidates.begin(); it < candidates.end(); ++it) {
        int y1 = it->edges[0];
        int y2 = it->edges[1];
        int x1 = it->edges[2];
        int x2 = it->edges[3];

        if (y1 > y2 || x1 > x2) {
            throw runtime_error("upper coordinates need to be smaller than the lower ones (candidate opening)");
        }

        int w = abs(x2 - x1);
        int h = abs(y2 - y1);

        // first feature, the area of the opening
        double wCentiMeters = this->patches[0][x2].first.distance(this->patches[0][x1].first);
        double hCentiMeters = this->patches[y2][0].first.distance(this->patches[y1][0].first);
        double area = wCentiMeters * hCentiMeters;
        it->features.push_back(static_cast<double>(area));

        // second feature, w/h
        it->features.push_back(static_cast<double>(w) / h);

        // third feature, w/W
        it->features.push_back(static_cast<double>(w) / imgWidth);

        // fourth feature, h/H
        it->features.push_back(static_cast<double>(h) / imgHeight);

        // distance from every edge
        // upper horizontal line
        it->features.push_back(this->patches[y1][0].first.distance(this->patches.front()[0].first));
        // lower horizontal line
        it->features.push_back(this->patches[y2][0].first.distance(this->patches.back()[0].first));
        // upper vertical line
        it->features.push_back(this->patches[0][x1].first.distance(this->patches[0].front().first));
        // lower vertical line
        it->features.push_back(this->patches[0][x2].first.distance(this->patches[0].back().first));

        // compute the RMS of the plane fit residual for this particular rectangle
        // in the same time compute the area of each label
        int empty = 0, occupied = 0, occluded = 0;

        // TODO figure out how to compute the residual
        double sqSum = 0.0;
        for (int i = y1; i < y2; ++i) {
            for (int j = x1; j < x2; ++j) {
                // TODO should be computed accordingly
                double temp = 0.0;

                switch (this->patches[i][j].second) {
                case EMPTY:
                    empty++;
                    break;
                case OCCUPIED:
                    occupied++;
                    break;
                case OCCLUDED:
                    occluded++;
                    break;
                default:
                    throw runtime_error("default branch taken while computing features for opening candidate");
                    break;
                }
                sqSum += pow(temp, 2.0);
            }
        }

        // add the RMS plane fit
        int totalCount = w * h;
        it->features.push_back(sqrt(sqSum / totalCount));

        // discard a few more candidates
        if ((static_cast<double>(empty) / totalCount) < MIN_EMPTY_AREA) {
            discarded++;
            continue;
        }

        // add the area of each label, empty, occupied, occluded
        it->features.push_back(static_cast<double>(empty) / totalCount);
        it->features.push_back(static_cast<double>(occupied) / totalCount);
        it->features.push_back(static_cast<double>(occluded) / totalCount);

        // determine how many interior rectangles this candidate contains
        // compute the number of interior inverted U-shapes as well
        // start counting from 2 as we include the edges as well
        int vCount = 2, hCount = 2;
        for (vector<int>::iterator jt = horizontal.begin(); jt != horizontal.end(); ++jt) {
            if (*jt > y1 && *jt < y2) {
                hCount++;
            }
        }

        for (vector<int>::iterator jt = vertical.begin(); jt != vertical.end(); ++jt) {
            if (*jt > x1 && *jt < x2) {
                vCount++;
            }
        }

        // add the number of rectangles contained inside
        it->features.push_back((hCount * (hCount - 1) / 2.0) * (vCount * (vCount - 1) / 2.0));

        // add the number of interior inverted U-shapes
        if (y2 == imgHeight - 1) {
            it->features.push_back((hCount - 1) * (vCount * (vCount - 1) / 2.0));
        } else {
            it->features.push_back(0.0);
        }

        // add this current candidate to the final result
        result.push_back(*it);
    }

    if (!quiet) cout << "** Discarded openings due to small empty area: " << discarded << endl;
    if (!quiet) cout << "** Done computing features for all opening candidates" << endl;
}
