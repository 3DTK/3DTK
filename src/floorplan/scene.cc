/**
 * @file scene.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 13 Feb 2012
 *
 */

//==============================================================================
//  Includes.
//==============================================================================
#include "floorplan/scene.h"

// User includes.
#include "floorplan/util.h"
#include "floorplan/colorGradient.h"
#include "slam6d/globals.icc"
#include "slam6d/io_utils.h"
#include "slam6d/io_types.h"
#include "slam6d/scan.h"

// C++ includes.
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
using namespace std;

// C includes.
#include <stdlib.h>

//==============================================================================
//  Extern variables.
//==============================================================================
extern bool quiet;

//==============================================================================
//  Static fields initialization.
//==============================================================================
const int floorplan::Scene::MIN_CROSS_SECTION_COUNT = 10;

//==============================================================================
//  Helper classes and structs.
//==============================================================================
struct PairComp {
  bool operator() (pair<int, int> a, pair<int, int> b) {
      return (a.first < b.first);
  }
} pairComparer;

//==============================================================================
//  Implementation.
//==============================================================================
vector<vector<vector<bool> > >
floorplan::Scene::compute3DOccGrid(const double& voxDimInCm,
        const double& minHeight, const double& maxHeight)
{
    // Determine the extremities of the scene.
    pair<double, double> xtrX;
    pair<double, double> xtrY;
    pair<double, double> xtrZ;
    this->findExtremities(xtrX, xtrY, xtrZ);

    // Adjust vertical extremities.
    if (xtrY.first < minHeight) {
        xtrY.first = minHeight;
    }

    if (xtrY.second > maxHeight) {
        xtrY.second = maxHeight;
    }

    int xCount = (abs(xtrX.second - xtrX.first) / voxDimInCm) + 1;
    int yCount = (abs(xtrY.second - xtrY.first) / voxDimInCm) + 1;
    int zCount = (abs(xtrZ.second - xtrZ.first) / voxDimInCm) + 1;

    vector<vector<vector<bool> > > occVox;
    occVox.resize(xCount);

    for (size_t it = 0; it < occVox.size(); ++it) {
        occVox[it].resize(yCount);

        for (size_t jt = 0; jt < occVox[it].size(); ++jt) {
            occVox[it][jt].assign(zCount, false);
        }
    }

    // Fill in the voxels.
    for (size_t it = 0; it < this->points.size(); ++it) {
        double currX = this->points[it].x;
        double currY = this->points[it].y;
        double currZ = this->points[it].z;

        // Since we may assign new extremities, check if current point falls into limits.
        if (currX < xtrX.first || currX > xtrX.second
                || currY < xtrY.first || currY > xtrY.second
                || currZ < xtrZ.first || currZ > xtrZ.second)
        {
            continue;
        }

        int xVox = floor((currX - xtrX.first) / voxDimInCm);
        int yVox = floor((currY - xtrY.first) / voxDimInCm);
        int zVox = floor((currZ - xtrZ.first) / voxDimInCm);

        occVox[xVox][yVox][zVox] = true;
    }

    return occVox;
}

floorplan::Scene::Scene(const IOType& type,
        const int& start, const int& end,
        string dir, const bool& scanserver,
        const int& maxDist,  const int& minDist,
        const int& octree, const double& red)
{
    if (!quiet) cout << endl << "== Creating scene..." << endl;
    if (!quiet) cout << endl << "== Reading scans..." << endl;

    // Identity matrix.
    double id[16];
    M4identity(id);

    // Begin loading the point cloud.
    Scan::openDirectory(scanserver, dir, type, start, end);

    if(Scan::allScans.size() == 0) {
        cerr << "No scans found. Did you use the correct format?" << endl;
        exit(EXIT_FAILURE);
    }

    int currScan = start;
    for(ScanVector::iterator scan = Scan::allScans.begin(); scan != Scan::allScans.end(); ++scan) {

        (*scan)->setRangeFilter(maxDist, minDist);
        (*scan)->setReductionParameter(red, octree);
        (*scan)->toGlobal();
        (*scan)->transform(id, Scan::ICP, 0);

        // Read the pose of the current scan.
        pair<Vector3d, Rotation3d> pose;
        const double* pt  = (*scan)->get_rPos();
        const double* rot = (*scan)->get_rPosTheta();

        pose.first.x = pt[0];
        pose.first.y = pt[1];
        pose.first.z = pt[2];

        pose.second.x = rot[0];
        pose.second.y = rot[1];
        pose.second.z = rot[2];

        if (!quiet) cout << endl << "== Loaded scan " << currScan << " @ pose ("
                << pose.first.x << " " << pose.first.y << " " << pose.first.z << " "
                << pose.second.x << " " << pose.second.y << " " << pose.second.z << ")..." << endl;

        // Keep the pointer to the points.
        DataXYZ points = (*scan)->get("xyz reduced");
        for (size_t it = 0; it < points.size(); ++it) {
            // XXX get_points returns a pointer
            Point3d toPush(points[it][0], points[it][1], points[it][2]);

            // Transform not required anymore.
            //toPush.translate(pose.first);
            //toPush.rotate(Point3d(0.0, 0.0, 0.0), pose.second);

            this->points.push_back(toPush);
        }

        // Contains the current scan number.
        currScan++;
    }

    // Adjust scans to start fom y = 0.
    // Y-axis is vertical.
    double minY = numeric_limits<double>::max();
    for (size_t it = 0; it < this->points.size(); ++it) {
        double currY = this->points[it].y;
        minY = min(minY, currY);
    }

    if (!quiet) cout << endl << "== Shifting points by " << -minY << " on Y-axis..." << endl;

    for (size_t it = 0; it < this->points.size(); ++it) {
        this->points[it].y -= minY;
    }

    // Construct the octree.
    this->octTreePoints = new double*[this->points.size()];

    // Go over all points from all the scans.
    for (unsigned int it = 0; it < this->points.size(); ++it) {
        double x = this->points[it].x;
        double y = this->points[it].y;
        double z = this->points[it].z;

        // Add the points to the array to be added to the octree.
        this->octTreePoints[it] = new double[3];
        this->octTreePoints[it][0] = x;
        this->octTreePoints[it][1] = y;
        this->octTreePoints[it][2] = z;
    }

    // Create the octree and fill it in.
    if (!quiet) cout << endl << "== Building OctTree with " << this->points.size() << " points..." << endl;
    this->octTree = new BOctTree<double>(this->octTreePoints, this->points.size(), octree);
    //this->_octTree->init();
    this->nrPoints = this->points.size();

    Scan::allScans.clear();
}

floorplan::Scene::Scene(const Scene& other) {
    this->points   = other.points;
    this->nrPoints = other.nrPoints;
    this->poses    = other.poses;

    // TODO copy octree.
}

floorplan::Scene::~Scene() {
    // Delete the octree.
    if (this->octTree != NULL) {
        delete this->octTree;
    }

    if (this->octTreePoints != NULL) {
        for (unsigned int i = 0; i < this->nrPoints; ++i) {
            if (this->octTreePoints[i] != NULL) {
                delete[] this->octTreePoints[i];
            }
        }
        delete[] this->octTreePoints;
    }
}

bool floorplan::Scene::isOccupied(const Point3d& center, const double& width) {
    // look for a close point
    double pt[] = {center.x, center.y, center.z};
    double* closest = this->octTree->FindClosest(pt, pow(sqrt(3) * (width / 2.0), 2.0), 0);

    // bounding box limits
    double xUppLim = center.x + width / 2.0;
    double xLowLim = center.x - width / 2.0;
    double yUppLim = center.y + width / 2.0;
    double yLowLim = center.y - width / 2.0;
    double zUppLim = center.z + width / 2.0;
    double zLowLim = center.z - width / 2.0;

    if (closest != NULL &&
            (closest[0] > xLowLim && closest[0] < xUppLim) &&
            (closest[1] > yLowLim && closest[1] < yUppLim) &&
            (closest[2] > zLowLim && closest[2] < zUppLim))
    {
        return true;
    }

    return false;
}

void floorplan::Scene::findExtremities(pair<double, double>& xtrX, pair<double, double>& xtrY, pair<double, double>& xtrZ) {
    if (!quiet) cout << endl << "== Determining scene extremities..." << endl;

    // Determine the extremities of the scene.
    xtrX.first = numeric_limits<double>::max();
    xtrX.second = -numeric_limits<double>::max();
    xtrY.first = numeric_limits<double>::max();
    xtrY.second = -numeric_limits<double>::max();
    xtrZ.first = numeric_limits<double>::max();
    xtrZ.second = -numeric_limits<double>::max();

    for (size_t it = 0; it < this->points.size(); ++it) {
        Point3d pt = this->points[it];

        xtrX.first = min(xtrX.first, pt.x);
        xtrX.second = max(xtrX.second, pt.x);
        xtrY.first = min(xtrY.first, pt.y);
        xtrY.second = max(xtrY.second, pt.y);
        xtrZ.first = min(xtrZ.first, pt.z);
        xtrZ.second = max(xtrZ.second, pt.z);
    }

    if (!quiet) {
        cout << endl << "== Scene extremities are: " << endl
                << "\t" << xtrX.first << ", " << xtrX.second
                << "\t" << xtrY.first << ", " << xtrY.second
                << "\t" << xtrZ.first << ", " << xtrZ.second
                << endl;
    }
}

vector<int> floorplan::Scene::computeVerticalHist(const double& bucketHeighInCm)
{
    vector<int> hist;
    if (!quiet) cout << endl << "== Computing vertical histogram..." << endl;

    // Get a 3D occupancy grid.
    vector<vector<vector<bool> > > occVox = this->compute3DOccGrid(bucketHeighInCm);

    // Fill in the histogram.
    hist.assign(occVox.front().size(), 0);
    for (size_t it = 0; it < occVox.size(); ++it) {
        for (size_t jt = 0; jt < occVox[it].size(); ++jt) {
            for (size_t kt = 0; kt < occVox[it][jt].size(); ++kt) {
                if (occVox[it][jt][kt] == true) {
                    hist[jt]++;
                }
            }
        }
    }

    return hist;
}

cv::Mat floorplan::Scene::computeHorizontalHist(const double& voxDimInCm,
        const bool& useBest, const bool& unreliableWalls)
{
    if (!quiet) cout << endl << "== Computing horizontal histogram..." << endl;

    // Compute the vertical histogram.
    vector<int> hist = this->computeVerticalHist(voxDimInCm);

//    for (size_t it = 0; it < hist.size(); ++it) {
//        cout << hist[it] << endl;
//    }

    // Compute the center of gravity of the histogram.
    int sum = 0, avgIdx = 0;
    for (size_t it = 0; it < hist.size(); ++it) {
        avgIdx += it * hist[it];
        sum += hist[it];
    }
    avgIdx = (int) round((double) avgIdx / sum);

    // Compute the ceiling and the floor.
    int floorIdx = max_element(hist.begin(), hist.begin() + avgIdx) - hist.begin();
    int ceilIdx = max_element(hist.begin() + avgIdx + 1, hist.end()) - hist.begin();

    if (!quiet) cout << endl << "== Floor idx: " << floorIdx << " (" << hist[floorIdx] << ")"
            << ";\tCeiling idx: " << ceilIdx << " (" << hist[ceilIdx] << ")" << "..." << endl;

    // Remove the floor and ceiling slice by setting limits.
    double floorLimit = (floorIdx + 1) * voxDimInCm;
    double ceilLimit = ceilIdx * voxDimInCm;

    // Compute a 3D histogram excluding points contained in the floor and ceiling slice.
    vector<vector<vector<bool> > > occVox = this->compute3DOccGrid(voxDimInCm, floorLimit, ceilLimit);

    // Create a 2D histogram.
    vector<vector<int> > hMap;
    hMap.resize(occVox.size());
    for (size_t it = 0; it < hMap.size(); ++it) {
        hMap[it].assign(occVox.front().front().size(), 0);
    }

    // Maximum value in the histogram.
    int maxVal = numeric_limits<int>::min();

    // Using the minimum occupied layers for computing the 2D histogram.
    if (useBest) {
        int n = round(MIN_CROSS_SECTION_COUNT * 10.0 / voxDimInCm);
        if (!quiet) cout << endl << "== Using smallest " << n << "cross sections..." << endl;

        vector<pair<int, int> > pairHist(occVox.front().size());
        for (size_t it = 0; it < pairHist.size(); ++it) {
            pairHist[it].first = 0;
            pairHist[it].second = it;
        }

        for (size_t it = 0; it < occVox.size(); ++it) {
            for (size_t jt = 0; jt < occVox[it].size(); ++jt) {
                for (size_t kt = 0; kt < occVox[it][jt].size(); ++kt) {
                    if (occVox[it][jt][kt] == true) {
                        pairHist[jt].first++;
                    }
                }
            }
        }

        sort(pairHist.begin(), pairHist.end(), pairComparer);

        for (size_t it = 0; it < occVox.size(); ++it) {

            for (int i = 0; i < n; ++i) {
                size_t jt = pairHist[i].second;

                for (size_t kt = 0; kt < occVox[it][jt].size(); ++kt) {
                    if (occVox[it][jt][kt] == true) {
                        hMap[it][kt]++;
                        if (maxVal < hMap[it][kt]) {
                            maxVal = hMap[it][kt];
                        }
                    }
                }
            }
        }



    } else {
        if (!quiet) cout << endl << "== Using all cross sections..." << endl;

        for (size_t it = 0; it < occVox.size(); ++it) {
            for (size_t jt = 0; jt < occVox[it].size(); ++jt) {
                for (size_t kt = 0; kt < occVox[it][jt].size(); ++kt) {
                    if (occVox[it][jt][kt] == true) {
                        hMap[it][kt]++;
                        if (maxVal < hMap[it][kt]) {
                            maxVal = hMap[it][kt];
                        }
                    }
                }
            }
        }
    }

    // Gray image.
    cv::Mat gray(occVox.size(), occVox.front().front().size(), CV_32FC1);
    for (size_t it = 0; it < hMap.size(); ++it) {
        for (size_t jt = 0; jt < hMap[it].size(); ++jt) {
            gray.at<float>(it, jt) = (float) hMap[it][jt] / maxVal;
        }
    }

    // Heat map image.
    float r = 0, g = 0, b = 0;
    ColorGradient colorGradient;
    colorGradient.createHotMap();

    cv::Mat heat(gray.rows, gray.cols, CV_32FC3);
    for (int y = 0; y < gray.rows; ++y) {
        for (int x = 0; x < gray.cols; ++x) {
            colorGradient.getColorAtValue(gray.at<float>(y, x), r, g, b);
            heat.at<cv::Vec3f>(y, x) = cv::Vec3f(b, g, r);
        }
    }

    if (unreliableWalls) {
        cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));
    }

    cv::imshow("gray", gray);
    cv::imshow("heat", heat);
    cv::waitKey();

    // Convert to CV_8UC1.
    gray.convertTo(gray, CV_8UC1, 255);
    return gray;
}
