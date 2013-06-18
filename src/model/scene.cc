/**
 * @file scene.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 13 Feb 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/scene.h"

#include "model/graphicsAlg.h"
#include "model/util.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "opencv2/photo/photo.hpp" 

#include <boost/algorithm/string.hpp>

#include <sys/stat.h>
#include <errno.h>

#include <limits>
#include <iomanip>
#include <fstream>
#include <sstream>
using namespace std;

//==============================================================================
//  Static fields initialization
//==============================================================================
const double model::Scene::PRECISION    = 0.0;
const double model::Scene::RAY_DIST     = 2.5;
const double model::Scene::PATCH_DIST   = 1.0;
const double model::Scene::WALL_DIST    = 10.0; // must be bigger than RAY_DIST
const double model::Scene::MIN_EDGE_COV = 0.60;

//==============================================================================
//  Implementation
//==============================================================================
model::Scene::Scene(const IOType& type,
				const int& start, const int& end,
				std::string dir, const bool& scanserver,
				const int& maxDist,  const int& minDist,
				const PlaneAlgorithm& alg, const int& octree, const double& red,
				const vector<Pose6d>& poses)
{
    if (!quiet) cout << endl << "== Creating scene..." << endl;
    if (!quiet) cout << endl << "== Reading scans..." << endl;

    // create an output directory for planes and for images
    if (makeDir(dir + "img/")== false) {
    	throw runtime_error("failed to create directory " + dir + "img/");
    }

    // copy class fields here
    this->poses = poses;

    // identity matrix
    double id[16];
    M4identity(id);

    // begin the plane detection
    //    Scan::dir = dir;
    //    Scan::readScans(type, start, end, dir, maxDist, minDist, false);
    Scan::openDirectory(scanserver, dir, type, start, end);

    if(Scan::allScans.size() == 0) {
	 cerr << "No scans found. Did you use the correct format?" << endl;
	 exit(-1);
    }
    
    int nrPlanes = 0 , currScan = start;
    //    for(vector <Scan*>::iterator scan = Scan::allScans.begin(); scan != Scan::allScans.end(); ++scan) {
    for(ScanVector::iterator scan = Scan::allScans.begin(); scan != Scan::allScans.end(); ++scan) {
     
        // prepare for plane detection
	   (*scan)->setRangeFilter(maxDist, minDist);
	   (*scan)->setReductionParameter(red, octree);
	   // scan->setSearchTreeParameter(nns_method, cuda_enabled);

        (*scan)->toGlobal();
        (*scan)->transform(id, Scan::ICP, 0);

        // read the pose of the current scan
        pair<Vector3d, Rotation3d> pose;
        const double* pt  = (*scan)->get_rPos();
        const double* rot = (*scan)->get_rPosTheta();

        pose.first.x = pt[0];
        pose.first.y = pt[1];
        pose.first.z = pt[2];

        pose.second.x = rot[0];
        pose.second.y = rot[1];
        pose.second.z = rot[2];

        if (!quiet) cout << "\n== Starting plane detection for scan " << currScan << " @ pose ("
                << pose.first.x << " " << pose.first.y << " " << pose.first.z << " "
                << pose.second.x << " " << pose.second.y << " " << pose.second.z << ")..." << endl;

        // apply hough transform
        Hough hough((*scan), true);
        nrPlanes += hough.RHT();    // TODO call proper algorithm

        if (!quiet) cout << "** Detected " << nrPlanes << " planes so far" << endl;

        // loop over all planes and fill in the data
        for (vector<ConvexPlane*>::iterator convexPlane = hough.planes.begin();
                convexPlane != hough.planes.end(); convexPlane++)
        {
            // get the nomrla and the point of application of the normal
            double tempNormal[3], tempOrigin[3];
            (*convexPlane)->getNormal(tempNormal, tempOrigin);

            // the normal and application points using our datastructures
            Vector3d normal(tempNormal[0], tempNormal[1], tempNormal[2]);
            Point3d origin(tempOrigin[0], tempOrigin[1], tempOrigin[2]);

            // fill in the convex hulls
            vector<double> doubleHull = (*convexPlane)->getConvexHull();
            vector<Point3d> pointHull;

            for (vector<double>::iterator point = doubleHull.begin(); point != doubleHull.end(); point += 3) {
                pointHull.push_back(Point3d(*(point+0), *(point+1), *(point+2)));
            }

            Plane3d planeToAdd(origin, normal, pointHull);

            // add the planes to the vector
            if (planeToAdd.isHorizontal() || planeToAdd.isVertical()) {
                this->planes.push_back(planeToAdd);
            }
            else if (!quiet) {
                cout << "** Discarding oblique plane centered at "
                        << planeToAdd.pt.x << " " << planeToAdd.pt.y << " " << planeToAdd.pt.z
                        << endl;
            }
        }

        // keep the pointer to the points
	   DataXYZ points = (*scan)->get("xyz reduced");
	   unsigned int points_size = points.size();
	   //        const vector<Point>* points = (*scan)->get_points();

        for (unsigned int i = 0; i < points_size; ++i) {
            // XXX get_points returns a pointer
            Point3d toPush(points[i][0], points[i][1], points[i][2]);
            toPush.translate(pose.first);
            toPush.rotate(Point3d(0.0, 0.0, 0.0), pose.second);
            this->points.push_back(toPush);
        }

        // contains the current scan number
        currScan++;
    }

    // to be used when constructing the octree
    this->octTreePoints = new double*[this->points.size()];

    // go over all points from all the scans
    for (unsigned int i = 0; i < this->points.size(); ++i) {
        double x = this->points[i].x;
        double y = this->points[i].y;
        double z = this->points[i].z;

        // add the points to the array to be added to the octree
        this->octTreePoints[i] = new double[3];
        this->octTreePoints[i][0] = x;
        this->octTreePoints[i][1] = y;
        this->octTreePoints[i][2] = z;
    }

    // Create the octree and fill it in.
    if (!quiet) cout << endl << "== Building OctTree with " << this->points.size() << " points ..." << endl;
    this->octTree = new BOctTree<double>(this->octTreePoints, this->points.size(), octree);
    //this->_octTree->init();
    this->nrPoints = this->points.size();

    Scan::allScans.clear();
}

model::Scene::Scene(const Scene& other) {
    this->points  = other.points;
    this->planes  = other.planes;
    this->walls   = other.walls;
    this->ceiling = other.ceiling;
    this->floor   = other.floor;
}

model::Scene::~Scene() {
    // delete the octree
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

vector<model::Plane3d> model::Scene::getConvexHull(vector<Plane3d> planes) {
    if (planes.size() <= 3) {
        return planes;
    }

    vector<Plane3d> hull;
    vector<Plane3d>::iterator leftmostIt = planes.begin();

    // compute the lefmost point
    for (vector<Plane3d>::iterator it = planes.begin() + 1;
            it != planes.end(); ++it)
    {
        // add only vertical planes
        if (it->isVertical() && it->pt.x < leftmostIt->pt.x) {
            leftmostIt = it;
        }
    }

    Plane3d planeOnHull = *leftmostIt;
    Plane3d endPlane;

    do {
        // add the candidate plane to the hull
        hull.push_back(planeOnHull);
        endPlane = planes[0];

        for (unsigned int j = 1; j < planes.size(); ++j) {
            // consider only vertical planes
            if (planes[j].isVertical() == false) {
                continue;
            }

            // define dummy planes in the XOZ plane
            Point3d fakeCurPt(planes[j].pt.x, 0.0, planes[j].pt.z);
            Point3d fakePointOnHull(planeOnHull.pt.x, 0.0, planeOnHull.pt.z);
            Point3d fakeEndPoint(endPlane.pt.x, 0.0, endPlane.pt.z);

            // define a point to the left of the line
            Point3d farLeft = fakeEndPoint;
            farLeft.rotate(planeOnHull.pt, Rotation3d(0.0, -M_PI/6, 0.0));

            // if ... or planes[j] on the left side of line (planeOnHull, endPlane)
            if (planeOnHull.pt != planes[j].pt &&
                    (endPlane.pt == planeOnHull.pt || sameSide(fakeCurPt, farLeft, fakePointOnHull, fakeEndPoint)))
            {
                endPlane = planes[j];
            }
        }

        planeOnHull = endPlane;

    } while (endPlane.pt != hull[0].pt);

    return hull;
}

vector<model::Plane3d> model::Scene::getSignificantPlanes(vector<Plane3d> planes) {
    vector<model::Plane3d> result;

    while (!planes.empty()) {
        // add the first plane as viable
        result.push_back(planes.front());
        planes.erase(planes.begin());;

        for (vector<Plane3d>::iterator it = planes.begin(); it < planes.end(); ++it) {
            if (result.back().isSamePlane(*it)) {
                result.back().normal += it->normal;
                planes.erase(it);
            }
        }

        // normalize this normal after we have normalized
        result.back().normal.normalize();
    }

    return result;
}

bool model::Scene::getCeiling(vector<Plane3d> planes, Plane3d& result) {
    vector<Plane3d> horizontalPlanes;

    for (vector<Plane3d>::iterator it = planes.begin(); it != planes.end(); ++it) {
        if (it->isHorizontal()) {
            horizontalPlanes.push_back(*it);
        }
    }

    if (horizontalPlanes.size() <= 0) {
        return false;
    }

    // start with the first horizontal plane
    result = planes.front();

    for (vector<Plane3d>::iterator it = planes.begin()+1;
            it != planes.end(); ++it)
    {
        if (it->pt.y > result.pt.y) {
            result = *it;
        }
    }

    return true;
}

bool model::Scene::getFloor(vector<Plane3d> planes, Plane3d& result) {
    vector<Plane3d> horizontalPlanes;

    for (vector<Plane3d>::iterator it = planes.begin();
            it != planes.end(); ++it)
    {
        if (it->isHorizontal()) {
            horizontalPlanes.push_back(*it);
        }
    }

    if (horizontalPlanes.size() <= 0) {
        return false;
    }

    // start with the first horizontal plane
    result = planes.front();

    for (vector<Plane3d>::iterator it = planes.begin()+1;
            it != planes.end(); ++it)
    {
        if (it->pt.y < result.pt.y) {
            result = *it;
        }
    }

    return true;
}

void model::Scene::detectWalls() {
    if (!quiet) cout << endl << "== Building model..." << endl;

    // temporary wall storage
    vector<Plane3d> tempWalls = this->planes;

    // extract the convex hull of the room
    tempWalls = GraphicsAlg::getConcaveHull(tempWalls);

    // throw away redundant tempWalls
    tempWalls = this->getSignificantPlanes(tempWalls);

    // get other bounds, ceiling and floor
    Plane3d tempCeiling, tempFloor;

    if (!this->getCeiling(this->planes, tempCeiling) || !this->getFloor(this->planes, tempFloor)) {
        throw runtime_error("error while determining ceiling or floor");
    }

    if (tempWalls.size() < 4) {
        throw runtime_error("room has less than 4 walls");
    }

    if (!quiet) cout << "** " << tempWalls.size() << " walls have been detected" << endl;

    // XXX planes in the _walls vector should be in order
    vector< pair<Point3d, Point3d> > corners;
    Point3d upperCorner, lowerCorner;

    if(!quiet) cout << endl << "== Computing corners..." << endl;
    for (vector<Plane3d>::iterator it = tempWalls.begin(); it != tempWalls.end(); ++it) {
        // be careful at the last element
        if (it < tempWalls.end()-1) {
            upperCorner = it->intersect(*(it+1), tempCeiling);
            lowerCorner = it->intersect(*(it+1), tempFloor);
        }
        else if (it == tempWalls.end()-1) {
            upperCorner = it->intersect(tempWalls.front(), tempCeiling);
            lowerCorner = it->intersect(tempWalls.front(), tempFloor);
        }

        if (!quiet) cout << showpos << "** Computed upper corner " << upperCorner << endl;
        if (!quiet) cout << showpos << "** Computed lower corner " << lowerCorner << endl;

        corners.push_back(make_pair(upperCorner, lowerCorner));
    }

    if (!quiet) cout << endl << "== Building walls, ceiling and floor..." << endl;

    // used to create the walls
    Point3d upperLeft, upperRight;
    Point3d lowerLeft, lowerRight;

    // used to create the floor and the ceiling
    vector<Point3d> ceilingHull;
    vector<Point3d> floorHull;

    Point3d ceilingCenter(0.0, 0.0, 0.0);
    Point3d floorCenter(0.0, 0.0, 0.0);

    Vector3d ceilingNormal;
    Vector3d floorNormal;

    for (vector< pair<Point3d, Point3d> >::iterator it = corners.begin();
            it != corners.end(); ++it)
    {
        upperLeft = it->first;
        lowerLeft = it->second;

        if (it < corners.end()-1) {
            upperRight = (it+1)->first;
            lowerRight = (it+1)->second;
        }
        else if (it == corners.end()-1) {
            upperRight = corners.front().first;
            lowerRight = corners.front().second;
        }

        Point3d center((upperLeft.x + lowerLeft.x + upperRight.x + lowerRight.x) / 4.0,
                (upperLeft.y + lowerLeft.y + upperRight.y + lowerRight.y) / 4.0,
                (upperLeft.z + lowerLeft.z + upperRight.z + lowerRight.z) / 4.0);

        vector<Point3d> hull;
        hull.push_back(upperLeft);
        hull.push_back(lowerLeft);
        hull.push_back(lowerRight);
        hull.push_back(upperRight);

        if (!quiet) cout << "** Adding wall centered at " << center << endl;
        this->walls.push_back(LabeledPlane3d(center, Vector3d(), hull));
        this->walls.back().normal = this->walls.back().computeAverageNormal();

        // compute the floor and the ceiling
        ceilingHull.push_back(it->first);
        floorHull.push_back(it->second);

        ceilingCenter.x += it->first.x;
        ceilingCenter.y += it->first.y;
        ceilingCenter.z += it->first.z;

        floorCenter.x += it->second.x;
        floorCenter.y += it->second.y;
        floorCenter.z += it->second.z;
    }
    ceilingCenter /= ceilingHull.size();
    floorCenter   /= floorHull.size();

    // dummy normal
    Vector3d dummy;

    if (!quiet) cout << "** Adding ceiling centered at " << ceilingCenter << endl;
    this->ceiling = LabeledPlane3d(ceilingCenter, dummy, ceilingHull);
    this->ceiling.normal = this->ceiling.computeAverageNormal();

    if (!quiet) cout << "** Adding floor centered at " << floorCenter << endl;
    this->floor = LabeledPlane3d(floorCenter, dummy, floorHull);
    this->floor.normal = this->floor.computeAverageNormal();
}

bool model::Scene::castRay(const Point3d& src, const Point3d& dest, const double& extraDist,
        Point3d& ptHit)
{
    // put the Bresenham line in here, we need some extra distance in case the points are close
    // but after the detected wall, therefore we need to go a bit further than the wall to check
    vector<Point3d> line;
    GraphicsAlg::getDiscreteLine(src, dest, PRECISION, extraDist, line);

    // loop through all the points in the Bresenham line and decide if it hits something
    for (vector<Point3d>::iterator it = line.begin(); it != line.end(); ++it) {
        if (this->isOccupied(*it, RAY_DIST)) {
            ptHit = *it;
            return true;
        }
    }

    // no occlusion took place
    return false;
}

void model::Scene::applyLabels(LabeledPlane3d& surf) {
    if (this->poses.empty()) {
        throw runtime_error("no poses have been provided to the scene");
    }

    // the maximum distance from which we subtract to create a depth image
    double maxDist = 2.0 * WALL_DIST + RAY_DIST;

    // determine the points on the plane for which to apply labels
    vector<vector<Point3d> > discretePoints = surf.getDiscretePoints(PATCH_DIST);

    surf.patches.resize(discretePoints.size());
    surf.depthMap.resize(discretePoints.size());

    // we need these values for later
    surf.depthMapDistances.first = WALL_DIST;
    surf.depthMapDistances.second = maxDist;

    for (unsigned int i = 0; i < discretePoints.size(); ++i) {
        for (unsigned int j = 0; j < discretePoints.front().size(); ++j) {
            // consider everything is occluded initially
            surf.patches[i].push_back(make_pair(discretePoints[i][j], OCCLUDED));
            surf.depthMap[i].push_back(WALL_DIST);
        }
    }

    if (!quiet) cout << endl << "== Performing ray casting for surface centered at " << surf.pt << endl;

    for (vector<Pose6d>::iterator srcPose = this->poses.begin(); srcPose != this->poses.end(); ++srcPose) {
        for (unsigned int i = 0; i < surf.patches.size(); ++i) {
            for (unsigned int j = 0; j <  surf.patches[i].size(); ++j) {
                // prepare two points for ray casting through wall
                Point3d ptOnWall  = surf.patches[i][j].first;
                Point3d src(surf.normal.x + ptOnWall.x,
                        surf.normal.y + ptOnWall.y,
                        surf.normal.z + ptOnWall.z);

                double len = ptOnWall.distance(src);
                double temp = (len + WALL_DIST) / len;
                src.x = ptOnWall.x + (src.x - ptOnWall.x) * temp;
                src.y = ptOnWall.y + (src.y - ptOnWall.y) * temp;
                src.z = ptOnWall.z + (src.z - ptOnWall.z) * temp;

                // remember the point we hit when ray casting
                Point3d ptHit;

                if (insideHull(surf.patches[i][j].first, surf.hull) && castRay(src, ptOnWall, WALL_DIST, ptHit)) {
                    surf.patches[i][j].second = OCCUPIED;
                    surf.depthMap[i][j] = maxDist - src.distance(ptHit);
                } else if (!castRay(srcPose->first, surf.patches[i][j].first, 0, ptHit)) {
                    surf.patches[i][j].second = EMPTY;
                    surf.depthMap[i][j] = 0.0;
                }
            }
        }
    }

    // create the OpenCV depth image
    int imgHeight = static_cast<int>(surf.depthMap.size());
    int imgWidth  = static_cast<int>(surf.depthMap.front().size());

    // crate an OpenCV image and allocate memory for it
    cv::Mat img(imgHeight, imgWidth, CV_8UC1);

    // compute the maximum from the image
    double max = numeric_limits<double>::min();
    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            if (max < surf.depthMap[i][j]) {
                max = surf.depthMap[i][j];
            }
        }
    }

    // write the current image to a file
    cv::Mat labels(imgHeight, imgWidth, CV_8UC3);

    // adjust the image
    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            img.data[(i * img.cols) + j] = static_cast<char>(MAX_IMG_VAL * surf.depthMap[i][j] / max);

            // put the color into the image according to the label
            int curr = (i * 3 * img.cols) + 3 * j;
            switch (surf.patches[i][j].second) {
            case EMPTY:
            	labels.data[curr + 0] = 0;
            	labels.data[curr + 1] = 200;
            	labels.data[curr + 2] = 0;
            	break;
            case OCCLUDED:
            	labels.data[curr + 0] = 200;
            	labels.data[curr + 1] = 0;
            	labels.data[curr + 2] = 0;
            	break;
            case OCCUPIED:
            	labels.data[curr + 0] = 0;
            	labels.data[curr + 1] = 0;
            	labels.data[curr + 2] = 200;
            	break;
            default:
					throw runtime_error("invalid default branch taken");
					break;
			}
        }
    }
    cv::imwrite("./img/labels.png", labels);

    // copy the image
    surf.depthImg = img.clone();
}

void model::Scene::applyAllLabels() {
    if (!quiet) cout << endl << "== Performing ray casting for all surfaces..." << endl;

    applyLabels(this->ceiling);
    applyLabels(this->floor);

    for (vector<LabeledPlane3d>::iterator it = this->walls.begin(); it != this->walls.end(); ++it) {
        applyLabels(*it);
    }
}

void model::Scene::detectPotentialOpenings(const LabeledPlane3d& surf,
        std::vector<CandidateOpening>& openings)
{
    if (!quiet) cout << endl << "== Determining openings for surface centered at " << surf.pt << endl;

    // get the candidate openings for current plane
    vector<CandidateOpening> candidates;
    surf.computeOpeningCandidates(candidates);

    // TODO train the SVM using some outside variables, not hard coded in here, play around a lot with these numbers
    // prepare to train the SVM

    const unsigned int nrSamples    = 68;
    const unsigned int nrMainClass  = 14;
    const unsigned int nrFeatures   = model::CandidateOpening::NR_FEATURES;

    // make a set of excluded features
    int excludedFeaturesInit[] = {6, 7, 8, 12, 13};
    set<int> excludedFeatures(excludedFeaturesInit, excludedFeaturesInit + 5);

    // set up the training data using some real life examples, needs to be flaot
    float trainingData[nrSamples][14] = {
            // TODO fix the RMS fit residual
            // 1 area | 2 w/h | 3 w/W | 4 h/H | 5-8 dist to edges | 9 RMS | 10-12 E Occup Occl | 13 int rect | 14 int U-shapes
            { 3120, 0.42, 0.05, 0.36,  23, 138, 550, 126, 0, 0.93, 0.01, 0.06, 15, 0},
            { 3090, 0.42, 0.05, 0.36,  23, 138, 126, 550, 0, 0.93, 0.01, 0.06, 15, 0},
            { 6810, 0.94, 0.11, 0.34,  23, 140, 453, 176, 0, 0.91, 0.02, 0.07, 15, 0},
            { 6815, 0.94, 0.11, 0.34,  23, 140, 176, 453, 0, 0.91, 0.02, 0.07, 15, 0},
            {11780, 1.52, 0.19, 0.35,  23, 139, 450, 128, 0, 0.89, 0.01, 0.10, 15, 0},
            {11190, 1.52, 0.18, 0.35,  23, 139, 128, 450, 0, 0.88, 0.02, 0.10, 15, 0},
            {10374, 1.70, 0.18, 0.31,  31, 141, 453, 124, 0, 0.87, 0.02, 0.11, 15, 0},
            {10374, 1.70, 0.18, 0.31,  31, 141, 124, 453, 0, 0.87, 0.02, 0.11, 15, 0},
            {27600, 0.82, 0.22, 0.62,  18,  97, 403, 148, 0, 0.95, 0.03, 0.02, 12, 0},
            {27600, 0.82, 0.22, 0.62,  18,  97, 148, 403, 0, 0.94, 0.03, 0.03, 12, 0},
            {25516, 0.87, 0.22, 0.59,  28, 100,  49, 396, 0, 0.84, 0.11, 0.05, 20, 0},
            {26316, 0.87, 0.22, 0.59,  28, 100, 396,  49, 0, 0.84, 0.13, 0.03, 22, 0},
            {25516, 0.87, 0.22, 0.59,  28, 100,  70, 396, 0, 0.84, 0.11, 0.05, 20, 0},
            {26316, 0.87, 0.22, 0.59,  28, 100, 396,  70, 0, 0.84, 0.13, 0.03, 22, 0},
            //========================================================================
            {+143289, +2.10345, +0.786533, +0.887755, +31, +1, +1, +147, +0, +0.371222, +0.524681, +0.104097, +34300, +0},
            {+180612, +2.65134, +0.991404, +0.887755, +31, +1, +1, +4, +0, +0.294571, +0.620845, +0.0845846, +44100, +0},
            {+180873, +2.65517, +0.992837, +0.887755, +31, +1, +1, +3, +0, +0.294146, +0.621392, +0.0844626, +55125, +0},
            {+181395, +2.66284, +0.995702, +0.887755, +31, +1, +1, +1, +0, +0.293299, +0.622481, +0.0842195, +67375, +0},
            {+181656, +2.66667, +0.997135, +0.887755, +31, +1, +1, +0, +0, +0.292878, +0.621846, +0.0852766, +80850, +0},
            {+181828, +2.64885, +0.994269, +0.891156, +31, +0, +0, +3, +0, +0.292634, +0.621114, +0.0862518, +70125, +2750},
            {+143838, +2.09542, +0.786533, +0.891156, +31, +0, +1, +147, +0, +0.369805, +0.525466, +0.104729, +35700, +1400},
            {+181304, +2.64122, +0.991404, +0.891156, +31, +0, +1, +4, +0, +0.293446, +0.621465, +0.0850891, +45900, +1800},
            {+181566, +2.64504, +0.992837, +0.891156, +31, +0, +1, +3, +0, +0.293023, +0.622011, +0.0849663, +57375, +2250},
            {+182090, +2.65267, +0.995702, +0.891156, +31, +0, +1, +1, +0, +0.29218, +0.623093, +0.0847273, +70125, +2750},
            {+182352, +2.65649, +0.997135, +0.891156, +31, +0, +1, +0, +0, +0.29176, +0.622455, +0.0857846, +84150, +3300},
            {+120270, +2.70142, +0.792768, +0.717687, +31, +51, +1, +147, +0, +0.427181, +0.486813, +0.0860065, +8400, +0},
            {+150654, +3.38389, +0.993046, +0.717687, +31, +51, +1, +3, +0, +0.341299, +0.588308, +0.0703931, +10800, +0},
            {+150865, +3.38863, +0.994437, +0.717687, +31, +51, +1, +2, +0, +0.340821, +0.588884, +0.0702946, +13500, +0},
            {+39680, +0.605469, +0.222063, +0.870748, +32, +5, +396, +146, +0, +0.654738, +0.294254, +0.0510081, +3243, +0},
            {+38656, +0.589844, +0.216332, +0.870748, +32, +5, +400, +146, +0, +0.672082, +0.27931, +0.0486082, +1081, +0},
            {+39835, +0.603113, +0.222063, +0.87415, +32, +4, +396, +146, +0, +0.655153, +0.293787, +0.0510606, +3384, +0},
            {+38807, +0.587549, +0.216332, +0.87415, +32, +4, +400, +146, +0, +0.672508, +0.278893, +0.0485995, +1128, +0},
            {+39990, +0.600775, +0.222063, +0.877551, +32, +3, +396, +146, +0, +0.655539, +0.293373, +0.0510878, +3528, +0},
            {+38958, +0.585271, +0.216332, +0.877551, +32, +3, +400, +146, +0, +0.672904, +0.278505, +0.0485908, +1176, +0},
            {+40145, +0.598456, +0.222063, +0.880952, +32, +2, +396, +146, +0, +0.655374, +0.293586, +0.05104, +3675, +0},
            {+39109, +0.583012, +0.216332, +0.880952, +32, +2, +400, +146, +0, +0.672735, +0.278759, +0.0485055, +1225, +0},
            {+40300, +0.596154, +0.222063, +0.884354, +32, +1, +396, +146, +0, +0.652854, +0.296154, +0.0509926, +3825, +0},
            {+39260, +0.580769, +0.216332, +0.884354, +32, +1, +400, +146, +0, +0.670148, +0.281457, +0.0483953, +1275, +0},
            {+40455, +0.59387, +0.222063, +0.887755, +32, +0, +396, +146, +0, +0.650352, +0.298702, +0.0509455, +3978, +153},
            {+39411, +0.578544, +0.216332, +0.887755, +32, +0, +400, +146, +0, +0.66758, +0.284134, +0.048286, +1326, +51},
            {+115564, +4.14371, +0.991404, +0.568027, +33, +93, +1, +4, +0, +0.429745, +0.554031, +0.0162248, +990, +0},
            {+115731, +4.1497, +0.992837, +0.568027, +33, +93, +1, +3, +0, +0.429124, +0.554674, +0.0162014, +1170, +0},
            {+116065, +4.16168, +0.995702, +0.568027, +33, +93, +1, +1, +0, +0.42789, +0.555956, +0.0161547, +1365, +0},
            {+116232, +4.16766, +0.997135, +0.568027, +33, +93, +1, +0, +0, +0.427275, +0.555544, +0.0171812, +1575, +0},
            {+118674, +4.05848, +0.994269, +0.581633, +33, +89, +0, +3, +0, +0.419292, +0.556575, +0.0241333, +1911, +0},
            {+119016, +4.07018, +0.997135, +0.581633, +33, +89, +0, +1, +0, +0.418087, +0.557849, +0.024064, +2205, +0},
            {+118332, +4.04678, +0.991404, +0.581633, +33, +89, +1, +4, +0, +0.420453, +0.556739, +0.0228087, +1386, +0},
            {+33790, +0.711009, +0.222063, +0.741497, +33, +42, +396, +146, +0, +0.736372, +0.215715, +0.0479136, +828, +0},
            {+33136, +0.697248, +0.217765, +0.741497, +33, +42, +399, +146, +0, +0.750905, +0.201231, +0.0478634, +276, +0},
            {+65880, +4.575, +0.786533, +0.408163, +33, +140, +1, +147, +0, +0.541166, +0.455055, +0.0037796, +45, +0},
            {+66360, +4.60833, +0.792264, +0.408163, +33, +140, +1, +143, +0, +0.537251, +0.458996, +0.00375226, +55, +0},
            {+135218, +3.59278, +0.998567, +0.659864, +33, +66, +0, +0, +0, +0.375216, +0.557995, +0.0667884, +11550, +0},
            {+135135, +3.55385, +0.992837, +0.663265, +33, +65, +0, +4, +0, +0.375484, +0.557457, +0.0670589, +6468, +0},
            {+135330, +3.55897, +0.994269, +0.663265, +33, +65, +0, +3, +0, +0.374943, +0.558095, +0.0669622, +8316, +0},
            {+135720, +3.56923, +0.997135, +0.663265, +33, +65, +0, +1, +0, +0.373865, +0.559365, +0.0667698, +10395, +0},
            {+135915, +3.57436, +0.998567, +0.663265, +33, +65, +0, +0, +0, +0.373329, +0.558893, +0.0677777, +12705, +0},
            {+142065, +3.38049, +0.992837, +0.697279, +33, +55, +0, +4, +0, +0.357998, +0.572006, +0.0699961, +7728, +0},
            {+117117, +4.10059, +0.992837, +0.57483, +30, +94, +0, +4, +0, +0.428213, +0.555837, +0.0159499, +280, +0},
            {+117286, +4.10651, +0.994269, +0.57483, +30, +94, +0, +3, +0, +0.427596, +0.556477, +0.0159269, +360, +0},
            {+117624, +4.11834, +0.997135, +0.57483, +30, +94, +0, +1, +0, +0.426367, +0.557752, +0.0158811, +450, +0},
            {+117793, +4.12426, +0.998567, +0.57483, +30, +94, +0, +0, +0, +0.425755, +0.557359, +0.0168856, +550, +0},
            {+43176, +1.52976, +0.368195, +0.571429, +30, +95, +396, +44, +0, +0.571822, +0.428178, +0, +36, +0},
            {+43344, +1.53571, +0.369628, +0.571429, +30, +95, +396, +43, +0, +0.569606, +0.430394, +0, +60, +0},
            {+42672, +1.5119, +0.363897, +0.571429, +30, +95, +399, +44, +0, +0.578576, +0.421424, +0, +18, +0},
            {+42840, +1.51786, +0.36533, +0.571429, +30, +95, +399, +43, +0, +0.576307, +0.423693, +0, +36, +0},
            {+42504, +1.50595, +0.362464, +0.571429, +30, +95, +400, +44, +0, +0.580863, +0.419137, +0, +6, +0},
            {+42672, +1.5119, +0.363897, +0.571429, +30, +95, +400, +43, +0, +0.578576, +0.421424, +0, +18, +0},
            {+33338, +0.748815, +0.226361, +0.717687, +29, +53, +396, +143, +0, +0.760244, +0.195453, +0.0443038, +1134, +0},
    };

    // match the labels to the training data
    float mainLabel = 1.0;
    float labels[nrSamples];
    // create the labels
    for (unsigned int i = 0; i < nrSamples; ++i) {
        if (i < nrMainClass) {
            labels[i] = mainLabel;
        } else {
            labels[i] = -mainLabel;
        }
    }

    // normalize the values above
    float maxFeatures[nrFeatures];

    for (unsigned int i = 0; i < nrFeatures; ++i) {
        maxFeatures[i] = numeric_limits<float>::min();
    }

    for (unsigned int i = 0; i < nrSamples; ++i) {
        for (unsigned int j = 0; j < nrFeatures; ++j) {
            if (maxFeatures[j] < trainingData[i][j]) {
                maxFeatures[j] = trainingData[i][j];
            }
        }
    }

    for (unsigned int i = 0; i < nrSamples; ++i) {
        for (unsigned int j = 0; j < nrFeatures; ++j) {
            if (excludedFeatures.find(j) != excludedFeatures.end()) {
                trainingData[i][j] = 0.0;
            } else {
                trainingData[i][j] /= maxFeatures[j];
            }
        }
    }

    // compute the max of the values of the candidates, but normalize later
    for (unsigned int i = 0; i < nrFeatures; ++i) {
        maxFeatures[i] = numeric_limits<float>::min();
    }

    for (vector<CandidateOpening>::iterator it = candidates.begin(); it != candidates.end(); ++it) {
        for (unsigned int feat = 0; feat < nrFeatures; ++feat) {
            if (maxFeatures[feat] < it->features[feat]) {
                maxFeatures[feat] = it->features[feat];
            }
        }
    }

    // create two OpenCV matrices
    cv::Mat trainingMat(nrSamples, nrFeatures, CV_32FC1, trainingData);
    cv::Mat labelsMat(nrSamples, 1, CV_32FC1, labels);

    // set up the SVM parameters
    cv::SVMParams params;
    params.svm_type = cv::SVM::NU_SVC;
    params.kernel_type = cv::SVM::RBF;
    params.term_crit = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, 1e-6);

    params.nu    = 0.8;
    params.C     = 8.0;
    params.gamma = 20;

    // create SVM and train it
    cv::SVM svm;
    svm.train_auto(trainingMat, labelsMat, cv::Mat(), cv::Mat(), params); // XXX with or without auto

    // store these openings, and filter some more later on
    std::vector<CandidateOpening> tempOpenings;

    for (vector<CandidateOpening>::iterator it = candidates.begin(); it != candidates.end(); ++it) {
        if (it->features.size() != nrFeatures) {
            throw runtime_error("number of features in candidate in does not equal the expected one");
        }

        float candidateData[nrFeatures];
        for (size_t jt = 0; jt < it->features.size(); ++jt) {
            if (excludedFeatures.find(jt) != excludedFeatures.end()) {
                candidateData[jt] = 0.0;
            } else {
                // also normalize the data here, and not in the candidateOpening object
                candidateData[jt] = static_cast<float>(it->features[jt]) / maxFeatures[jt];
            }
        }
        cv::Mat candidateMat(1, nrFeatures, CV_32FC1, candidateData);

        // decide of which type the current opening is
        float response = svm.predict(candidateMat);
        if (response == mainLabel) {
            tempOpenings.push_back(*it);
        }
    }

    // filter openings with respect to how much they overlap with the Canny edge
    cv::Mat canny, hSobel, vSobel, combined;
    surf.detectEdges(canny, hSobel, vSobel, combined);

    double maxEdgeCoverage = numeric_limits<double>::min();
    double averageArea = 0.0;
    for (vector<CandidateOpening>::iterator it = tempOpenings.begin(); it != tempOpenings.end(); ++it) {
        int y1 = it->edges[0];
        int y2 = it->edges[1];
        int x1 = it->edges[2];
        int x2 = it->edges[3];

        // how many pixels overlap?
        int total = 2 * (y2 - y1 + x2 - x1) + 1;
        int count = total;

        if (y2 < y1 || x2 < x1) {
            throw runtime_error("candidate edges must be in proper order");
        }

        for (int i = y1; i <= y2; ++i) {
            if (combined.data[i * combined.cols + x1] == 0) {
                count--;
            }
            if (combined.data[i * combined.cols + x2] == 0) {
                count--;
            }
        }

        for (int i = x1; i <= x2; ++i) {
            if (combined.data[y1 * combined.cols + i] == 0) {
                count--;
            }
            if (combined.data[y2 * combined.cols + i] == 0) {
                count--;
            }
        }

        // remember the edge coverage inside the object
        it->edgeCoverage = static_cast<double>(count) / total;
        if (maxEdgeCoverage < it->edgeCoverage) {
            maxEdgeCoverage = it->edgeCoverage;
        }

        // increment the average area
        averageArea += it->features[0];
    }
    averageArea /= tempOpenings.size();

    unsigned int discarded = 0;
    for (vector<CandidateOpening>::iterator it = tempOpenings.begin(); it != tempOpenings.end(); ++it) {
        // TODO put coef in class
        if ((it->features[0] > 0.3 * averageArea) &&
                (it->edgeCoverage > MIN_EDGE_COV * maxEdgeCoverage))
        {
            openings.push_back(*it);
        } else {
            discarded++;
        }
    }

    if (!quiet) cout << "** Discarded openings due to relative size to average candidate: " << discarded << endl;

    // draw the chosen rectangles, and also the occupancy map
    cv::Mat imgWithOpenings = surf.depthImg;
    cv::cvtColor(imgWithOpenings, imgWithOpenings, CV_GRAY2BGR, 3);

    for (size_t i = 0; i < openings.size(); ++i) {
        int y1 = openings[i].edges[0];
        int y2 = openings[i].edges[1];
        int x1 = openings[i].edges[2];
        int x2 = openings[i].edges[3];

        cv::rectangle(imgWithOpenings, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, MAX_IMG_VAL), 1);
    }

    cv::imwrite("./img/depthCandidates.png", imgWithOpenings);

    if (!quiet) cout << "** Detected " << openings.size() << " potential openings" << endl;
}

void model::Scene::clusterOpenings(const LabeledPlane3d& surf, const vector<CandidateOpening>& openings,
        std::vector<CandidateOpening>& result) const
{
    // clear the result just in case
    result.clear();

    if (openings.size() == 1) {
        if (!quiet) cerr << "WARNING: only one candidate received for clustering!" << endl;
        result.push_back(openings.front());
        return;
    } else if (openings.size() < 1) {
        if (!quiet) cerr << "WARNING: no candidates received for clustering!" << endl;
        return;
    }

    if (!quiet) cout << endl << "== Clustering openings for surface centered at " << surf.pt << endl;

    size_t nrSamples = openings.size();
    unsigned int dim = 7;

    // a few matrix initializations
    CvMat* points   = cvCreateMat(nrSamples, dim, CV_32FC1);
    CvMat* clusters = cvCreateMat(nrSamples, 1, CV_32SC1);

    // fill in the matrices with data
    for (size_t i = 0; i < nrSamples; ++i) {
        if (openings[i].features.size() != model::CandidateOpening::NR_FEATURES) {
            throw runtime_error("invalid feature vector size");
        }

        // compute the center of each opening
        float x = static_cast<float>(openings[i].edges[2] + openings[i].edges[3]) / 2.0;
        float y = static_cast<float>(openings[i].edges[0] + openings[i].edges[1]) / 2.0;

        points->data.fl[i * dim + 0]  = x;                      // window center
        points->data.fl[i * dim + 1]  = y;                      // window center
        points->data.fl[i * dim + 2]  = openings[i].edges[0];   // upper edge
        points->data.fl[i * dim + 3]  = openings[i].edges[1];   // lower edge
        points->data.fl[i * dim + 4]  = openings[i].edges[2];   // left edge
        points->data.fl[i * dim + 5]  = openings[i].edges[3];   // right edge
        points->data.fl[i * dim + 6]  = openings[i].features[1];// edge ratio
    }

    // termination criteria for the kMeans algorithm
    CvTermCriteria term;
    term.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
    term.max_iter = 10;
    term.epsilon  = 1.0;
    int maxNrClusters = 256;

    int nrClusters = 2;                                             // from how many clusters to start
    double prev, compactness;                                       // previous compactness, and current compactness
    CvMat *centers = cvCreateMat(nrClusters - 1, dim, CV_32FC1);    // place where we compute the centers of each cluster
    cvKMeans2(points, nrClusters - 1, clusters, term, 1, 0, 0, centers, &prev);

    while (1) {
        cvReleaseMat(&centers);
        centers = cvCreateMat(nrClusters, dim, CV_32FC1);
        cvKMeans2(points, nrClusters, clusters, term, 1, 0, 0, centers, &compactness);

        // TODO put somewhere everything in class
        double deltaCompactness = compactness - prev;
        if (!quiet) cout << "** Delta compactness: " << deltaCompactness << endl;
        if (fabs(deltaCompactness) < 1.0 || nrClusters >= maxNrClusters) {
            break;
        }

        prev = compactness;
        nrClusters++;
    }

    if (!quiet) cout << "** Detected " << static_cast<unsigned int>(nrClusters) << " clusters of potential openings" << endl;

    // generate colors for each cluster
    int r, g, b;
    cv::Scalar colors[nrClusters];
    for (int i = 0; i < nrClusters; ++i) {
        randomColor(MAX_IMG_VAL, r, g, b);
        colors[i] = (cv::Scalar(b, g, r));
    }

    // create an image with the clusters in separate colors
    cv::Mat clusterImg = surf.depthImg;
    cv::Mat finalImg   = surf.depthImg;
    cv::cvtColor(clusterImg, clusterImg, CV_GRAY2BGR, 3);
    cv::cvtColor(finalImg, finalImg, CV_GRAY2BGR, 3);

    // count how many elements we have in each cluster
    vector<int> clusterCount;
    clusterCount.resize(nrClusters);
    fill(clusterCount.begin(), clusterCount.end(), 0);

    // compute the candidate for each cluster with the best coverage of empty area
    vector<CandidateOpening> bestCandidates;
    bestCandidates.resize(nrClusters);

    vector<double> bestEmptyAreas;
    bestEmptyAreas.resize(nrClusters);
    fill(bestEmptyAreas.begin(), bestEmptyAreas.end(), numeric_limits<double>::min());

    // fill in the result separating each cluster
    for (size_t i = 0; i < nrSamples; ++i) {
        // the cluster of the current opening
        int currCluster = clusters->data.i[i];

        // draw each cluster
        int y1 = openings[i].edges[0];
        int y2 = openings[i].edges[1];
        int x1 = openings[i].edges[2];
        int x2 = openings[i].edges[3];
        cv::rectangle(clusterImg, cv::Point(x1, y1), cv::Point(x2, y2), colors[currCluster], 1);

        // TODO avoid hard coding
        // find which is the best choice from the cluster
        double currEmptyArea = openings[i].features[0] *
                (openings[i].features[9] - openings[i].features[10] - openings[i].features[11]);
        if (bestEmptyAreas[currCluster] < currEmptyArea) {
            bestEmptyAreas[currCluster] = currEmptyArea;
            bestCandidates[currCluster] = openings[i];
        }

        clusterCount[currCluster]++;
    }

    // divide by the cluster count to get the average of the cluster
    for (size_t i = 0; i < bestEmptyAreas.size(); ++i) {
        if (clusterCount[i] <= 0) {
            continue;
        }

        int y1 = bestCandidates[i].edges[0];
        int y2 = bestCandidates[i].edges[1];
        int x1 = bestCandidates[i].edges[2];
        int x2 = bestCandidates[i].edges[3];

        cv::rectangle(finalImg, cv::Point(x1, y1), cv::Point(x2, y2), colors[i], 2);
        result.push_back(bestCandidates[i]);
    }

    if (!quiet) {
        cout << "** The count for each cluster is [";
        for (unsigned int i = 0; i < clusterCount.size() - 1; ++i) {
            cout << clusterCount[i] << ", ";
        }
        cout << clusterCount.back() << "]" << endl;
    }

    // display the center of the cluster only for the clusters that have candidates
    for (int i = 0; i < nrClusters; ++i) {
        if (clusterCount[i] > 0) {
            int x = static_cast<int>(centers->data.fl[i * dim + 0]);
            int y = static_cast<int>(centers->data.fl[i * dim + 1]);
            cv::circle(clusterImg, cv::Point(x, y), 2, colors[i], CV_FILLED);
            cv::circle(finalImg, cv::Point(x, y), 2, colors[i], CV_FILLED);
        }
    }

    cv::imwrite("./img/depthClusters.png", clusterImg);
    cv::imwrite("./img/depthOpenings.png", finalImg);

    cvReleaseMat(&points);
    cvReleaseMat(&centers);
    cvReleaseMat(&clusters);
}

void model::Scene::addFinalOpenings(const LabeledPlane3d& surf,
        std::vector<CandidateOpening>& result)
{
    int imgHeight = static_cast<int>(surf.depthMap.size());
    int imgWidth  = static_cast<int>(surf.depthMap.front().size());

    vector<CandidateOpening> openings, candidates;
    this->detectPotentialOpenings(surf, openings);
    this->clusterOpenings(surf, openings, candidates);

    // sweep across the wall and see which candidates overlap
    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            // put currently overlapping candidates here
            vector<pair<vector<CandidateOpening>::iterator, double> > temp;

            // the maximum opening area in the current overlapping windows
            double bestEmptyArea = numeric_limits<double>::min();

            for (vector<CandidateOpening>::iterator it = candidates.begin(); it < candidates.end(); ++it) {
                int y1 = it->edges[0];
                int y2 = it->edges[1];
                int x1 = it->edges[2];
                int x2 = it->edges[3];

                if (i >= y1 && i < y2 && j >= x1 && j <= x2) {
                    // TODO avoid hard coding
                    double freeArea = it->features[0] *
                            (it->features[9] - it->features[10] - it->features[11]);
                    temp.push_back(make_pair(it, freeArea));

                    if (bestEmptyArea < freeArea) {
                        bestEmptyArea = freeArea;
                    }
                }
            }

            for (vector<pair<vector<CandidateOpening>::iterator, double> >::iterator it = temp.begin(); it < temp.end(); ++it) {
                // if the area is smaller than the maximum discard this plane
                if (it->second < bestEmptyArea) {
                    candidates.erase(it->first);
                }
            }
        }
    }

    // display an image with the final openings
    int r, g, b;
    cv::Mat finalOpeningsImg = surf.depthImg.clone();
    cv::cvtColor(finalOpeningsImg, finalOpeningsImg, CV_GRAY2BGR, 3);

    for (vector<CandidateOpening>::iterator it = candidates.begin(); it < candidates.end(); ++it) {
        int y1 = it->edges[0];
        int y2 = it->edges[1];
        int x1 = it->edges[2];
        int x2 = it->edges[3];

        randomColor(MAX_IMG_VAL, r, g, b);
        cv::rectangle(finalOpeningsImg, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(b, g, r), 3);

        // add the new openings as windows
        Point3d pt(0.0, 0.0, 0.0);
        pt += surf.patches[y1].front().first;
        pt += surf.patches[y2].front().first;
        pt += surf.patches.front()[x1].first;
        pt += surf.patches.front()[x2].first;
        pt /= 4.0;

        vector<Point3d> hull;
        hull.push_back(surf.patches[y1][x1].first);
        hull.push_back(surf.patches[y1][x2].first);
        hull.push_back(surf.patches[y2][x2].first);
        hull.push_back(surf.patches[y2][x1].first);

        Plane3d toPush(pt, surf.normal, hull);
        this->finalOpenings.push_back(toPush);
    }

    cv::imwrite("./img/depthOpeningsFinal.png", finalOpeningsImg);
    if (!quiet) cout << "** Final openings found on current surface: " << candidates.size() << endl;
    if (!quiet) cout << "** Final openings found so far: " << this->finalOpenings.size() << endl;

    // copy the result
    result = candidates;
}

void model::Scene::correct(LabeledPlane3d& surf, const vector<CandidateOpening>& openings) {
    if (!quiet) cout << endl << "== Correcting surface centered at " << surf.pt << endl;

    int imgHeight = static_cast<int>(surf.depthMap.size());
    int imgWidth  = static_cast<int>(surf.depthMap.front().size());

    // some images we are interested in
    cv::Mat depthImg = surf.depthImg.clone();
    cv::Mat depthImgMask(imgHeight, imgWidth, CV_8UC1);
    cv::Mat labelsImg(imgHeight, imgWidth, CV_8UC3);

    // compute the maximum from the image
    double max = numeric_limits<double>::min();
    double mean = 0.0, stdDev = 0.0;
    int count = 0;

    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            // set all empty and occluded to unknown
            if (surf.patches[i][j].second == EMPTY || surf.patches[i][j].second == OCCLUDED) {
                surf.patches[i][j].second = UNKOWN;
            // compute the mean of the occupied patches
            } else if (surf.patches[i][j].second == OCCUPIED) {
                mean += surf.depthImg.data[i * surf.depthImg.cols + j];
                count++;
            }

            // compute the maximum from the image
            if (max < surf.depthMap[i][j]) {
                max = surf.depthMap[i][j];
            }
        }
    }

    // adjust the mean
    mean /= count;
    count = 0;

    // compute the standard deviation
    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            if (surf.patches[i][j].second == OCCUPIED) {
                stdDev += pow(surf.depthImg.data[i * surf.depthImg.cols + j] - mean, 2.0);
                count++;
            }

        }
    }

    // adjust the standard deviation
    stdDev = sqrt(stdDev / count);

    // mark all patches contained in openings
    for (vector<CandidateOpening>::const_iterator it = openings.begin(); it != openings.end(); ++it) {
        int y1 = it->edges[0];
        int y2 = it->edges[1];
        int x1 = it->edges[2];
        int x2 = it->edges[3];

        for (int i = 0; i < imgHeight; ++i) {
            for (int j = 0; j < imgWidth; ++j) {
                if (i >= y1 && i <= y2 && j >= x1 && j <= x2) {
                    surf.patches[i][j].second = OPENING;
                }
            }
        }
    }

    // adjust the image
    for (int i = 0; i < imgHeight; ++i) {
        for (int j = 0; j < imgWidth; ++j) {
            // initially mark as not part of the mask
            depthImgMask.data[i * depthImgMask.cols + j] = 0;

            // put the color into the image according to the label
            int curr = (i * 3 * labelsImg.cols) + 3 * j;
            switch (surf.patches[i][j].second) {
            case OPENING:
                labelsImg.data[curr + 0] = 200;
                labelsImg.data[curr + 1] = 200;
                labelsImg.data[curr + 2] = 200;
                break;
            case UNKOWN:
                labelsImg.data[curr + 0] = 50;
                labelsImg.data[curr + 1] = 50;
                labelsImg.data[curr + 2] = 50;

                // add pixel to mask for inpainting
                depthImgMask.data[i * depthImgMask.cols + j] = 200;
                break;
            case OCCUPIED:
                labelsImg.data[curr + 0] = 0;
                labelsImg.data[curr + 1] = 0;
                labelsImg.data[curr + 2] = 200;
                break;
            default:
                throw runtime_error("invalid default branch taken, possibly EMPTY or OCCLUDED patches");
                break;
            }
        }
    }

    cv::imwrite("./img/labelsWithOpenings.png", labelsImg);
    cv::imwrite("./img/depthMask.png", depthImgMask);
    cv::imwrite("./img/depthSimple.png", depthImg);

    // TODO add values as part of class
    if (!quiet) cout << "** Filling in missing data using inpaint algorithm" << endl;
    cv::inpaint(depthImg, depthImgMask, depthImg, 0.05 * (imgWidth + imgHeight), cv::INPAINT_NS);
    cv::imwrite("./img/depthInpaint.png", depthImg);

    // add some gaussian noise to make the image look more real
    if (!quiet) cout << "** Adding gaussian noise: mean = " << mean << ", stdDev = " << stdDev << endl;
    cv::Mat noise(imgHeight, imgWidth, CV_8UC1);
    cv::randn(noise, mean, stdDev);
    cv::addWeighted(depthImg, 0.9, noise, 0.1, 0.0, depthImg);
    cv::GaussianBlur(depthImg, depthImg, cv::Size(3, 3), 0.5, 0.5);
    cv::imwrite("./img/depthInpaintNoise.png", depthImg);
    surf.correctedDepthImg = depthImg.clone();

    // TODO apply the inverse transform from 2D to 3D and create a new point cloud using UOS RGB
}

void model::Scene::writeModel(string dir) {
    if (!fileIsDir(dir)) {
        throw runtime_error(dir + " is not a directory");
    }

    string outputDir;

    if (dir[dir.length()-1] == '/') {
        outputDir = dir + "model";
    }
    else {
        outputDir = dir + "/model";
    }

    int ret = mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

    if (ret != 0 && errno != EEXIST) {
        throw runtime_error("unable to create directory " + outputDir);
    }

    if (!quiet) cout << endl << "== Writing planes to " << outputDir << " directory..." << endl;

    string planesListFile = outputDir + "/planes.list";
    ofstream list(planesListFile.c_str(), ios::out);

    for (unsigned int counter = 0; counter < this->walls.size() + this->finalOpenings.size() + 2; ++counter) {
        Plane3d currPlane; // the plane to be printed
        string type;       // the objet's type, floor, ceiling, wall or opening

        if (counter < this->walls.size()) {
            currPlane = this->walls[counter];
            type = "Wall";
        }
        else if (counter == this->walls.size()) {
            currPlane = this->ceiling;
            type = "Ceiling";
        }
        else if (counter == this->walls.size() + 1) {
            currPlane = this->floor;
            type = "Floor";
        } else {
            // TODO find a better solution
            currPlane = this->finalOpenings[counter - (this->walls.size() + 2)];
            currPlane.normal.normalize();
            for (vector<Point3d>::iterator it = currPlane.hull.begin(); it != currPlane.hull.end(); ++it) {
                it->x += currPlane.normal.x;
                it->y += currPlane.normal.y;
                it->z += currPlane.normal.z;
            }
            type = "Opening";
        }

        stringstream ss;
        ss << outputDir << "/scan" << setfill('0') << setw(3) << counter << ".3d";
        ofstream file(ss.str().c_str(), ios::out);


        // add the plane to the list of planes
        list << type << " " << ss.str() << endl;
        boost::algorithm::to_lower(type);
        if (!quiet) cout << "** Writing " << type << ss.str() << endl;

        // add the points to the scan file
        for (vector<Point3d>::iterator it = currPlane.hull.begin();
                it != currPlane.hull.end(); ++it)
        {
            file << *it << endl;
        }

        file.close();
    }

    list.close();
}

void model::Scene::writeCorrectedWalls(std::string dir) {
    if (!fileIsDir(dir)) {
        throw runtime_error(dir + " is not a directory");
    }

    string outputDir;

    if (dir[dir.length()-1] == '/') {
        outputDir = dir + "cloud";
    }
    else {
        outputDir = dir + "/cloud";
    }

    int ret = mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

    if (ret != 0 && errno != EEXIST) {
        throw runtime_error("unable to create directory " + outputDir);
    }

    if (!quiet) cout << endl << "== Writing corrected points to " << outputDir << " directory..." << endl;

    // write the pose file
    string poseFileName = outputDir + "/scan000.pose";
    ofstream out(poseFileName.c_str(), ios::out);
    out << "0 0 0" << endl;
    out << "0 0 0" << endl;
    out.close();

    string pointsFileName = outputDir + "/scan000.3d";
    out.open(pointsFileName.c_str(), ios::out);

    // fetch each points and its color
    vector<pair<Point3d, cv::Vec3i> > points;

    getCorrectedWall(this->ceiling, points);
    for (vector<pair<Point3d, cv::Vec3i> >::iterator it = points.begin(); it != points.end(); ++it) {
        out << it->first << " " << static_cast<int>(it->second[0]) << " " << static_cast<int>(it->second[1]) << " " << static_cast<int>(it->second[2]) << endl;
    }

    getCorrectedWall(this->floor, points);
    for (vector<pair<Point3d, cv::Vec3i> >::iterator it = points.begin(); it != points.end(); ++it) {
        out << it->first << " " << static_cast<int>(it->second[0]) << " " << static_cast<int>(it->second[1]) << " " << static_cast<int>(it->second[2]) << endl;
    }

    for (vector<LabeledPlane3d>::iterator wall = this->walls.begin(); wall != this->walls.end(); ++wall) {
        getCorrectedWall(*wall, points);
        for (vector<pair<Point3d, cv::Vec3i> >::iterator it = points.begin(); it != points.end(); ++it) {
            out << it->first << " " << static_cast<int>(it->second[0]) << " " << static_cast<int>(it->second[1]) << " " << static_cast<int>(it->second[2]) << endl;
        }
    }

    out.close();
}

void model::Scene::getCorrectedWall(const LabeledPlane3d& surf, vector<pair<Point3d, cv::Vec3i> >& points) {
    if (!quiet) cout << "** Computing point cloud for surface centered at " << surf.pt << endl;

    // clear the output
    points.clear();

    if (surf.correctedDepthImg.empty()) {
        if (!quiet) cout << "WARNING: the surface centered at " << surf.pt << " has not been corrected yet" << endl;
        return;
    }

    if ((size_t) surf.correctedDepthImg.rows != surf.patches.size() ||
            (size_t) surf.correctedDepthImg.cols != surf.patches.front().size())
    {
        throw runtime_error("corrected image must be the same size as depth map");
    }

    // compute the maximum of the depth image
    int depthImgMax = 0;
    for (size_t i = 0; i < surf.depthMap.size(); ++i) {
        for (size_t j = 0; j < surf.depthMap.front().size(); ++j) {
            if (depthImgMax < surf.depthMap[i][j]) {
                depthImgMax = surf.depthMap[i][j];
            }
        }
    }

    // make the original points darker than the corrected ones
    int r = 255, g = 50, b = 50;
    int wr = 255, wg = 150, wb = 150;

    for (int i = 0; i < surf.correctedDepthImg.rows; ++i) {
        for (int j = 0; j < surf.correctedDepthImg.cols; ++j) {
            cv::Vec3i color;

            // only consider occupied and unknown pixels
            if (surf.patches[i][j].second == OCCUPIED) {
                color = cv::Vec3i(r, g, b);
            } else if (surf.patches[i][j].second == UNKOWN) {
                color = cv::Vec3i(wr, wg, wb);
            } else {
                continue;
            }

            // the value from the depth map
            double depth = static_cast<double>(depthImgMax * surf.correctedDepthImg.data[i * surf.correctedDepthImg.cols + j]) / MAX_IMG_VAL;

            // the distance from where the raytracing took place to the point it hit
            double dist = depth + surf.depthMapDistances.first - surf.depthMapDistances.second;

            Vector3d normal = surf.normal;
            normal.normalize();

            Point3d pt = surf.patches[i][j].first;
            pt.x = pt.x + normal.x * dist;
            pt.y = pt.y + normal.y * dist;
            pt.z = pt.z + normal.z * dist;

            points.push_back(make_pair(pt, color));
        }
    }
}

void model::Scene::flood(LabeledPlane3d& surf,
        const int& i, const int& j,
        const Label& target, const Label& replacement)
{
    // no point of carrying on
    if (target == replacement) {
        return;
    }

    pair<int, int> north = make_pair(i-1, j);
    pair<int, int> south = make_pair(i+1, j);
    pair<int, int> east = make_pair(i, j+1);
    pair<int, int> west = make_pair(i, j-1);

    vector<pair<int, int> > neigh;
    neigh.push_back(north);
    neigh.push_back(south);
    neigh.push_back(east);
    neigh.push_back(west);

    if (surf.patches[i][j].second == target) {
        surf.patches[i][j].second = replacement;
    }

    for (vector<pair<int, int> >::iterator it = neigh.begin(); it != neigh.end(); ++it) {
        if ((it->first > 0 && it->second > 0) &&
                surf.patches[it->first][it->second].second == target)
        {
            flood(surf, it->first, it->second, target, replacement);
        }
    }
}

bool model::Scene::isOccupied(const Point3d& center, const double& width) {
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
