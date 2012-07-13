/**
 * @file labeledPlane3d.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 15 Apr 2012
 *
 */

#ifndef LABELEDPLANE3D_H_
#define LABELEDPLANE3D_H_

//==============================================================================
//  Includes
//==============================================================================
#include "model/plane3d.h"
#include "model/candidateOpening.h"
#include "model/commonTypes.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include <vector>

//==============================================================================
//  Global Variables
//==============================================================================
extern bool quiet;

//==============================================================================
//  Class Declarations
//==============================================================================
namespace model {

/**
 * The three possible labels on a wall, as a result of the raytracing algorithm.
 */
enum Label {
    EMPTY, OCCUPIED, OCCLUDED, OPENING, UNKOWN
};

/**
 * A class representing a plane that has additional features such as labeling of patches.
 */
class LabeledPlane3d : public Plane3d {
private:
    static const std::string RED;
    static const std::string GREEN;
    static const std::string BLUE;
    static const std::string WHITE;

    static const double LINE_EPSILON;        //!< An epislon value to decide whether a line is horizontal or vertical.

    static const double SOBEL_SCALE;    //!< To be used when applying Sobel to any image.
    static const double SOBEL_DELTA;    //!< To be used when applying Sobel to any image.
    static const int SOBEL_KERNEL;      //!< To be used when applying Sobel to any image.

    static const double CANNY_THRESH1;  //!< The lower threshold for Canny.
    static const double CANNY_THRESH2;  //!< The upper threshold for Canny.

    static const double HOUGH_RHO;          //!< Rho used in the Hough transform.
    static const double HOUGH_THETA;        //!< Theta used in the Hough transform.
    static const int HOUGH_THRESH;          //!< Hough transform threshold.
    static const double HOUGH_MIN_LINE_LEN; //!< Minimum line length for Hough transform.

    static const double MIN_EMPTY_AREA;     //!< Minimum empty area percentage.
    static const double MIN_TOTAL_AREA;     //!< Minimum total area percentage of wall.
    static const double MAX_TOTAL_AREA;     //!< Maximum total area percentage of wall.

private:
    std::string name;

public:
    /**
     * Each discrete patch is described in this matrix.
     */
    std::vector<std::vector<std::pair<model::Point3d, Label> > > patches;

    /**
     * Remember a depth map of the plane to be used when detecting the windows.
     * To be built in the same time when applying labels.
     */
    std::vector<std::vector<double> > depthMap;

    /**
     * The OpenCV image of the depthMap.
     */
    cv::Mat depthImg;

    /**
     * The corrected depth image of the wall;
     */
    cv::Mat correctedDepthImg;

    /**
     * The wall distance and the maximum distance (2 * wall distance + ray distance)
     * at which the depth map was computed.
     */
    std::pair<double, double> depthMapDistances;

    // constructores and destructors
    LabeledPlane3d();
    LabeledPlane3d(const Point3d& pt, const Vector3d& normal);
    LabeledPlane3d(const Point3d& pt, const Vector3d& normal, const std::vector<Point3d>& hull);

    LabeledPlane3d(const LabeledPlane3d& other);
    virtual ~LabeledPlane3d();

    // operators
    LabeledPlane3d& operator=(const LabeledPlane3d& other);

    /**
     * Computes the Canny for this wall using the depth image.
     */
    void detectEdges(cv::Mat& img, cv::Mat& hImg, cv::Mat& vImg, cv::Mat& combined) const;

    /**
     * Computes the horizontal and vertical lines.
     */
    void computeLines(std::vector<int>& verticalResult, std::vector<int>& horizontalResult) const;

    /**
     * Returns all opening candidates.
     */
    void computeOpeningCandidates(std::vector<CandidateOpening>& candidates) const;
};

} /* namespace model */

#endif /* LABELEDPLANE3D_H_ */
