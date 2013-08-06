/**
 * @file registration.h
 * @brife register two scans.
 * This class register two scans with respect to the matched features.
 * It determines the transformation matrix.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @date Date: 2012/05/30 1:00
 */

#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include "fbr_global.h"
#include "slam6d/icp6Dquat.h"

using namespace std;

namespace fbr{
  /**
   * @class registration : registers two scnas
   * @param minDistance threshold for min distance between the each three points for registration process 
   * @param minError threshold for min error after transformation of a point from second coordinate to first to determin the inliers
   * @param minInlier threshold fir min inlier to consider the align as positive
   * @param rMethod registration Method
   * @param bAlign best alignment
   * @param bError error of registration with bAlign
   * @param bErrorIndex error index of registration with bAlign
   */
  class registration{
    double minDistance;
    double minError;
    unsigned int minInlier;
    registration_method rMethod;
    double bestAlign[16];
    double bestError;
    unsigned int bestErrorIndex;
    
    /**
     * getCoord : get 3D coordinate of query scan and train scan for each match
     * @param fKeypoints vector of KeyPoints from query (first) scan
     * @param sKeypoints vector of KeyPoints from train (second) scan
     * @param matches vector<DMatch> holding found matches
     * @param fPMap Mat containing the 3D coordinate of each match from query scan
     * @param sPMap Mat containing the 3D coordinate of each match from train scan
     * @param idx index of the match
     * @param cq Point3f for returning the 3D coordinate of query scan
     * @param ct Point3f for returning the 3D coordinate of train scan
     * @return 1 on success 0 on failure
     */
    int getCoord( vector<cv::KeyPoint> fKeypoints, vector<cv::KeyPoint> sKeypoints, vector<cv::DMatch> matches, cv::Mat fPMap, cv::Mat sPMap, int idx, cv::Point3f& cq, cv::Point3f& ct);
    /**
     * pointToArray : convert 3D point3f to double array
     * @param c 3D point whith Point3f type
     * @param cd returning 3d point as double array
     */
    void pointToArray(cv::Point3f c, double* cd);
    /**
     * coordTransform : transform a point with the align
     * @param p input train 3D point
     * @param align input transformation matrix
     * @return Point3f trasformed 3D point
     */
    cv::Point3f coordTransform(cv::Point3f p, double* align);
    /**
     * findAlign : find the best align
     * @param fPMap input Mat wich contains the 3D coordinates of each pixel of first panorama image
     * @param fKeypoints vector<KeyPoints> containing the features of first panorama image
     * @param sPMap input Mat wich contains the 3D coordinates of each pixel of second panorama image
     * @param sKeypoints vector<KeyPoints> containing the features of second panorama image
     * @param matches vector<DMatches> containing the matched features of both images
     */
    int findAlign(unsigned int i, unsigned int j, unsigned int k, cv::Mat fPMap, vector<cv::KeyPoint> fKeypoints, cv::Mat sPMap, vector<cv::KeyPoint> sKeypoints, vector<cv::DMatch> matches);

  public:
    registration();
    registration(double minD, double minE, unsigned int minI, registration_method method);
    /**
     * findRegistration : find the transformation matrix between two scans
     * @param fPMap first panorama map which is 3D points coresponding to fist panorama image
     * @param fKeypoints keypoints from the first scan
     * @param sPMap second panorama map which is 3D points coresponding to second panorama image
     * @param sKeypoints keypoints from the second scan
     * @param matches matched keypoints from first to second scan
     */
    void findRegistration(cv::Mat fPMap, vector<cv::KeyPoint> fKeypoints, cv::Mat sPMap, vector<cv::KeyPoint> sKeypoints, vector<cv::DMatch> matches);
    double getMinDistance();
    double getMinError();
    unsigned int getMinInlier();
    registration_method getRegistrationMethod();
    double * getBestAlign();
    double getBestError();
    unsigned int getBestErrorIndex();
    void getDescription();
  };
}
#endif /* REGISTRATION_H_ */
