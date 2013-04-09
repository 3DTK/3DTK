/**
 * @file feature_drawer.h
 * @brife draws features and matches on panorma images
 * This class is a feature drawer that draws the keypoints found on a scan on a panorama image of the scan.
 * It also draws the matches between two scan on panorama images and draw a line between them.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 */

#ifndef FEATURE_DRAWER_H_
#define FEATURE_DRAWER_H_

#include "fbr_global.h"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/internal.hpp"

namespace fbr{
  
  const int draw_shift_bits = 4;
  const int draw_multiplier = 1 << draw_shift_bits;
  /**
   * @class feature_drawer : draws keypoints on panorama images and also matches between two panorama image
   */
  class feature_drawer{
    
    void _DrawKeypoint( cv::Mat& img, const cv::KeyPoint& p, const cv::Scalar& color, int flags );
    void _PrepareImgAndDrawKeypoints( const cv::Mat& img1, const vector<cv::KeyPoint>& keypoints1,
				      const cv::Mat& img2, const vector<cv::KeyPoint>& keypoints2,
				      cv::Mat& outImg, cv::Mat& outImg1, cv::Mat& outImg2,
				      const cv::Scalar& singlePointColor, int flags );
    void _DrawMatch( cv::Mat& outImg, cv::Mat& outImg1, cv::Mat& outImg2 ,
		     const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Scalar& matchColor, int flags );
  public:
    void DrawKeypoints( const cv::Mat& image, const vector<cv::KeyPoint>& keypoints, cv::Mat& outImage,
			const cv::Scalar& _color, int flags );
    void DrawMatches( const cv::Mat& img1, const vector<cv::KeyPoint>& keypoints1,
		      const cv::Mat& img2, const vector<cv::KeyPoint>& keypoints2,
		      const vector<cv::DMatch>& matches1to2, cv::Mat& outImg,
		      const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
		      const vector<char>& matchesMask, int flags );
    void DrawMatches( const cv::Mat& img1, const vector<cv::KeyPoint>& keypoints1,
		      const cv::Mat& img2, const vector<cv::KeyPoint>& keypoints2,
		      const vector<vector<cv::DMatch> >& matches1to2, cv::Mat& outImg,
		      const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
		      const vector<vector<char> >& matchesMask, int flags );
  };
}
#endif /*FEATURE_DRAWER_H*/
