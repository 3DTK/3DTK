
#include "slam6d/fbr/feature_drawer.h"

using namespace std;

namespace fbr{

  // Functions to draw keypoints and matches.
  void feature_drawer::_DrawKeypoint( cv::Mat& img, const cv::KeyPoint& p, const cv::Scalar& color, int flags )
  {
    CV_Assert( !img.empty() );
    cv::Point center( cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier) );
    
    if( flags & cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS )
      {
        int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter

        // draw the circles around keypoints with the keypoints size
        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );

        // draw orientation of the keypoint, if it is applicable
        if( p.angle != -1 )
	  {
            float srcAngleRad = p.angle*(float)CV_PI/180.f;
	    cv::Point orient( cvRound( cos(srcAngleRad)*radius ),
			      cvRound(-sin(srcAngleRad)*radius ) // "-" to invert orientation of axis y
			      );
            line( img, center, center+orient, color, 1, CV_AA, draw_shift_bits );
	  }
#if 0
        else
	  {
            // draw center with R=1
            int radius = 1 * draw_multiplier;
            circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
	  }
#endif
      }
    else
      {
        // draw center with R=3
        int radius = 3 * draw_multiplier;
        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
      }
  }

  void feature_drawer::DrawKeypoints( const cv::Mat& image, const vector<cv::KeyPoint>& keypoints, cv::Mat& outImage,
				      const cv::Scalar& _color, int flags )
  {
    if( !(flags & cv::DrawMatchesFlags::DRAW_OVER_OUTIMG) )
      {
        if( image.type() == CV_8UC3 )
	  {
            image.copyTo( outImage );
	  }
        else if( image.type() == CV_8UC1 )
	  {
            cvtColor( image, outImage, CV_GRAY2BGR );
	  }
        else
	  {
            CV_Error( CV_StsBadArg, "Incorrect type of input image.\n" );
	  }
      }

    cv::RNG& rng=cv::theRNG();
    bool isRandColor = _color == cv::Scalar::all(-1);

    CV_Assert( !outImage.empty() );
    vector<cv::KeyPoint>::const_iterator it = keypoints.begin(),
      end = keypoints.end();
    for( ; it != end; ++it )
      {
	cv::Scalar color = isRandColor ? cv::Scalar(rng(256), rng(256), rng(256)) : _color;
        _DrawKeypoint( outImage, *it, color, flags );
      }
  }

  void feature_drawer::_PrepareImgAndDrawKeypoints( const cv::Mat& img1, const vector<cv::KeyPoint>& keypoints1,
						    const cv::Mat& img2, const vector<cv::KeyPoint>& keypoints2,
						    cv::Mat& outImg, cv::Mat& outImg1, cv::Mat& outImg2,
						    const cv::Scalar& singlePointColor, int flags )
  {
    //Size size( img1.cols + img2.cols, MAX(img1.rows, img2.rows) );
    cv::Size size(MAX(img1.cols, img2.cols), img1.rows + img2.rows );
    if( flags & cv::DrawMatchesFlags::DRAW_OVER_OUTIMG )
      {
        if( size.width > outImg.cols || size.height > outImg.rows )
	  CV_Error( CV_StsBadSize, "outImg has size less than need to draw img1 and img2 together" );
        outImg1 = outImg( cv::Rect(0, 0, img1.cols, img1.rows) );
        outImg2 = outImg( cv::Rect(img1.cols, 0, img2.cols, img2.rows) );
      }
    else
      {
        outImg.create( size, CV_MAKETYPE(img1.depth(), 3) );
        outImg1 = outImg( cv::Rect(0, 0, img1.cols, img1.rows) );
	//outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
	outImg2 = outImg( cv::Rect(0, img1.rows, img2.cols, img2.rows) );

        if( img1.type() == CV_8U )
	  cvtColor( img1, outImg1, CV_GRAY2BGR );
        else
	  img1.copyTo( outImg1 );

        if( img2.type() == CV_8U )
	  cvtColor( img2, outImg2, CV_GRAY2BGR );
        else
	  img2.copyTo( outImg2 );
      }

    // draw keypoints
    if( !(flags & cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )
      {
	cv::Mat outImg1 = outImg( cv::Rect(0, 0, img1.cols, img1.rows) );
	DrawKeypoints( outImg1, keypoints1, outImg1, singlePointColor, flags + cv::DrawMatchesFlags::DRAW_OVER_OUTIMG );

        //Mat outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
	cv::Mat outImg2 = outImg( cv::Rect(0, img1.rows, img2.cols, img2.rows) );
        DrawKeypoints( outImg2, keypoints2, outImg2, singlePointColor, flags + cv::DrawMatchesFlags::DRAW_OVER_OUTIMG );
      }
  }

  void feature_drawer::_DrawMatch( cv::Mat& outImg, cv::Mat& outImg1, cv::Mat& outImg2 ,
				   const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Scalar& matchColor, int flags )
  {
    cv::RNG& rng = cv::theRNG();
    bool isRandMatchColor = matchColor == cv::Scalar::all(-1);
    cv::Scalar color = isRandMatchColor ? cv::Scalar( rng(256), rng(256), rng(256) ) : matchColor;

    _DrawKeypoint( outImg1, kp1, color, flags );
    _DrawKeypoint( outImg2, kp2, color, flags );

    cv::Point2f pt1 = kp1.pt;
    cv::Point2f pt2 = kp2.pt;
    cv::Point2f dpt2 = cv::Point2f( pt2.x, min(pt2.y+outImg1.rows, float(outImg.rows-1)) );
    //dpt2 = Point2f( std::min(pt2.x+outImg1.cols, float(outImg.cols-1)), pt2.y );

    line( outImg, 
	  cv::Point(cvRound(pt1.x*draw_multiplier), cvRound(pt1.y*draw_multiplier)),
	  cv::Point(cvRound(dpt2.x*draw_multiplier), cvRound(dpt2.y*draw_multiplier)),
          color, 4, CV_AA, draw_shift_bits );
  }

  void feature_drawer::DrawMatches( const cv::Mat& img1, const vector<cv::KeyPoint>& keypoints1,
				    const cv::Mat& img2, const vector<cv::KeyPoint>& keypoints2,
				    const vector<cv::DMatch>& matches1to2, cv::Mat& outImg,
				    const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
				    const vector<char>& matchesMask, int flags )
  {
    if( !matchesMask.empty() && matchesMask.size() != matches1to2.size() )
      CV_Error( CV_StsBadSize, "matchesMask must have the same size as matches1to2" );

    cv::Mat outImg1, outImg2;
    _PrepareImgAndDrawKeypoints( img1, keypoints1, img2, keypoints2,
                                 outImg, outImg1, outImg2, singlePointColor, flags );

    // draw matches
    for( size_t m = 0; m < matches1to2.size(); m++ )
      {
        int i1 = matches1to2[m].queryIdx;
        int i2 = matches1to2[m].trainIdx;
        if( matchesMask.empty() || matchesMask[m] )
	  {
	    const cv::KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
            _DrawMatch( outImg, outImg1, outImg2, kp1, kp2, matchColor, flags );
	  }
      }
  }

  void feature_drawer::DrawMatches( const cv::Mat& img1, const vector<cv::KeyPoint>& keypoints1,
				    const cv::Mat& img2, const vector<cv::KeyPoint>& keypoints2,
				    const vector<vector<cv::DMatch> >& matches1to2, cv::Mat& outImg,
				    const cv::Scalar& matchColor, const cv::Scalar& singlePointColor,
				    const vector<vector<char> >& matchesMask, int flags )
  {
    if( !matchesMask.empty() && matchesMask.size() != matches1to2.size() )
      CV_Error( CV_StsBadSize, "matchesMask must have the same size as matches1to2" );

    cv::Mat outImg1, outImg2;
    _PrepareImgAndDrawKeypoints( img1, keypoints1, img2, keypoints2,
                                 outImg, outImg1, outImg2, singlePointColor, flags );

    // draw matches
    for( size_t i = 0; i < matches1to2.size(); i++ )
      {
        for( size_t j = 0; j < matches1to2[i].size(); j++ )
	  {
            int i1 = matches1to2[i][j].queryIdx;
            int i2 = matches1to2[i][j].trainIdx;
            if( matchesMask.empty() || matchesMask[i][j] )
	      {
		const cv::KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
                _DrawMatch( outImg, outImg1, outImg2, kp1, kp2, matchColor, flags );
	      }
	  }
      }
  }
}
