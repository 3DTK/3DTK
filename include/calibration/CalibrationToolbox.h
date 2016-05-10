//
// Created by Joschka van der Lucht on 17.09.15.
//

#ifndef CAMERACALIBRATIONTOOL_CALIBRATIONTOOLBOX_H
#define CAMERACALIBRATIONTOOL_CALIBRATIONTOOLBOX_H

/**
 * Punkte mit AprilTag einlesen und in einem Vector nach ID's ordnen. Point2f
 * Punkte des Pattern in einem Vector, nach ID's geordnet einstellen. Point3f
 * eventuell nicht erkannte IDs aus dem Bild auch im Pattern löschen.
 * mit OpenCV kalibrieren und Ergebniss als XML speichern
 *
 */

#include "CalibrationPattern.h"
#include "PictureHandler.h"
#include "calibration/Settings.h"


enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

/**
 *
 */
class CalibrationToolbox {

private:

    /**
     * vector of vector Point3d for detected tag points in picture
     */
    vector<vector<Point2f>> vecImagePoints;

    /**
     * vector of pattern tag points
     */
    vector<vector<Point3f>> vecPatternPoints;

    vector<vector<Point2f>> estimateImagePoints;
    vector<vector<Point3f>> estimatePatternPoints;

    vector<Point3f> chessboardCorners;

    /**
     * Pattern handler
     */
    CalibrationPattern pattern;

    /**
     * image handler
     */
    PictureHandler pictureHandler;

    /**
     * image path
     */
    vector<string> imagePath;

    /**
     * Settings.h with config for calibration
     */
    Settings settings;

    Mat camMatrix;

    vector<Mat> rvector;
    vector<Mat> tvector;
    Mat distorCoeff;

    /**
     * match the detected tags form image and pattern
     */
    void matchTags();

    /**
     *
     */
    void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                               const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                               const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                                               double totalAvgErr, bool estimation );

    /**
     *
     */
    bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                             vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                                             vector<float>& reprojErrs,  double& totalAvgErr);

    void calcBoardCornerPositions(Settings::Pattern patternType);

    double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                                          const vector<vector<Point2f> >& imagePoints,
                                                          const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                                          const Mat& cameraMatrix , const Mat& distCoeffs,
                                                          vector<float>& perViewErrors);

    bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints );

    void initImage(string path);

    void estimateInitial3DCameraMatrix(Settings& s, vector<vector<Point2f>> imagePoints,vector<vector<Point3f>> patternPoints, Mat& cameraMatrix, Size& imageSize );

public:

    /**
     * constructor
     * @param String picturePath path to image
     */
    CalibrationToolbox(Settings &s);

    /**
     *  deconstructor
     */
    ~CalibrationToolbox();

    /**
     * Getter for pattern
     * return CalibrationPattern pattern
     */
    CalibrationPattern getPattern();

    /**
     * getter for PictureHandler
     * return PictureHandler pictureHandler
     */
    PictureHandler getPictureHandler();

    /**
     * start calibration
     */
    int calibrate();

    /**
     * compute extrinsic parameters
     */
    int computeExtrinsic();

    /**
     * draw detected points to an image
     */
    void visualize(bool readCameraParamFromFile);
};


#endif //CAMERACALIBRATIONTOOL_CALIBRATIONTOOLBOX_H
