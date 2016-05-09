//
// Created by Joschka van der Lucht on 17.09.15.
//

#include "calibration/CalibrationToolbox.h"
#include "calibration/Chessboard.h"
#include <boost/filesystem.hpp>


using namespace cv;
using namespace std;

CalibrationToolbox::CalibrationToolbox(Settings &s) :
        settings(s),
        pattern(),
        imagePath(s.picturePath),
        pictureHandler(s.decimate, s.blur, s.threads, s.debug, s.refine_edges, s.refine_decodes, s.refine_pose,
                       s.tagFamily) {
    calcBoardCornerPositions(settings.calibrationPattern);
    int count = 1;
    for (string path : this->imagePath) {
        this->initImage(path);
        cout << "initImage succeed (" << path << ")" << endl;
        cout << "read picture and matchTags succeed \npicture " << count << "/" << this->imagePath.size() << "\n" << endl;
        count++;
    }

}

void CalibrationToolbox::matchTags() {
    vector<AprilTag2f> imagePoints = this->pictureHandler.getPointList();
    vector<AprilTag3f> patternPoints = this->pattern.getPoints();

    vector<Point2f> imgPoints;
    vector<Point3f> patPoints;

    vector<Point2f> imgEstPoints;
    vector<Point3f> patEstPoints;

    for (AprilTag2f aprilTag2f : imagePoints) {
        for (AprilTag3f aprilTag3f : patternPoints) {
            if (aprilTag3f.compair(aprilTag2f) == 0) {
                imgPoints.push_back(aprilTag2f.point1);
                imgPoints.push_back(aprilTag2f.point2);
                imgPoints.push_back(aprilTag2f.point3);
                imgPoints.push_back(aprilTag2f.point4);

                patPoints.push_back(aprilTag3f.point1);
                patPoints.push_back(aprilTag3f.point2);
                patPoints.push_back(aprilTag3f.point3);
                patPoints.push_back(aprilTag3f.point4);

                //Punkte mit ID zwischen minEstID und maxEstID
                if (find(settings.estIDs.begin(), settings.estIDs.end(), aprilTag2f.id) != settings.estIDs.end()) {
                    imgEstPoints.push_back(aprilTag2f.point1);
                    imgEstPoints.push_back(aprilTag2f.point2);
                    imgEstPoints.push_back(aprilTag2f.point3);
                    imgEstPoints.push_back(aprilTag2f.point4);

                    patEstPoints.push_back(Point3f(aprilTag3f.point1.x, aprilTag3f.point1.y, 0));
                    patEstPoints.push_back(Point3f(aprilTag3f.point2.x, aprilTag3f.point1.y, 0));
                    patEstPoints.push_back(Point3f(aprilTag3f.point3.x, aprilTag3f.point1.y, 0));
                    patEstPoints.push_back(Point3f(aprilTag3f.point4.x, aprilTag3f.point1.y, 0));
                }
                break;
            }
        }
    }
    if (imgPoints.size() > 4 && patPoints.size() > 4) {
        this->vecImagePoints.push_back(imgPoints);
        this->vecPatternPoints.push_back(patPoints);

        if (imgEstPoints.size() > 4) {
            this->estimateImagePoints.push_back(imgEstPoints);
            this->estimatePatternPoints.push_back(patEstPoints);
        }
    }
}

CalibrationToolbox::~CalibrationToolbox() {

}

CalibrationPattern CalibrationToolbox::getPattern() {
    return this->pattern;
}

PictureHandler CalibrationToolbox::getPictureHandler() {
    return this->pictureHandler;
}

void CalibrationToolbox::initImage(string path) {
    this->pictureHandler.clearPointlist();
    if(settings.calibrationPattern == Settings::APRILTAG) {
        if (FILE *detecFile = fopen((path + ".detections").c_str(), "r")) {
            fclose(detecFile);
            this->pictureHandler.setPointList(AprilTag::createAprilTag2fFromFile((path + ".detections")));
            cout << "points load from file" << endl;
        } else {
            this->pictureHandler.loadImage(path);
            this->pictureHandler.detectTags(Settings::APRILTAG, settings);
            fstream f;
            f.open((path + ".detections").c_str(), ios::out);
            f << "#APRIL_2D" << endl;
            for (AprilTag2f tag : this->pictureHandler.getPointList()) {

                f << tag.toString();

            }
            f.close();
            cout << "count of detected Points: " << this->pictureHandler.getPointList().size() << endl;
        }

        if (this->settings.visualize && this->pictureHandler.getPointList().size() > 0) {
            static int i = 0;

            Mat image = imread(path);
            cvtColor(image, image, CV_BGR2GRAY);
            cvtColor(image, image, CV_GRAY2BGR);

            std::stringstream imgss;
            imgss << settings.visualizePath << "image" << i << ".png";
            cv::imwrite(imgss.str(), image);

            for (AprilTag2f tag : this->pictureHandler.getPointList()) {
                circle(image, tag.point1, 1, Scalar(0, 0, 255), 2);
                circle(image, tag.point2, 1, Scalar(0, 255, 0), 2);
                circle(image, tag.point3, 1, Scalar(255, 0, 0), 2);
                circle(image, tag.point4, 1, Scalar(0, 255, 255), 2);
            }

            std::stringstream detss;
            detss << settings.visualizePath << "detections" << i << ".png";
            cv::imwrite(detss.str(), image);

            i++;
        }
        this->matchTags();
    } else if(settings.calibrationPattern == Settings::CHESSBOARD){
        if (FILE *detecFile = fopen((path + ".detections").c_str(), "r")) {
            fclose(detecFile);
            this->vecImagePoints.push_back(Chessboard::readPoint2fChessboardFromFile(path + ".detections"));
            cout << "points load from file" << endl;
        } else {
            this->pictureHandler.loadImage(path);
            int found = this->pictureHandler.detectTags(Settings::CHESSBOARD, settings);
            if (found) {
                this->vecImagePoints.push_back(this->pictureHandler.getPoint2fVec());
                this->vecPatternPoints.push_back(this->chessboardCorners);
                fstream f;
                f.open((path + ".detections").c_str(), ios::out);
                f << "#CHESSBOARD" << endl;
                f << Chessboard::toString(this->pictureHandler.getPoint2fVec());
                f.close();
                cout << "chessboard detected" << endl;
            }else{
                cout << "no chessboard detected" << endl;
            }
        }

    }else if(settings.calibrationPattern == Settings::CIRCLES_GRID){
        this->pictureHandler.loadImage(path);
        int found = this->pictureHandler.detectTags(Settings::CIRCLES_GRID, settings);
        if (found) {
            this->vecImagePoints.push_back(this->pictureHandler.getPoint2fVec());
            this->vecPatternPoints.push_back(this->chessboardCorners);
        }

    }else if(settings.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID){
        this->pictureHandler.loadImage(path);
        int found = this->pictureHandler.detectTags(Settings::ASYMMETRIC_CIRCLES_GRID, settings);
        if (found) {
            this->vecImagePoints.push_back(this->pictureHandler.getPoint2fVec());
            this->vecPatternPoints.push_back(this->chessboardCorners);
        }
    }
}

int CalibrationToolbox::calibrate() {
    Mat cameraMatrix, distCoeffs;

    //int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    //clock_t prevTimestamp = 0;
    //const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
    //const char ESC_KEY = 27;
    Mat image = imread(imagePath[0], CV_LOAD_IMAGE_COLOR);
    Size imageSize = image.size();

    // Draw the corners.
    //drawChessboardCorners( image, settings.boardSize, Mat(this->vecPatternPoints), true );
    runCalibrationAndSave(settings, imageSize, cameraMatrix, distCoeffs, vecImagePoints);

    return 0;
}

double CalibrationToolbox::computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints,
                                                     const vector<vector<Point2f> > &imagePoints,
                                                     const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                                     const Mat &cameraMatrix, const Mat &distCoeffs,
                                                     vector<float> &perViewErrors) {
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int) objectPoints.size(); ++i) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                      distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int) objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

void CalibrationToolbox::calcBoardCornerPositions(Settings::Pattern patternType) {
    //corners.clear();
    switch (patternType) {
        case Settings::APRILTAG:
            this->pattern.readPattern(this->settings.patternPath, settings);
            break;
        case Settings::CHESSBOARD:
        case Settings::CIRCLES_GRID:
            for (int i = 0; i < settings.boardSize.height; ++i)
                for (int j = 0; j < settings.boardSize.width; ++j)
                    chessboardCorners.push_back(Point3f(j * settings.squareSize, i * settings.squareSize, 0));
            break;

        case Settings::ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < settings.boardSize.height; i++)
                for (int j = 0; j < settings.boardSize.width; j++)
                    chessboardCorners.push_back(Point3f((2 * j + i % 2) * settings.squareSize,i * settings.squareSize, 0));
            break;
        default:
            break;
    }
    cout << "initPattern succeed\n" << endl;
}

bool CalibrationToolbox::runCalibration(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                                        vector<vector<Point2f> > imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                                        vector<float> &reprojErrs, double &totalAvgErr) {

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<vector<Point3f> > objectPoints = this->vecPatternPoints;

    int flags = 0;

    if (settings.pattern == Settings::APRIL_3D) {
        if(settings.estFromInput){
            cameraMatrix = s.estCameraMatrix;
        } else {
            estimateInitial3DCameraMatrix(s, imagePoints, objectPoints, cameraMatrix, imageSize);
        }
        flags |= CV_CALIB_USE_INTRINSIC_GUESS;
    }

    cout << "objectPoints size: " << objectPoints.size() << "; imagePoints size: " << imagePoints.size() << endl;
    //Find intrinsic and extrinsic camera parameters

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                                 flags);

    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    if (this->settings.visualize) {
        Mat pointsImage = Mat_<cv::Vec3b>::zeros(imageSize.height, imageSize.width);

        for (int i = 0; i < (int) objectPoints.size(); i++) {
            std::stringstream imgss;
            imgss << settings.visualizePath << "/image" << i << ".png";
            Mat reprojImage = cv::imread(imgss.str());

            vector<Point2f> imagePoints2;
            projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
            double err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

            for (Point2f p : imagePoints[i]) {
                circle(reprojImage, p, 1, Scalar(0, 0, 255), 2);

                circle(pointsImage, p, 1, Scalar(255, 255, 255), 2);
            }

            for (Point2f p : imagePoints2) {
                circle(reprojImage, p, 1, Scalar(0, 255, 0), 2);
            }

            std::stringstream ss;
            ss << settings.visualizePath << "/reprojection" << i << ".png";
            cv::imwrite(ss.str(), reprojImage);
        }

        cv::imwrite(settings.visualizePath + "/points.png", pointsImage);
    }

    return ok;
}

// Print camera parameters to the output file
void CalibrationToolbox::saveCameraParams(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                                          const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                          const vector<float> &reprojErrs, const vector<vector<Point2f> > &imagePoints,
                                          double totalAvgErr, bool estimation) {
    FileStorage fs(s.outputFileName, FileStorage::WRITE);
    if (estimation) {
        fs = FileStorage((s.estimationXML), FileStorage::WRITE);
    }

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_Time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nrOfFrames" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
        fs << "FixAspectRatio" << s.aspectRatio;

    if (s.flag) {
        sprintf(buf, "flags: %s%s%s%s",
                s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
        cvWriteComment(*fs, buf, 0);

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if (!reprojErrs.empty())
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if (!rvecs.empty() && !tvecs.empty()) {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int) rvecs.size(); i++) {
            Mat r = bigmat(Range(i, i + 1), Range(0, 3));
            Mat t = bigmat(Range(i, i + 1), Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if (!imagePoints.empty()) {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int) imagePoints.size(); i++) {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

bool CalibrationToolbox::runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                                               vector<vector<Point2f> > imagePoints) {
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
    << ". avg re projection error = " << totalAvgErr;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs,
                         imagePoints, totalAvgErr, false);
    return ok;
}

void CalibrationToolbox::estimateInitial3DCameraMatrix(Settings &s, vector<vector<Point2f>> imagePoints,
                                                       vector<vector<Point3f>> patternPoints, Mat &cameraMatrix,
                                                       Size &imageSize) {
    vector<Mat> rvecs, tvecs;
    Mat distCoeffs;
    vector<float> reprojErrs;
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    double totalAvgErr = 0;
    if (s.picturesForDstimateInitial3DCameraMatrix > estimateImagePoints.size())
        s.picturesForDstimateInitial3DCameraMatrix = estimateImagePoints.size();
    while (this->estimateImagePoints.size() > s.picturesForDstimateInitial3DCameraMatrix &&
           this->estimatePatternPoints.size() > s.picturesForDstimateInitial3DCameraMatrix) {
        this->estimateImagePoints.pop_back();
        this->estimatePatternPoints.pop_back();
    }


    double estRMS = calibrateCamera(this->estimatePatternPoints, this->estimateImagePoints, imageSize, cameraMatrix,
                                    distCoeffs, rvecs, tvecs, s.flag | CV_CALIB_FIX_K1| CV_CALIB_FIX_K2| CV_CALIB_FIX_K3| CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
    totalAvgErr = computeReprojectionErrors(this->estimatePatternPoints, this->estimateImagePoints, rvecs, tvecs,
                                            cameraMatrix, distCoeffs, reprojErrs);
    cout << "Re-projection error for estimate: " << estRMS << endl;
    saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr, true);
    if(std::isnan(estRMS)){
        cameraMatrix.at<double>(0, 0) = 1000.0;
        cameraMatrix.at<double>(1, 1) = 1000.0;
        cameraMatrix.at<double>(0, 2) = imageSize.width / 2;
        cameraMatrix.at<double>(1, 2) = imageSize.height / 2;
        cameraMatrix.at<double>(2, 2) = 1.0;
    }
}

int CalibrationToolbox::computeExtrinsic() {
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    Mat cameraMatrix = settings.estCameraMatrix;
    Mat distCoeff = settings.estDistCoeff;
    double totalAvgErr = 0;
    Mat image = imread(imagePath[0], CV_LOAD_IMAGE_COLOR);
    Size imageSize = image.size();
    for(int i = 0; i < vecImagePoints.size(); i++){
        Mat rvec, tvec;
        solvePnP(vecPatternPoints[i], vecImagePoints[i], cameraMatrix, distCoeff, rvec, tvec, false, CV_ITERATIVE);
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }
    saveCameraParams(settings, imageSize, settings.estCameraMatrix, settings.estDistCoeff, rvecs, tvecs, reprojErrs,vecImagePoints, totalAvgErr, false);

    return 1;
}
