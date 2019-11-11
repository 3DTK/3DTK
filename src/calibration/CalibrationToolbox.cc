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
        pictureHandler(s.decimate, s.blur, s.threads, s.debug, s.refine_edges, s.tagFamily) {
    if(s.calibrationPattern != Settings::FROM_FILES) {
        calcBoardCornerPositions(settings.calibrationPattern);
        int count = 1;
        for (string path : this->imagePath) {
            this->initImage(path);
            cout << "initImage succeed (" << path << ")" << endl;
            cout << "read picture and matchTags succeed \npicture " << count << "/" << this->imagePath.size() << "\n" <<
            endl;
            count++;
        }
    }else{
        int count = 1;
        for (string path : this->imagePath) {
            vector<Point2f> imgpoints;
            vector<Point3f> patpoints;
            ifstream pointfile;
            string line;
            pointfile.open(path, ios_base::in);
            if(!pointfile){
                cerr << "can't read: " << path << endl;
            }else{
                while (getline(pointfile, line)){
                    char* fEnd;
                    float xi,yi,xp,yp,zp;
                    xi = strtof (line.c_str(), &fEnd);
                    yi = strtof (fEnd, &fEnd);
                    xp = strtof (fEnd, &fEnd);
                    yp = strtof (fEnd, &fEnd);
                    zp = strtof (fEnd, NULL);
                    imgpoints.push_back(Point2f(xi,yi));
                    patpoints.push_back(Point3f(xp,yp,zp));
                    line = "";
                }
            }

            if(imgpoints.size() > 4 && patpoints.size() > 4){
                this->vecImagePoints.push_back(imgpoints);
                this->vecPatternPoints.push_back(patpoints);
            } else{
                cout << path << "\n has not enough points" << endl;
            }
            cout << "file read succeed (" << path << ")" << "\npicture " << count << "/" << this->imagePath.size() << "\n" << endl;
            count++;
        }
    }

}

bool CalibrationToolbox::matchTags() {
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
                if (settings.pattern == Settings::APRIL_3D && find(settings.estIDs.begin(), settings.estIDs.end(), aprilTag2f.id) != settings.estIDs.end()) {
                    imgEstPoints.push_back(aprilTag2f.point1);
                    imgEstPoints.push_back(aprilTag2f.point2);
                    imgEstPoints.push_back(aprilTag2f.point3);
                    imgEstPoints.push_back(aprilTag2f.point4);

                    patEstPoints.push_back(Point3f(aprilTag3f.point1.x, aprilTag3f.point1.y, 0));
                    patEstPoints.push_back(Point3f(aprilTag3f.point2.x, aprilTag3f.point2.y, 0));
                    patEstPoints.push_back(Point3f(aprilTag3f.point3.x, aprilTag3f.point3.y, 0));
                    patEstPoints.push_back(Point3f(aprilTag3f.point4.x, aprilTag3f.point4.y, 0));
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

        return true;
    }

    return false;
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

        if (this->matchTags()) {
            this->vecCalibImagePaths.push_back(path);
        }
    } else if(settings.calibrationPattern == Settings::CHESSBOARD){
        if (FILE *detecFile = fopen((path + ".detections").c_str(), "r")) {
            fclose(detecFile);
            this->vecImagePoints.push_back(Chessboard::readPoint2fChessboardFromFile(path + ".detections"));
            this->vecPatternPoints.push_back(this->chessboardCorners);
            this->vecCalibImagePaths.push_back(path);
            cout << "points load from file" << endl;
        } else {
            this->pictureHandler.loadImage(path);
            int found = this->pictureHandler.detectTags(Settings::CHESSBOARD, settings);
            if (found) {
                this->vecImagePoints.push_back(this->pictureHandler.getPoint2fVec());
                this->vecPatternPoints.push_back(this->chessboardCorners);
                this->vecCalibImagePaths.push_back(path);
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
            this->vecCalibImagePaths.push_back(path);
        }

    }else if(settings.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID){
        this->pictureHandler.loadImage(path);
        int found = this->pictureHandler.detectTags(Settings::ASYMMETRIC_CIRCLES_GRID, settings);
        if (found) {
            this->vecImagePoints.push_back(this->pictureHandler.getPoint2fVec());
            this->vecPatternPoints.push_back(this->chessboardCorners);
            this->vecCalibImagePaths.push_back(path);
        }
    }
}

int CalibrationToolbox::calibrate() {
    //Mat cameraMatrix, distCoeffs;

    //int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    //clock_t prevTimestamp = 0;
    //const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
    //const char ESC_KEY = 27;
    Mat image = imread(imagePath[0],
#if CV_MAJOR_VERSION > 2
			cv::ImreadModes::IMREAD_COLOR
#else
			CV_LOAD_IMAGE_COLOR
#endif
			);
    Size imageSize = image.size();
    this->settings.imageSize = imageSize;

    // Draw the corners.
    //drawChessboardCorners( image, settings.boardSize, Mat(this->vecPatternPoints), true );
    runCalibrationAndSave(settings, imageSize, camMatrix, distorCoeff, vecImagePoints);

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
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), cv::NORM_L2);

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
    if (s.flag & cv::CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<vector<Point3f> > objectPoints = this->vecPatternPoints;

    int flags = 0;

    if (settings.calibrationPattern == Settings::APRILTAG && settings.pattern == Settings::APRIL_3D) {
        if(settings.estFromInput){
            cameraMatrix = s.estCameraMatrix;
        } else {
            estimateInitial3DCameraMatrix(s, imagePoints, objectPoints, cameraMatrix, imageSize);
        }
        flags |= cv::CALIB_USE_INTRINSIC_GUESS;
    }

    cout << "objectPoints size: " << objectPoints.size() << "; imagePoints size: " << imagePoints.size() << endl;
    //Find intrinsic and extrinsic camera parameters

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                                 flags);

    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

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

    if (s.flag & cv::CALIB_FIX_ASPECT_RATIO)
        fs << "FixAspectRatio" << s.aspectRatio;

    if (s.flag) {
        sprintf(buf, "flags: %s%s%s%s",
                s.flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                s.flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                s.flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                s.flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
#if CV_MAJOR_VERSION > 2
        fs.writeComment(
#else
        cvWriteComment(*fs,
#endif
				buf, 0);

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
#if CV_MAJOR_VERSION > 2
        fs.writeComment(
#else
        cvWriteComment(*fs,
#endif
				"a set of 6-tuples (rotation vector + translation vector) for each view", 0);
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
    //vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvector, tvector,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
    << ". avg re projection error = " << totalAvgErr;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvector, tvector, reprojErrs,
                         imagePoints, totalAvgErr, false);
    return ok;
}

void CalibrationToolbox::estimateInitial3DCameraMatrix(Settings &s, vector<vector<Point2f>> imagePoints,
                                                       vector<vector<Point3f>> patternPoints, Mat &cameraMatrix,
                                                       Size &imageSize) {
    vector<Mat> rvecs, tvecs;
    Mat distCoeffs;
    vector<float> reprojErrs;
    distCoeffs = Mat::zeros(5, 1, CV_64F);
    double totalAvgErr = 0;
    if (s.picturesForDstimateInitial3DCameraMatrix > estimateImagePoints.size())
        s.picturesForDstimateInitial3DCameraMatrix = estimateImagePoints.size();
    while (this->estimateImagePoints.size() > s.picturesForDstimateInitial3DCameraMatrix &&
           this->estimatePatternPoints.size() > s.picturesForDstimateInitial3DCameraMatrix) {
        this->estimateImagePoints.pop_back();
        this->estimatePatternPoints.pop_back();
    }


    double estRMS = calibrateCamera(this->estimatePatternPoints, this->estimateImagePoints, imageSize, cameraMatrix,
                                    distCoeffs, rvecs, tvecs, cv::CALIB_FIX_K1| cv::CALIB_FIX_K2| cv::CALIB_FIX_K3| cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
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
    camMatrix = settings.estCameraMatrix;
    distorCoeff = settings.estDistCoeff;
    //vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    //Mat cameraMatrix = settings.estCameraMatrix;
    //Mat distCoeff = settings.estDistCoeff;
    double totalAvgErr = 0;
    Mat image = imread(imagePath[0],
#if CV_MAJOR_VERSION > 2
			cv::ImreadModes::IMREAD_COLOR
#else
			CV_LOAD_IMAGE_COLOR
#endif
			);
    Size imageSize = image.size();
    for(int i = 0; i < vecImagePoints.size(); i++){
        Mat rvec, tvec;
        solvePnP(vecPatternPoints[i], vecImagePoints[i], camMatrix, distorCoeff, rvec, tvec, false,
#if CV_MAJOR_VERSION > 2
				cv::SOLVEPNP_ITERATIVE
#else
				CV_ITERATIVE
#endif
				);
        rvector.push_back(rvec);
        tvector.push_back(tvec);
    }
    saveCameraParams(settings, imageSize, settings.estCameraMatrix, settings.estDistCoeff, rvector, tvector, reprojErrs,vecImagePoints, totalAvgErr, false);

    return 1;
}

void CalibrationToolbox::visualize(bool readCameraParamFromFile){
    if(readCameraParamFromFile){
        this->camMatrix = settings.estCameraMatrix;
        this->distorCoeff = settings.estDistCoeff;
    }

    boost::filesystem::create_directories(settings.visualizePath);

    Mat pointsImage = Mat_<cv::Vec3b>::zeros(settings.imageSize.height, settings.imageSize.width);
    for(int i = 0; i < this->vecCalibImagePaths.size(); i++) {
        boost::filesystem::path filename(this->vecCalibImagePaths[i]);
        //gefunden Punkte bgr
        Mat image = imread(this->vecCalibImagePaths[i]);
        vector<Point2f> imagePoints2;
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
        cvtColor(image, image, cv::COLOR_GRAY2BGR);

        std::stringstream detss;
        detss << settings.visualizePath << "/" << filename.filename().string() << ".detections" << ".png";
        std::stringstream ss;
        ss << settings.visualizePath << "/" << filename.filename().string() << ".reprojection" << ".png";

        uint colorIndex = 0;
        Scalar colorMap[4] = {Scalar(0,0,255), Scalar(0,255,0), Scalar(255,0,0), Scalar(0,255,255)};

        for(Point2f point2f : this->vecImagePoints[i]){
            circle(image, point2f, 1, colorMap[colorIndex],2);
            if(!readCameraParamFromFile) {
                projectPoints(Mat(vecPatternPoints[i]), rvector[i], tvector[i], camMatrix, distorCoeff, imagePoints2);
                double err = norm(Mat(vecImagePoints[i]), Mat(imagePoints2), cv::NORM_L2);
            }
            circle(pointsImage, point2f, 1, Scalar(255, 255, 255), 2);

            colorIndex++;
            if (colorIndex > 3) colorIndex = 0;
        }
        if(!readCameraParamFromFile) {
            cv::imwrite(detss.str(), image);
        }

        for (Point2f p : imagePoints2) {
            circle(image, p, 1, Scalar(0, 255, 0), 2);
        }
        cv::imwrite(ss.str(), image);
    }
    cv::imwrite(settings.visualizePath + "/points.png", pointsImage);
}
