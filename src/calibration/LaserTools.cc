//
// Created by Joschka van der Lucht on 17.10.16.
//

#include "calibration/LaserTools.h"




LaserTools::LaserTools(Settings &s):
        settings(s),
        pattern(),
        pictureHandler(s.decimate, s.blur, s.threads, s.debug, s.refine_edges, s.refine_decodes, s.refine_pose, s.tagFamily) {

}

void LaserTools::detectTags(vector<vector<int>> planeIDs) {
    pattern = CalibrationPattern();
    pattern.readPattern(settings.patternPath, settings);
    //init image and dectect tags
    int count = 0;
    for(std::string path : this->settings.picturePath){
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

            //sort tags by image and planes
            vector<vector<Point2f>> planesPointsImage;
            vector<vector<Point3f>> planesPointsPattern;
            for(vector<int> plane : planeIDs){
                vector<Point2f> planeImage;
                vector<Point3f> planePattern;
                for(int id : plane){
                    //match pattern and image tags
                    for(AprilTag2f aprilTag2f : this->pictureHandler.getPointList()) {
                        for (AprilTag3f aprilTag3f : this->pattern.getPoints()) {
                            if (aprilTag3f.id == id && aprilTag3f.compair(aprilTag2f) == 0) {
                                planeImage.push_back(aprilTag2f.point1);
                                planeImage.push_back(aprilTag2f.point2);
                                planeImage.push_back(aprilTag2f.point3);
                                planeImage.push_back(aprilTag2f.point4);

                                planePattern.push_back(aprilTag3f.point1);
                                planePattern.push_back(aprilTag3f.point2);
                                planePattern.push_back(aprilTag3f.point3);
                                planePattern.push_back(aprilTag3f.point4);
                                break;
                            }
                        }
                    }
                }
                planesPointsImage.push_back(planeImage);
                planesPointsPattern.push_back(planePattern);
            }
            imageListWithPoints.push_back(planesPointsImage);
            patternListWithPoints.push_back(planesPointsPattern);

        } else{
            cout << "patterntype not implemented" << endl;
            return;
        }
        count ++;
        cout << "picture " << count << "/" << settings.picturePath.size() << endl;
        cout << "detectTags succeed (" << path << ")" << endl;
    }
}

int LaserTools::computeExtrinsic() {
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    Mat image = imread(settings.picturePath[0], CV_LOAD_IMAGE_COLOR);
    Size imageSize = image.size();

    for(int j = 0; j < imageListWithPoints.size(); j++) {
        vector<Mat> rvectorPictures;
        vector<Mat> tvectorPictures;
        for (int i = 0; i < imageListWithPoints[j].size(); i++) {
            Mat rvec, tvec;
            solvePnP(patternListWithPoints[j][i], imageListWithPoints[j][i], settings.estCameraMatrix, settings.estDistCoeff, rvec,
                     tvec, false, CV_ITERATIVE);
            rvectorPictures.push_back(rvec);
            tvectorPictures.push_back(tvec);
        }
        rvector.push_back(rvectorPictures);
        tvector.push_back(tvectorPictures);
    }
    //TODO: use Rodrigues() to convert rvetor to rotation matrix

    return 1;
}

void LaserTools::printMatrix() {
    for(int i = 0; i < rvector.size(); i++){
        cout << "picture " << i+1 << endl;
        for(int j = 0; j < rvector[i].size(); j++){
            Mat r, t;
            Rodrigues(rvector[i][j],r);
            cout << "plane " << j+1 << endl;
            cout << "rotation:\n" << r << "\n" << endl;
            cout << "translation:\n" << tvector[i][j] << "\n\n" << endl;
        }
        cout << endl;
    }
}

void LaserTools::calculateEquation() {
    for(int i = 0; i < rvector.size(); i++){
        cout << "picture " << i+1 << endl;
        for(int j = 0; j < rvector[i].size(); j++){
            Mat r;
            Rodrigues(rvector[i][j],r);
            cout << "plane " << j+1 << endl;

            Mat z = (Mat_<double>(3,1) <<0, 0, 1);
            Mat n = r * z;
            float d = norm(Vec3f(tvector[i][j]));

            cout << n << endl;
            //cout << tvector[i][j] << endl;
            cout << d << endl;
            cout << endl;
        }
    }
}