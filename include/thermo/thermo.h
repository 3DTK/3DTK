#ifndef __THERMO_H__
#define __THERMO_H__

#if (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#include <opencv/highgui.h>
#else
#include <opencv2/opencv.hpp>
#endif
//#include <opencv2/highgui.hpp>
#include <string>
#include <slam6d/scan.h>
//typedef vector<vector<float> > Float2D[1200][1600];
typedef std::vector<std::vector<float> > Float2D[2592][3888];

void calcBoard(std::vector<std::vector<double> > &point_array, int board_n, double &x, double &y, double &cx, double &cy, bool pc);
void sortBlobs(std::vector<std::vector<double> > &point_array, int board_n, int board_h, int board_w, bool quiet);
IplImage* detectBlobs(IplImage *org_image, int &corner_exp, int board_h, int board_w, bool quiet, std::vector<std::vector<double> > &point_array2);
void drawLines(std::vector<std::vector<double> > &point_array2, int corner_exp, IplImage *image, bool color=false);
IplImage* resizeImage(IplImage *source, int scale);
IplImage* detectCorners(IplImage *orgimage, int &corner_exp, int board_h, int board_w, bool quiet, double point_array2[][2], int scale=1);
void CalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, std::string dir, int scale=1);
void writeCalibParam(int images, int corner_exp, int board_w, CvMat* image_points, CvSize size, std::string dir);

void loadIntrinsicCalibration(CvMat * &intrinsic, CvMat * &distortion, std::string dir, bool optical=false) ;
void loadExtrinsicCalibration(CvMat * &Translation, CvMat * &Rotation, std::string dir, int method, bool optical=false) ;
void ProjectAndMap(int start, int end, bool optical, bool quiet, std::string dir,
IOType type, int scale, double rot_angle, double minDist, double maxDist,
bool correction, int neighborhood, int method=0);

bool readPoints(std::string filename, CvPoint3D32f *corners, int size) ;
void sortElementByElement(CvMat * vectors, int nr_elems, int nr_vectors);
void calculateExtrinsicsWithReprojectionCheck(CvMat * points2D, CvMat *
points3D, CvMat * rotation_vectors_temp, CvMat * translation_vectors_temp, CvMat
* distortions, CvMat * instrinsics, int corners, int successes, std::string dir, bool quiet=true, std::string substring = "") ;
void calculateExtrinsics(CvMat * rotation_vectors_temp, CvMat * translation_vectors_temp, int successes, std::string dir, bool quiet=true, std::string substring = "") ;
void CorrectErrorAndWrite(Float2D &data, std::fstream &outfile, CvSize size, bool optical);
void clusterSearch(float ** points, int size, double thresh1, double thresh2, std::fstream &outfile, bool optical);
void sortDistances(float ** points, int size);
void ExtrCalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, std::string dir, int scale=1);


#endif
