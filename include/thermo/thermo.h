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
using namespace std;
//typedef vector<vector<float> > Float2D[1200][1600];
typedef vector<vector<float> > Float2D[2592][3888];

void calcBoard(vector<vector<double> > &point_array, int board_n, double &x, double &y, double &cx, double &cy, bool pc); 
void sortBlobs(vector<vector<double> > &point_array, int board_n, int board_h, int board_w, bool quiet); 
IplImage* detectBlobs(IplImage *org_image, int &corner_exp, int board_h, int board_w, bool quiet, vector<vector<double> > &point_array2);
void drawLines(vector<vector<double> > &point_array2, int corner_exp, IplImage *image, bool color=false);
IplImage* resizeImage(IplImage *source, int scale);
IplImage* detectCorners(IplImage *orgimage, int &corner_exp, int board_h, int board_w, bool quiet, double point_array2[][2], int scale=1);
void CalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, string dir, int scale=1);
void writeCalibParam(int images, int corner_exp, int board_w, CvMat* image_points, CvSize size, string dir);

void loadIntrinsicCalibration(CvMat * &intrinsic, CvMat * &distortion, string dir, bool optical=false) ;
void loadExtrinsicCalibration(CvMat * &Translation, CvMat * &Rotation, string dir, int method, bool optical=false) ;
void ProjectAndMap(int start, int end, bool optical, bool quiet, string dir,
IOType type, int scale, double rot_angle, double minDist, double maxDist,
bool correction, int neighborhood, int method=0);

bool readPoints(string filename, CvPoint3D32f *corners, int size) ;
void sortElementByElement(CvMat * vectors, int nr_elems, int nr_vectors); 
void calculateExtrinsicsWithReprojectionCheck(CvMat * points2D, CvMat *
points3D, CvMat * rotation_vectors_temp, CvMat * translation_vectors_temp, CvMat
* distortions, CvMat * instrinsics, int corners, int successes, string dir, bool quiet=true, string substring = "") ;
void calculateExtrinsics(CvMat * rotation_vectors_temp, CvMat * translation_vectors_temp, int successes, string dir, bool quiet=true, string substring = "") ;
void CorrectErrorAndWrite(Float2D &data, fstream &outfile, CvSize size, bool optical);
void clusterSearch(float ** points, int size, double thresh1, double thresh2, fstream &outfile, bool optical);
void sortDistances(float ** points, int size);
void ExtrCalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, string dir, int scale=1);


#endif
