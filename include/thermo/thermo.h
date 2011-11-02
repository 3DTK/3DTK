#ifndef __THERMO_H__
#define __THERMO_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include <slam6d/scan.h>
using namespace std;
typedef vector<vector<float> > Float2D[1200][1600];

void calcBoard(double point_array[][2], int board_n, double &x, double &y, double &cx, double &cy, bool pc);    
void sortCluster(double point_array[][2], int board_n, int board_h, int board_w, bool quiet); 
IplImage* detectBlobs(IplImage *org_image, int corner_exp, int board_h, int board_w, bool quiet, double point_array2[][2]);
void drawLines(double point_array2[][2], int corner_exp, IplImage *image, bool color=false);
IplImage* resizeImage(IplImage *source, int scale);
IplImage* detectCorners(IplImage *orgimage, int corner_exp, int board_h, int board_w, bool quiet, double point_array2[][2], int scale=1);
void CalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, string dir, int scale=1);
void writeCalibParam(int images, int corner_exp, int board_w, CvMat* image_points, CvSize size, string dir);

void ProjectAndMap(int start, int end, bool optical, bool quiet, string dir, reader_type type, int scale, double rot_angle, double minDist, double maxDist, bool correction, int neighborhood);

void readPoints(string filename, CvPoint3D32f *corners, int size) ;
void sortElementByElement(CvMat * vectors, int nr_elems, int nr_vectors); 
void calculateExtrinsics(CvMat * rotation_vectors_temp, CvMat * translation_vectors_temp, int successes, string dir, bool quiet=true, string substring = "") ;
void CorrectErrorAndWrite(Float2D &data, fstream &outfile, CvSize size);
bool clusterSearch(float ** points, int size, double thresh1, double thresh2, fstream &outfile);
void sortDistances(float ** points, int size);
void ExtrCalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, string dir, int scale=1);


#endif
