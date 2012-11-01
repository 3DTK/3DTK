/*
 * David Scanner implementation
 *
 * Copyright (C) Vladislav Perelman
 *
 * Released under the GPL version 3.
 *
 */

/*
 * david_scanner.cc
 * Program takes as an input path to the config file which needs to
 * have all the necessary information for the program.
 * Config file has to have (each on a new line, 9 lines in total):
 *
 * Path to the directory where frames from the video are stored
 * The first frame that has to be used
 * The last frame that has to be used
 * The empty frame without the laser
 * Path to the file with intrinsics of the camera
 * Path to the rotation of the left board
 * Path to the rotation of the right board
 * Path to the translation of the left board
 * Path to the translation of the right board
 *
 * Program computes the 3 point cloud of the object and stores it in the
 * file scan000.3d, each point in the cloud is represented by the line
 * in the file:
 * x y z r g b
 *
 *
 *  Created on: Oct 4, 2010
 *  Author: Vladislav Perelman v.perelman@jacobs-university.de
 */

#include <iostream>
#include <string>
#include <fstream>

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxcore.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

#include <math.h>
#include <vector>
#define PI 3.14159265
using namespace std;

int main(int argc, char** argv){

	if (argc!=2){
		cout<<"USAGE: david_scanner config_file\nConfig file should contain path_to_frames first_valid_frame last_valid_frame empty_frame path_to_intrinsics"
				"path_to_rotation_left path_to_rotation_right path_to_translation_left and path_to_translation_right each on a new line!"<<endl;
		return -1;
	}
	//******Reading Input********
	ifstream indata;
	indata.open(argv[1]);
	if (!indata){
		cout<<"Config file could not be opened"<<endl;
		return -1;
	}
	string line;
	int numlines=0;
	while( getline(indata, line) ) numlines++;

	if (numlines != 9) {
		cout<<"Invalid number of lines in a config file!\nConfig file should contain path_to_frames first_valid_frame last_valid_frame empty_frame path_to_intrinsics"
					"path_to_rotation_left path_to_rotation_right path_to_translation_left and path_to_translation_right each on a new line!";
		return -1;
	}
	indata.clear();
	indata.seekg(0);
	char path[200];
	indata.getline(path,200);

	char first_c[10];
	char last_c[10];
	char empty_c[10];

	indata.getline(first_c,10);
	indata.getline(last_c,10);
	indata.getline(empty_c,10);

	int first = atoi(first_c);
	int last = atoi(last_c);
	int empty = atoi(empty_c);

	char intrinsics_path[200];
	char rot_left[200];
	char rot_right[200];
	char tran_left[200];
	char tran_right[200];

	indata.getline(intrinsics_path,200);
	indata.getline(rot_left,200);
	indata.getline(rot_right,200);
	indata.getline(tran_left,200);
	indata.getline(tran_right,200);
	//*********done************

	//loading an empty frame
	IplImage* image_empty;
	IplImage* image;
	char empty_name[100];
	sprintf(empty_name,"%s/%08d.ppm",path,empty);
	if ((image_empty=cvLoadImage(empty_name,1))==NULL){
		cout<<"Cannot load empty frame...check input name"<<endl;
		return -1;
	}

	//*******LOADING CAMERA PARAMETERS + CREATING MATRICES FOR FUTURE USE*********

	CvMat *intrinsic = cvCreateMat(3,3,CV_32F);
	if ((intrinsic = (CvMat*)cvLoad( intrinsics_path ))==NULL){
		cout<<"Cannot load intrinsic parameters...check input path and file name"<<endl;
		return -1;
	}

	//loading R1
	CvMat* rotation_left = cvCreateMat(3,1,CV_32F);
	if ((rotation_left = (CvMat*)cvLoad( rot_left ))==NULL){
		cout<<"Cannot load rotation of the left board...check input"<<endl;
		return -1;
	}
	//loading T1
	CvMat* translation_left = cvCreateMat(3,1,CV_32F);
	if ((translation_left= (CvMat*)cvLoad( tran_left ))==NULL){
		cout<<"Cannot load translation of the left board...check input"<<endl;
		return -1;
	}
	CvMat* rotation_matrix_left = cvCreateMat( 3, 3, CV_32F );
	cvRodrigues2(rotation_left, rotation_matrix_left);

	//loading R2
	CvMat* rotation_right = cvCreateMat(3,1,CV_32F);
	if ((rotation_right = (CvMat*)cvLoad( rot_right ))==NULL){
		cout<<"Cannot load rotation of the right board...check input"<<endl;
		return -1;
	}
	//loading T2
	CvMat* translation_right = cvCreateMat(3,1,CV_32F);
	if((translation_right=(CvMat*)cvLoad( tran_right ))==NULL){
		cout<<"Cannot load translation of the right board...check input"<<endl;
		return -1;
	}
	CvMat* rotation_matrix_right = cvCreateMat( 3, 3, CV_32F );
	cvRodrigues2(rotation_right, rotation_matrix_right);

	//creating [R1|T1]
	CvMat* r1t1 = cvCreateMat( 3, 4, CV_32F );
	for (int i = 0; i < 3; i++){
		CV_MAT_ELEM( *r1t1, float, i, 0) = CV_MAT_ELEM( *rotation_matrix_left, float, i, 0);
		CV_MAT_ELEM( *r1t1, float, i, 1) = CV_MAT_ELEM( *rotation_matrix_left, float, i, 1);
		CV_MAT_ELEM( *r1t1, float, i, 2) = CV_MAT_ELEM( *rotation_matrix_left, float, i, 2);
		CV_MAT_ELEM( *r1t1, float, i, 3) = CV_MAT_ELEM( *translation_left, float, i, 0);
	}

	//creating [R2|T2]
	CvMat* r2t2 = cvCreateMat( 3, 4, CV_32F );
	for (int i = 0; i < 3; i++){
		CV_MAT_ELEM( *r2t2, float, i, 0) = CV_MAT_ELEM( *rotation_matrix_right, float, i, 0);
		CV_MAT_ELEM( *r2t2, float, i, 1) = CV_MAT_ELEM( *rotation_matrix_right, float, i, 1);
		CV_MAT_ELEM( *r2t2, float, i, 2) = CV_MAT_ELEM( *rotation_matrix_right, float, i, 2);
		CV_MAT_ELEM( *r2t2, float, i, 3) = CV_MAT_ELEM( *translation_right, float, i, 0);
	}

	//creating R1.i()
	CvMat* r1inv = cvCreateMat( 3, 3, CV_32F );
	cvInvert(rotation_matrix_left, r1inv);

	//creating A.i()
	CvMat* intrinsicinv = cvCreateMat( 3, 3, CV_32F );
	cvInvert(intrinsic, intrinsicinv);

	//creating R1.i()*A.i()
	CvMat* R1iAi = cvCreateMat( 3, 3, CV_32F );
	cvMatMul(r1inv, intrinsicinv, R1iAi);

	//creating R2.i()
	CvMat* r2inv = cvCreateMat( 3, 3, CV_32F );
	cvInvert(rotation_matrix_right, r2inv, CV_LU);

	//creating R2.i()*A.i()
	CvMat* R2iAi = cvCreateMat( 3, 3, CV_32F );
	cvMatMul(r2inv, intrinsicinv, R2iAi);

	//creating R1.i()*T1
	CvMat* a1 = cvCreateMat(3, 1, CV_32F);
	cvMatMul(r1inv, translation_left, a1);

	//creating R2.i()*T2
	CvMat* a2 = cvCreateMat(3, 1, CV_32F);
	cvMatMul(r2inv, translation_right, a2);

	//*****************DONE********************

	//open file for writing
	ofstream scanfile;
	char scanname[20];
	sprintf(scanname,"scan000.3d");
	scanfile.open(scanname);

	//for loop going through each frame in the provided folder between first_valid_frame and last_valid_frame
	for (int m=first; m<last; m++){
		char name[100];
		sprintf(name, "%s/%08d.ppm", path,m);
		cout<<name<<endl;
		if ((image =cvLoadImage(name))==NULL){
			cout<<"cannot load image: "<<name<<endl;
			continue;
		}
		//do difference between current frame and the empty frame with no laser
		IplImage* diff = cvCloneImage(image);
		cvAbsDiff(image_empty, image, diff);

		//focus on the red pixels, make others black
		unsigned char* pixels = (unsigned char*)diff->imageData;
		for (int row = 0; row < diff->height; row++){
			for (int col = 0; col < diff->width; col++){
				int R;
				R = pixels[ row * diff->widthStep + col * 3 + 2 ];
				if (R>30) {
					pixels[ row * diff->widthStep + col * 3 + 0 ] = 0;
					pixels[ row * diff->widthStep + col * 3 + 1 ] = 0;
					pixels[ row * diff->widthStep + col * 3 + 2 ] = 255;
				} else {
					pixels[ row * diff->widthStep + col * 3 + 0 ] = 0;
					pixels[ row * diff->widthStep + col * 3 + 1 ] = 0;
					pixels[ row * diff->widthStep + col * 3 + 2 ] = 0;
				}

			}
		}

		//remove pixels that don't have at least 2 red neighbors
		for (int row = 1; row < diff->height-1; row++){
			for (int col = 1; col < diff->width-1; col++){
				int R = pixels[ row * diff->widthStep + col * 3 + 2 ];
				if (R == 255){
					int r1 = pixels[ (row-1)*diff->widthStep + col * 3 + 2];
					int r2 = pixels[ (row-1)*diff->widthStep + (col-1) * 3 + 2];
					int r3 = pixels[ (row-1)*diff->widthStep + (col+1) * 3 + 2];
					int r4 = pixels[ (row+1)*diff->widthStep + col * 3 + 2];
					int r5 = pixels[ (row+1)*diff->widthStep + (col-1) * 3 + 2];
					int r6 = pixels[ (row+1)*diff->widthStep + (col+1) * 3 + 2];
					int r7 = pixels[ (row)*diff->widthStep + (col-1) * 3 + 2];
					int r8 = pixels[ (row)*diff->widthStep + (col+1) * 3 + 2];
					if (r1+r2+r3+r4+r5+r6+r7+r8<=255) pixels[ row * diff->widthStep + col * 3 + 2 ]=0;
				}
			}
		}

		//*****finding 2 lines on the image*****

		bool good = false;
		int threshold = 50; //original threshold for Hough transform, incremented if too many groups of lines found
		IplImage* color_dst;
		IplImage* tmpImage;
		int minX1, minX2, maxX1, maxX2;
		CvSeq* lines = 0;
		CvPoint* line1;
		CvPoint* line2;
		int count_groups;

		//incrementing thresholds until only 2 groups of lines can be found
		while(!good){
			good = true;
			count_groups = 0; //counter for number of line groups. Line group is defined by the slope
			int epsilon = 1.5; //error margin for the slope
			color_dst = cvCreateImage( cvGetSize(diff), 8, 3 );
			color_dst = cvCloneImage(diff);
			tmpImage = cvCreateImage(cvGetSize(diff), IPL_DEPTH_8U, 1);
			cvCvtColor(diff, tmpImage, CV_RGB2GRAY);
			IplImage* dst = cvCreateImage( cvGetSize(diff), 8, 1 );
			cvCanny(tmpImage, dst, 20, 60, 3 );
			CvMemStorage* storage = cvCreateMemStorage(0);
			//find all lines using Hough transform
			lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180,threshold, 150, 100 );
			double first_group, second_group;
			for(int i = 0; i < lines->total; i++ ){
				//get the slope of the line, check if it belongs to an already existing group
				CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
				double angle = atan((double)(line[1].x-line[0].x)/(double)(line[1].y-line[0].y))*180/PI;
				//starting first group
				if (count_groups==0){
					first_group = angle;
					line1 = line;
					minX1 = line[0].x;
					maxX1 = line[1].x;
					count_groups++;

				} else {

					if (angle-first_group<epsilon && angle-first_group>(epsilon*-1)){
						//line belongs to the first group of line..that's good
						if (line[0].x<minX1)minX1=line[0].x;
						if (line[1].x>maxX1)maxX1=line[1].x;
					} else {
						//check if belongs to the second group
						if ( count_groups == 2 ){
							if (angle-second_group<epsilon && angle - second_group>(epsilon*-1)){
								if (line[0].x<minX2)minX2=line[0].x;
								if (line[1].x>maxX2)maxX2=line[1].x;
							}else{
								//if not then try again with a higher threshold
								good = false;
								threshold+=20;
								cout<<"Increased threshold: "<<threshold<<" ";
								cvReleaseImage(&color_dst);
								cvReleaseImage(&tmpImage);
								cvReleaseImage(&dst);
								break;	 	 //get out of here and increase the threshold since too many lines were found
							}
						} else { //starting second group
							second_group = angle;
							minX2 = line[0].x;
							maxX2 = line[1].x;
							line2 = line;
							count_groups++;
						}
					}
				}

			}
			//freeing some memory along the way
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&dst);
		}
		//at this point we have found at most 2 groups of lines, we need to take only 1 line from each group
		//basically finding the left-most and right-most point of each group and draw a line between those points, removing all the other lines.
		//starting and ending points of 2 lines
		CvPoint point1;
		CvPoint point2;
		CvPoint point3;
		CvPoint point4;
		if (count_groups==2){

			int x1 = line1[0].x;
			int x2 = line1[1].x;
			int y1 = line1[0].y;
			int y2 = line1[1].y;

			double c1 = (double)(x1 - minX1)/(double)(x2 - minX1);
			double c2 = (double)(maxX1 - x1)/(double)(maxX1 - x2);
			int ymax, ymin;
			ymin = (c1*y2 - y1)/(c1-1);
			ymax = (c2*y2 - y1)/(c2-1);
			if (maxX1 == x2) ymax = y2;
			if (minX1 == x1) ymin = y1;
			//getting start and end of the first line
			point1 = cvPoint(minX1, ymin);
			point2 = cvPoint(maxX1, ymax);
			//points around all the lines in a group so that a black rectangle can be drawn above them
			CvPoint points[4];
			points[0]=cvPoint(minX1, max(0,ymin-10));
			points[1]=cvPoint(minX1, min(color_dst->height,ymin+10));
			points[2]=cvPoint(maxX1, min(color_dst->height,ymax+10));
			points[3]=cvPoint(maxX1, max(0,ymax-10));
			CvPoint* pts[1];
			pts[0]=points;
			int npts[1];
			npts[0]=4;
			cvPolyLine(color_dst, pts, npts,1,1, CV_RGB(0,0,0), 20, 8 );//removing the group

			x1 = line2[0].x;
			x2 = line2[1].x;
			y1 = line2[0].y;
			y2 = line2[1].y;

			c1 = (double)(x1 - minX2)/(double)(x2 - minX2);
			c2 = (double)(maxX2 - x1)/(double)(maxX2 - x2);

			ymin = (c1*y2 - y1)/(c1-1);
			ymax = (c2*y2 - y1)/(c2-1);

			if (maxX2 == x2) ymax = y2;
			if (minX2 == x1) ymin = y1;

			//getting start and end of the second line
			point3 = cvPoint(minX2, ymin);
			point4 = cvPoint(maxX2, ymax);

			points[0]=cvPoint(minX2, max(0,ymin-10));
			points[1]=cvPoint(minX2, min(color_dst->height,ymin+10));
			points[2]=cvPoint(maxX2, min(color_dst->height,ymax+10));
			points[3]=cvPoint(maxX2, max(0,ymax-10));

			pts[0]=points;

			cvPolyLine(color_dst, pts, npts,1,1, CV_RGB(0,0,0), 20, 8 );//removing the group
			cvLine(color_dst, point3, point4,CV_RGB(0,255,0),3, 8 ); //draw the second line!
			cvLine(color_dst, point1, point2,CV_RGB(0,255,0),3, 8 ); //draw the first line!

			//removing everything to the left of the left line and to the right of the right line
			if (point4.x > point2.x){
				if (color_dst->width > point4.x){
					cvRectangle(color_dst,cvPoint(point4.x,0),cvPoint(color_dst->width,color_dst->height),CV_RGB(0,0,0),CV_FILLED);
				}
				if (point1.x > 0){
					cvRectangle(color_dst,cvPoint(point1.x,0),cvPoint(0,color_dst->height),CV_RGB(0,0,0),CV_FILLED);
				}
			}
			if (point4.x < point2.x){
				if (color_dst->width > point2.x){
					cvRectangle(color_dst,cvPoint(point2.x,0),cvPoint(color_dst->width,color_dst->height),CV_RGB(0,0,0),CV_FILLED);
				}
				if (point3.x > 0){
					cvRectangle(color_dst,cvPoint(point3.x,0),cvPoint(0,color_dst->height),CV_RGB(0,0,0),CV_FILLED);
				}
			}
			//at this point we have to lines which we drew in green...which means all the red pixels that remain on the image
			//are supposed to be laying on the object. Make them blue (for no particular reason..just looked nicer :) )
			unsigned char* pixels = (unsigned char*)color_dst->imageData;

			for (int row = 1; row < color_dst->height-1; row++){
				for (int col = 1; col < color_dst->width-1; col++){
					int R = pixels[ row * color_dst->widthStep + col * 3 + 2 ];
					if (R == 255){
						pixels[ row * color_dst->widthStep + col * 3 + 0 ]=255;
						pixels[ row * color_dst->widthStep + col * 3 + 1 ]=0;
						pixels[ row * color_dst->widthStep + col * 3 + 2 ]=0;
					}
				}
			}

		} else continue;

		//take points on planes
		CvPoint left1, left2, right1;
		if (point1.x < point3.x){
			left1 = point1;
			left2 = point2;
			right1 = point3;
		} else {
			left1 = point3;
			left2 = point4;
			right1 = point1;
		}

		//find 3d coordinate of the 2 points on the line on the left plane
		//(x,y,z).t() = s*R.i()*A.i()*(u,v,1).t() - R.i()*T

		CvMat* imagepoint1 = cvCreateMat( 3, 1, CV_32F );
		CV_MAT_ELEM(*imagepoint1, float, 0, 0) = left1.x;
		CV_MAT_ELEM(*imagepoint1, float, 1, 0) = left1.y;
		CV_MAT_ELEM(*imagepoint1, float, 2, 0) = 1;


		CvMat* b1 = cvCreateMat(3, 1, CV_32F);
		cvMatMul(R1iAi, imagepoint1, b1);

		//calculate scalar s based on the fact that point we take is on the wall => z coordinate is 0
		float s1 = CV_MAT_ELEM(*a1, float, 2, 0)/CV_MAT_ELEM(*b1, float, 2, 0);

		CvMat* identity = cvCreateMat(3,3,CV_32F);
		cvSetIdentity(identity);
		for (int i = 0; i < 3; i++){
			CV_MAT_ELEM(*identity, float, i, i)=s1;
		}
		CvMat* temp = cvCreateMat(3,1,CV_32F);
		cvMatMul(identity,b1, temp);

		CvMat* dpoint1 = cvCreateMat(3,1,CV_32F);
		cvSub(temp, a1, dpoint1); //first 3d point on the left plane

		//same thing for the second point
		CvMat* imagepoint2 = cvCreateMat( 3, 1, CV_32F );

		CV_MAT_ELEM(*imagepoint2, float, 0, 0) = left2.x;
		CV_MAT_ELEM(*imagepoint2, float, 1, 0) = left2.y;
		CV_MAT_ELEM(*imagepoint2, float, 2, 0) = 1;

		CvMat* b2 = cvCreateMat(3, 1, CV_32F);
		cvMatMul(R1iAi, imagepoint2, b2);

		float s2 = CV_MAT_ELEM(*a1, float, 2, 0)/CV_MAT_ELEM(*b2, float, 2, 0);

		cvSetIdentity(identity, cvRealScalar(s2));
		cvMatMul(identity,b2, b2);

		CvMat* dpoint2 = cvCreateMat(3,1,CV_32F);
		cvSub(b2, a1, dpoint2); //second 3d point on the left plane

		//same for the point on the right plane
		CvMat* imagepoint3 = cvCreateMat( 3, 1, CV_32F );
		CV_MAT_ELEM(*imagepoint3, float, 0, 0) = right1.x;
		CV_MAT_ELEM(*imagepoint3, float, 1, 0) = right1.y;
		CV_MAT_ELEM(*imagepoint3, float, 2, 0) = 1;

		CvMat* b3 = cvCreateMat(3, 1, CV_32F);
		cvMatMul(R2iAi, imagepoint3, b3);

		float s3 = CV_MAT_ELEM(*a2, float, 2, 0)/CV_MAT_ELEM(*b3, float, 2, 0);

		cvSetIdentity(identity, cvRealScalar(s3));
		cvMatMul(identity,b3, b3);

		CvMat* dpoint3 = cvCreateMat(3,1,CV_32F);
		cvSub(b3, a2, dpoint3); //point on the right plane

		//convert point from the right plane into the coord. system of the left plane
		//p1 = R1.i()*[R2|T2]*p2 - R1.i()*T1
		CvMat* dpoint3left = cvCreateMat(3,1,CV_32F);
		CvMat* pw = cvCreateMat(4,1,CV_32F);
		for (int i = 0; i<3; i++){
			CV_MAT_ELEM(*pw, float, i, 0) = CV_MAT_ELEM(*dpoint3, float, i, 0);
		}
		CV_MAT_ELEM(*pw, float, 3, 0) = 1.0;
		CvMat* r2t2pw = cvCreateMat(3,1,CV_32F);
		cvMatMul(r2t2, pw, r2t2pw);
		CvMat* r1invr2t2pw = cvCreateMat(3,1,CV_32F);
		cvMatMul(r1inv, r2t2pw, r1invr2t2pw);

		cvSub(r1invr2t2pw, a1, dpoint3left);
		//now that we have 3 non-colinear point in the same coordinate system we can find the equation of the plane
		/*
       A = y1 (z2 - z3) + y2 (z3 - z1) + y3 (z1 - z2)
       B = z1 (x2 - x3) + z2 (x3 - x1) + z3 (x1 - x2)
       C = x1 (y2 - y3) + x2 (y3 - y1) + x3 (y1 - y2)
     - D = x1 (y2 z3 - y3 z2) + x2 (y3 z1 - y1 z3) + x3 (y1 z2 - y2 z1)
       */
		float x1 = CV_MAT_ELEM(*dpoint1, float,0,0);
		float y1 = CV_MAT_ELEM(*dpoint1, float,1,0);
		float z1 = CV_MAT_ELEM(*dpoint1, float,2,0);
		float x2 = CV_MAT_ELEM(*dpoint2, float,0,0);
		float y2 = CV_MAT_ELEM(*dpoint2, float,1,0);
		float z2 = CV_MAT_ELEM(*dpoint2, float,2,0);
		float x3 = CV_MAT_ELEM(*dpoint3left, float,0,0);
		float y3 = CV_MAT_ELEM(*dpoint3left, float,1,0);
		float z3 = CV_MAT_ELEM(*dpoint3left, float,2,0);
		float planeA = (y1 * (z2 - z3)) + (y2 * (z3 - z1)) + (y3 * (z1 - z2));
		float planeB = (z1 * (x2 - x3)) + (z2 * (x3 - x1)) + (z3 * (x1 - x2));
		float planeC = (x1 * (y2 - y3)) + (x2 * (y3 - y1)) + (x3 * (y1 - y2));
		float planeD = -((x1 * (y2 * z3 - y3 * z2)) + (x2 * (y3 * z1 - y1 * z3)) + (x3 * (y1 * z2 - y2 * z1)));

		//calculate normal to the lazer plane
		CvMat* planeNormal = cvCreateMat(3, 1, CV_32F);
		CV_MAT_ELEM(*planeNormal, float,0,0) = planeA;
		CV_MAT_ELEM(*planeNormal, float,1,0) = planeB;
		CV_MAT_ELEM(*planeNormal, float,2,0) = planeC;

		pixels = (unsigned char*)color_dst->imageData;
		unsigned char* color_pixels = (unsigned char*)image_empty->imageData;

		//go through all the pixels on the object and calculate the 3d coordinate
		for (int row = 1; row < color_dst->height-1; row++){
			for (int col = 1; col < color_dst->width-1; col++){
				int B = pixels[ row * color_dst->widthStep + col * 3];
				if (B == 255){
					//get RGB of the pixel on the original image
					int realB = color_pixels[ row * color_dst->widthStep + col * 3];
					int realG = color_pixels[ row * color_dst->widthStep + col * 3 + 1];
					int realR = color_pixels[ row * color_dst->widthStep + col * 3 + 2];
					//Used http://www.cs.princeton.edu/courses/archive/fall00/cs426/lectures/raycast/sld017.htm for reference
					//on how to find intersection of ray and a plane
					float p0dotN = cvDotProduct(a1,planeNormal);

					CvMat* vtmp = cvCreateMat(3,1,CV_32F);
					CV_MAT_ELEM(*vtmp, float,0,0) = col;
					CV_MAT_ELEM(*vtmp, float,1,0) = row;
					CV_MAT_ELEM(*vtmp, float,2,0) = 1;

					CvMat* v = cvCreateMat(3,1,CV_32F);
					cvMatMul(R1iAi, vtmp, v);

					float vdotN  = cvDotProduct(v,planeNormal);
					float t = (p0dotN - planeD)/vdotN;

					cvSetIdentity(identity, cvRealScalar(t));
					cvMatMul(identity,v,v);

					CvMat* final = cvCreateMat(3,1,CV_32F);
					cvSub(v,a1,final); //final point is still in the coordinate system of the left plane.

					CvMat* final_rotated = cvCreateMat(3,1,CV_32F); //translate it into the coordinate system of the camera
					cvMatMul(rotation_matrix_left,final,final_rotated);
					cvAdd(final_rotated,translation_left, final_rotated);

					//add point to the file (minus next to the y coordinate is there to compensate for the left-handed coordinate system of slam6d, otherwise
					//dwarf is shown upside-down.
					scanfile<<CV_MAT_ELEM(*final_rotated,float,0,0)<<" "<<-CV_MAT_ELEM(*final_rotated,float,1,0)<<" "<<CV_MAT_ELEM(*final_rotated,float,2,0)<<
							" "<< realR<<" "<<realG<<" "<<realB<<"\n";
					cvReleaseMat(&vtmp);
					cvReleaseMat(&v);
					cvReleaseMat(&final);
					cvReleaseMat(&final_rotated);

				}
			}
		}

		//save the image of the lines and points of the object
		char name2[100];
		sprintf(name2, "%s/%08d_diff.ppm", path,m);
		cvSaveImage(name2, color_dst);
		//free memory
		cvReleaseImage(&image);
		cvReleaseImage(&diff);
		cvReleaseImage(&color_dst);
		cvReleaseImage(&tmpImage);
		cvReleaseMat(&imagepoint1);
		cvReleaseMat(&imagepoint2);
		cvReleaseMat(&imagepoint3);
		cvReleaseMat(&b1);
		cvReleaseMat(&b2);
		cvReleaseMat(&b3);
		cvReleaseMat(&temp);
		cvReleaseMat(&dpoint1);
		cvReleaseMat(&dpoint2);
		cvReleaseMat(&dpoint3);
		cvReleaseMat(&dpoint3left);
		cvReleaseMat(&identity);
		cvReleaseMat(&pw);
		cvReleaseMat(&r2t2pw);
		cvReleaseMat(&r1invr2t2pw);
		cvReleaseMat(&planeNormal);
	}
	//free more memory
	cvReleaseImage(&image_empty);
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&intrinsicinv);
	cvReleaseMat(&rotation_left);
	cvReleaseMat(&rotation_matrix_left);
	cvReleaseMat(&rotation_right);
	cvReleaseMat(&rotation_matrix_right);
	cvReleaseMat(&translation_left);
	cvReleaseMat(&translation_right);
	cvReleaseMat(&r1inv);
	cvReleaseMat(&r2inv);
	cvReleaseMat(&R1iAi);
	cvReleaseMat(&R2iAi);
	cvReleaseMat(&r1t1);
	cvReleaseMat(&r2t2);
	cvReleaseMat(&a1);
	cvReleaseMat(&a2);

	//close file
	scanfile.close();
	return 0;
}
