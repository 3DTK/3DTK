#include <errno.h>
#include "thermo/thermo.h"
#include "newmat/newmatap.h"
using namespace NEWMAT;

#include "cvblob.h"
using namespace cvb;

#include <slam6d/globals.icc>

#ifndef _MSC_VER
#include <getopt.h>
#include <sys/stat.h>
#else
#include "XGetopt.h"
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <windows.h>
#include <direct.h>
#endif

#ifdef _EiC
#define WIN32
#endif

Float2D data1;
Float2D data2;

int n_boards = 0; //Will be set by input list total number of images for which calibration is t be performed
int board_w = 6;//default size can be changed when asked for
int board_h = 4;//default size can be change
int successes = 0;//number of boards in which the all the mentioned corners are successfully found and aproved by user

unsigned int BLOB_SIZE = 55;
double AVG_THRES = 0.8;
    
/*
void quickSort(double* sequence, int lower, int upper) {
  double tmp;
  int l = lower;
  int u = upper;
  int pivot = (upper + lower) / 2;
  double value = sequence[pivot];

  do {
    while(sequence[l] < value) l++;
    while(sequence[u] > value) u--;
    if(l <= u) {
      tmp = sequence[l];
      sequence[l] = sequence[u];
      sequence[u] = tmp;
      l++;
      u--;
    }
  } while(l <= u);

  if(lower < u) quickSort(sequence, lower, u);
  if(upper > l) quickSort(sequence, l, upper);
}
*/
void calcBoard(double point_array[][2], int board_n, double &x, double &y, double &cx, double &cy, bool pc) {    
  cx = cy = 0;
  for (int a = 0; a < board_n; a++) {
    cx += point_array[a][0];
    cy += point_array[a][1];
  }

  cx /= board_n;
  cy /= board_n;

  SymmetricMatrix A(2);
  A = 0;

  for(int a = 0; a < board_n; a++) {
    A(1,1) += (point_array[a][0] - cx)*(point_array[a][0] - cx);
    A(2,2) += (point_array[a][1] - cy)*(point_array[a][1] - cy);
    A(1,2) += (point_array[a][0] - cx)*(point_array[a][1] - cy);
  }    
  DiagonalMatrix D;
  Matrix V;
  try {
    Jacobi(A,D,V);
  } catch (ConvergenceException) {
    cout << "couldn't find board..." << endl;
  }

  int min, max;
  
  D.MaximumAbsoluteValue1(max);
  D.MinimumAbsoluteValue1(min);
  
  if(pc) {
    x = V(1,max);
    y = V(2,max);
  } else {
    x = V(1,min);
    y = V(2,min);
  }
  cout << V(1,max) << " " << V(2,max) << endl;
  cout << V(1,min) << " " << V(2,min) << endl;

}

//double calcAngle(double x, double y) {
//  angle atan2(x,y);

void sortCluster(double point_array[][2], int board_n, int board_h, int board_w, bool quiet) {
  double x, y, cx, cy;
  calcBoard(point_array, board_n, x, y, cx, cy, board_h <= board_w);
  double point_array2[board_n][2];
  double angle = -atan2(y,x);
  for(int i = 0; i < board_n; i++) {
    double tmpx = point_array[i][0] - cx;
    double tmpy = point_array[i][1] - cy;
    point_array2[i][0] = tmpx * cos(angle) - tmpy * sin(angle); 
    point_array2[i][1] = tmpx * sin(angle) + tmpy * cos(angle);
  }
  /////////////////sorting the points on the basis of y coordinate//////
  int swapped1 = 0;
  do {
    swapped1 = 0;
    
    for (int a = 1; a <= board_n - 1; a++) {
      if (point_array2[a][1] < point_array2[a - 1][1]) {
        //rotated points
        double tempx = point_array2[a][0];
        double tempy = point_array2[a][1];
        point_array2[a][0] = point_array2[a - 1][0];
        point_array2[a][1] = point_array2[a - 1][1];
        point_array2[a - 1][0] = tempx;
        point_array2[a - 1][1] = tempy;
        
        //original points
        double tmpx = point_array[a][0];
        double tmpy = point_array[a][1];
        point_array[a][0] = point_array[a - 1][0];
        point_array[a][1] = point_array[a - 1][1];
        point_array[a - 1][0] = tmpx;
        point_array[a - 1][1] = tmpy;
        swapped1 = 1;
      }
    }
  } while (swapped1 == 1);
  
  if(!quiet) {
    cout << "sorted array:" << endl;
    for (int f = 0; f < board_n; f++) {
      cout << point_array2[f][0] << " " << point_array2[f][1] << endl;
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////sorting the array rows now
  for (int x = 0; x < board_h; x++) {
    double row_points[board_w][2];
    double row_points2[board_w][2];
    for (int y = 0; y < board_w; y++) {
      row_points[y][0] = point_array[x * board_w + y][0];
      row_points[y][1] = point_array[x * board_w + y][1];
      row_points2[y][0] = point_array2[x * board_w + y][0];
      row_points2[y][1] = point_array2[x * board_w + y][1];
      if(!quiet) cout << row_points[y][0] << " " << row_points[y][1] << " ";
    }
    if(!quiet) cout << endl;
    int swapped = 0;
    do {
      swapped = 0;
      for (int a = 1; a <= board_w - 1; a++) {
        if (row_points2[a][0] < row_points2[a - 1][0]) {
          double tempx = row_points[a][0];
          double tempy = row_points[a][1];
          row_points[a][0] = row_points[a - 1][0];
          row_points[a][1] = row_points[a - 1][1];
          row_points[a - 1][0] = tempx;
          row_points[a - 1][1] = tempy;
          double tmpx = row_points2[a][0];
          double tmpy = row_points2[a][1];
          row_points2[a][0] = row_points2[a - 1][0];
          row_points2[a][1] = row_points2[a - 1][1];
          row_points2[a - 1][0] = tmpx;
          row_points2[a - 1][1] = tmpy;
          swapped = 1;
        }
      }
    } while (swapped == 1);
    if(!quiet) cout << "sorted:" << endl;
    for (int z = 0; z < board_w; z++) {
      point_array2[x * board_w + z][0] = row_points2[z][0];
      point_array2[x * board_w + z][1] = row_points2[z][1];
      if(!quiet) cout << point_array[x * board_w + z][0] << " " << point_array[x * board_w + z][1] << " ";
      point_array[x * board_w + z][0] = row_points[z][0];
      point_array[x * board_w + z][1] = row_points[z][1];
    }
    if(!quiet) cout << endl;

  }
  
}

IplImage* detectBlobs(IplImage *org_image, int corner_exp, int board_h, int board_w, bool quiet, double point_array2[][2]) {

  IplImage *gray_image = cvCloneImage(org_image); 
  cvThreshold(gray_image, gray_image, 140, 255, CV_THRESH_BINARY);
  //cvShowImage("Original Image", org_image);
  IplImage *labelImg = cvCreateImage(cvGetSize(gray_image), IPL_DEPTH_LABEL, 1);
  
  // detect blobs
  CvBlobs blobs;
  cvLabel(gray_image, labelImg, blobs);
  double average_size = 0;
  int count = 0;
  for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
    if (it->second->area < BLOB_SIZE) {
      count++;
      average_size += it->second->area;
    }
  }
  if(!quiet) cout << "centroid:" << average_size << endl;
  
  // refine blobs
  average_size = average_size / count;
  double average_size_min = average_size * (1.0 - AVG_THRES);
  double average_size_max = average_size * (1.0 + AVG_THRES);
  int blob_count = 0;
  
  for (CvBlobs::const_iterator it2 = blobs.begin(); it2 != blobs.end(); ++it2) {
    if (it2->second->area >= average_size_min && 
        it2->second->area <= average_size_max && 
        blob_count < corner_exp) {
        
      if(!quiet) {
      }
      point_array2[blob_count][0] = it2->second->centroid.x;
      point_array2[blob_count][1] = it2->second->centroid.y;
      blob_count++;
    }
  }

  if(!quiet) cout << "Refined number of blobs=" << blob_count << endl;
  // sorting the points
  sortCluster(point_array2, corner_exp, board_h, board_w, true); 
  cvReleaseImage(&labelImg);
  cout << "return" << endl;
  cout << "returned" << endl;
  return gray_image;
}

void drawLines(double point_array2[][2], int corner_exp, IplImage *image, bool color) {
  for (int i = 0; i <= corner_exp - 2; i++) {
    CvPoint pt1;
    CvPoint pt2;
    CvScalar s; 
    if(color) {
      s = CV_RGB(255,0,0);
    } else {
      s.val[0] = 100;
    }
    double temp1 = point_array2[i][0] - floor(point_array2[i][0]);
    if (temp1 < .5) {
      pt1.x = floor(point_array2[i][0]);
    } else {
      pt1.x = floor(point_array2[i][0]) + 1;
    }
    double temp2 = point_array2[i][1] - floor(point_array2[i][1]);
    if (temp2 < .5) {
      pt1.y = floor(point_array2[i][1]);
    } else {
      pt1.y = floor(point_array2[i][1]) + 1;
    }
    double temp3 = point_array2[i + 1][0] - floor(
        point_array2[i + 1][0]);
    if (temp3 < .5) {
      pt2.x = floor(point_array2[i + 1][0]);
    } else {
      pt2.x = floor(point_array2[i + 1][0]) + 1;
    }
    double temp4 = point_array2[i + 1][1] - floor(
        point_array2[i + 1][1]);
    if (temp4 < .5) {
      pt2.y = floor(point_array2[i + 1][1]);
    } else {
      pt2.y = floor(point_array2[i + 1][1]) + 1;
    }
    cvLine(image, pt1, pt2, s, 1, 8);
  }
  cvShowImage("Final Result", image);

}

IplImage* resizeImage(IplImage *source, int scale) {
  int width, height;
  IplImage *image;
  switch(scale) {
    case 2:
      width = 1200;
      height = 900;
      break;
    case 3:
      width = 800;
      height = 600;
      break;
    case 4:
      width = 400;
      height = 300;
      break;
    case 5:
      width = 160;
      height = 120;
      break;
    case 1:
    default:
      return cvCloneImage(source);
  }
 
  image = cvCreateImage(cvSize(width,height),source->depth,source->nChannels);
  cvResize(source,image);
  return image;
}

IplImage* detectCorners(IplImage *orgimage, int corner_exp, int board_h, int board_w, bool quiet, double point_array2[][2], int scale) {
  
  IplImage *image = resizeImage(orgimage, scale);
  CvSize size = cvGetSize(image);
  CvPoint2D32f* corners = new CvPoint2D32f[corner_exp];
	CvSize board_sz = cvSize(board_w, board_h);
  IplImage *gray_image = cvCreateImage(size,8,1);

  if (image->nChannels == 3) {
    cvCvtColor(image, gray_image, CV_BGR2GRAY);
  } else {
    gray_image = image;
  }
  
  int found = cvFindChessboardCorners(image, board_sz, corners, &corner_exp, 
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
  
  cout << "found corners:" << corner_exp << endl;
  if (found != 0) {//if all corners found successfully
    //Get Subpixel accuracy on those corners
    if(size.width > 400) {
      cvFindCornerSubPix(gray_image, corners, corner_exp, cvSize(11, 11), cvSize(-1, -1), 
                         cvTermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    else {
      cvFindCornerSubPix(gray_image, corners, corner_exp, cvSize(2, 2), cvSize(-1, -1), 
                         cvTermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
  }
	
  for (int i = 0; i < corner_exp; i++) {
    point_array2[i][0] = corners[i].x;
    point_array2[i][1] = corners[i].y;
  }
  return gray_image;
   
}

void writeCalibParam(int images, int corner_exp, int board_w, CvMat*
image_points, CvSize size, string dir, string substring) {
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
	//ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
	CvMat* object_points2 = cvCreateMat(images * corner_exp, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(images * corner_exp, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(images, 1, CV_32SC1);
	CvMat* Rotation = cvCreateMat(images, 3, CV_32FC1);
	CvMat* Translation = cvCreateMat(images, 3, CV_32FC1);
	//TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
	int j;
  for (int i = 0; i < successes * corner_exp; ++i) {
    j = i % corner_exp;
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0);
		CV_MAT_ELEM( *image_points2, float,i,1) = CV_MAT_ELEM( *image_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 0) = (j / board_w) * 8; 
		CV_MAT_ELEM( *object_points2, float, i, 1) = (j % board_w) * 8;
		CV_MAT_ELEM( *object_points2, float, i, 2) = 0.0f;
	}
	for (int i = 0; i < images; ++i) { //These are all the same number
		CV_MAT_ELEM( *point_counts2, int, i, 0) = corner_exp;
	}
	
  // Initialize the intrinsic matrix with focal length = 1.0 
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
	//CALIBRATE THE CAMERA!
	cvCalibrateCamera2(object_points2, image_points2, point_counts2, size,
			intrinsic_matrix, distortion_coeffs, Rotation, Translation, 0 //CV_CALIB_FIX_ASPECT_RATIO
	);
	// SAVE AND PRINT THE INTRINSICS AND DISTORTIONS
	
  string file = dir + "Intrinsics" + substring + ".xml";
  cvSave(file.c_str(), intrinsic_matrix);
	file = dir + "Distortion" + substring + ".xml";
  cvSave(file.c_str(), distortion_coeffs);
	cout << "Camera Intrinsic Matrix is:" << endl;
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			cout << CV_MAT_ELEM( *intrinsic_matrix, float, row, col ) << "\t";
		}
		cout << endl;
	}
	cout << "Distortion Coefficients are:" << endl;
	for (int row = 0; row < 5; row++) {
		for (int col = 0; col < 1; col++) {
			cout << CV_MAT_ELEM( *distortion_coeffs, float, row, col ) << "\t";
		}
	}
	cout << endl;
	////////////////////////////////////
	CvMat *intrinsic = intrinsic_matrix;
	CvMat *distortion = distortion_coeffs;
	// Build the undistort map that we will use for all subsequent frames.
	//
	IplImage* mapx = cvCreateImage(size, IPL_DEPTH_32F, 1);
	IplImage* mapy = cvCreateImage(size, IPL_DEPTH_32F, 1);
	cvInitUndistortMap(intrinsic, distortion, mapx, mapy);
	file = dir + "mapx" + substring + ".xml";
  cvSave(file.c_str(), mapx);
	file = dir + "mapy" + substring + ".xml";
	cvSave(file.c_str(), mapy);
	
  // CLEANUP
  cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);
	cvReleaseMat(&Rotation);
	cvReleaseMat(&Translation);
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&distortion);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
}

void CalibFunc(int board_w, int board_h, int start, int end, bool optical, bool
chess, bool quiet, string dir, int scale) {
  cvNamedWindow("Original Image", 0);
  cvResizeWindow( "Final Result", 320, 240 ); 
  cvNamedWindow("Final Result", 0);
  cvResizeWindow( "Final Result", 320, 240 ); 

  int nr_img = end - start + 1;
	if (nr_img == 0) {
		cout << "ImageCount is zero!" << endl;
		return;
	}
	
  int corner_exp = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);
	CvSize size;
	//ALLOCATE STORAGE(depending upon the number of images in(in case if command line arguments are given )
	//not on the basis of number of images in which all corner extracted/while in the other case the number is the same )
	CvMat* image_points = cvCreateMat(nr_img * corner_exp, 2, CV_32FC1);
	//TODO CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	
  successes = 0;
	int step = 0;
	
  for (int count = start; count <= end; count++) {
		string t;
    string t1;

    if(optical) {
      t = dir + to_string(count, 3) + "/photo" + to_string(count, 3) + ".ppm";
		  t1 = dir + to_string(count, 3) + "/cimage" + to_string(count, 3) + ".ppm";
    } else {
      t = dir + to_string(count, 3) + "/image" + to_string(count, 3) + ".ppm";
		  t1 = dir + to_string(count, 3) + "/timage" + to_string(count, 3) + ".ppm";
    }
    cout << t << endl;
		//loading images and finding corners
		IplImage* image1 = cvLoadImage(t.c_str(), -1);
		if (!image1) {
			cout << "image cannot be loaded" << endl;
			return;
		}
    cvShowImage("Original Image", image1);
	
    /////////////////////////////////////////////////////////////
	
    double point_array2[corner_exp][2];
    IplImage *image;
    
    if(chess) {
      cout << "detect corners" << endl;
      image = detectCorners(image1, corner_exp, board_h, board_w, quiet, point_array2, scale);
    } else {
      cout << "detect blob" << endl;
      image = detectBlobs(image1, corner_exp, board_h, board_w, quiet, point_array2);
    }

		//drawing the lines on the image now
		drawLines(point_array2, corner_exp, image);

		cout << "\nDo you want to use the data from this image ('y' or 'n'). 'x' aborts the calibration? : ";
		int c = cvWaitKey(100);
		if (c == 27) {
			break;
		}
		char in;
		cin >> in;
		if (in == 'y') {
			cvSaveImage(t1.c_str(), image);
			size = cvGetSize(image);
			step = successes * corner_exp;
			//appending corner data to a generic data structure for all images
			for (int i = step, j = 0; j < corner_exp; ++i, ++j) {
				CV_MAT_ELEM(*image_points, float,i,0) = (float) point_array2[j][0];
				CV_MAT_ELEM(*image_points, float,i,1) = (float) point_array2[j][1];
      }
			successes++;
		} else if(in == 'x') {
      break;
    }
		cvReleaseImage(&image);
		cvReleaseImage(&image1);

	}
	cout << "Images for which all corners were found successfully="
			<< successes << endl;
	if (successes == 0) {
		cout << "No successful corners found from any image" << endl;
		return;
	}

  string substring = optical? "Optical" : "";
  writeCalibParam(successes, corner_exp, board_w, image_points, size, dir, substring); 

	cvReleaseMat(&image_points);

//END
}

void readPoints(string filename, CvPoint3D32f *corners, int size) {
  ifstream infile(filename.c_str(), ios::in);
  if (!infile) {
    cout << "3Ddata file cannot be loaded" << endl;
    return;
  }
  
  for(int l = 0; l < size; l++) {
      infile >> corners[l].y;
      infile >> corners[l].z;
      infile >> corners[l].x;
      corners[l].y = -corners[l].y;
  }
}

void sortElementByElement(CvMat * vectors, int nr_elems, int nr_vectors) {
  bool swapped;
  for (int i = 0; i < nr_elems; i++) {
		do {
			swapped = false;
			for (int j = 1; j <= successes - 1; j++) {
				if (CV_MAT_ELEM(*vectors,float,j,i) < CV_MAT_ELEM(*vectors,float,j-1,i)) {
					float temp = CV_MAT_ELEM(*vectors,float,j,i);
					CV_MAT_ELEM(*vectors,float,j,i) = CV_MAT_ELEM(*vectors,float,j-1,i);
					CV_MAT_ELEM(*vectors,float,j-1,i) = temp;
					swapped = true;
				}
			}
		} while (swapped);
	}
}

void calculateExtrinsics(CvMat * rotation_vectors_temp, CvMat * translation_vectors_temp, int successes, string dir, bool quiet, string substring) {
  
  cout << "Start calculateExtrinsics" << endl;
  CvMat* rotation_vectors = cvCreateMat(successes, 3, CV_32FC1);
	CvMat* translation_vectors = cvCreateMat(successes, 3, CV_32FC1);
	CvMat* rotation_vector_mean = cvCreateMat(1, 3, CV_32FC1);
	CvMat* translation_vector_mean = cvCreateMat(1, 3, CV_32FC1);
	CvMat* rotation_vector_median = cvCreateMat(1, 3, CV_32FC1);
	CvMat* translation_vector_median = cvCreateMat(1, 3, CV_32FC1);

	for (int t = 0; t < 3; t++) {
    CV_MAT_ELEM(*rotation_vector_mean,float,0,t) = 0;
		CV_MAT_ELEM(*translation_vector_mean,float,0,t) = 0;
		CV_MAT_ELEM(*rotation_vector_median,float,0,t) = 0;
		CV_MAT_ELEM(*translation_vector_median,float,0,t) = 0;
	}

	for (int h = 0; h < successes; h++) {
		for(int t = 0; t < 3; t++) {
      CV_MAT_ELEM(*rotation_vectors,float,h,t) =CV_MAT_ELEM(*rotation_vectors_temp,float,h,t);
		  CV_MAT_ELEM(*rotation_vector_mean,float,0,t) +=CV_MAT_ELEM(*rotation_vectors,float,h,t);

		  CV_MAT_ELEM(*translation_vectors,float,h,t) =CV_MAT_ELEM(*translation_vectors_temp,float,h,t);
		  CV_MAT_ELEM(*translation_vector_mean,float,0,t) +=CV_MAT_ELEM(*translation_vectors,float,h,t);
	  }
  }
	
  for(int t = 0; t < 3; t++) {
    CV_MAT_ELEM(*rotation_vector_mean,float,0,t) /= successes;
	  CV_MAT_ELEM(*translation_vector_mean,float,0,t) /= successes;
  }

	// finding the median of rotation and translation
	// sorting the rotation vectors element by element
	sortElementByElement(rotation_vectors, 3, successes);
	sortElementByElement(translation_vectors, 3, successes);
	
	if(!quiet) {
    cout << "number of successes : " << successes << endl;
	  cout << "rotation vectors are" << endl;
	  for (int i = 0; i < successes; i++) {
		  cout << CV_MAT_ELEM(*rotation_vectors,float,i,0) << " "
				<<CV_MAT_ELEM(*rotation_vectors,float,i,1) << " "
				<<CV_MAT_ELEM(*rotation_vectors,float,i,2) << endl;
	  }
	  cout << "translation vectors are" << endl;
	  for (int i = 0; i < successes; i++) {
		  cout << CV_MAT_ELEM(*translation_vectors,float,i,0) << " "
				<<CV_MAT_ELEM(*translation_vectors,float,i,1) << " "
				<<CV_MAT_ELEM(*translation_vectors,float,i,2) << endl;
	  }
  }
	// getting the median vectors
	int index = successes / 2;
  for(int t = 0; t < 3; t++) {
    CV_MAT_ELEM(*translation_vector_median,float,0,t) = CV_MAT_ELEM(*translation_vectors,float,index,t);
	  CV_MAT_ELEM(*rotation_vector_median,float,0,t) = CV_MAT_ELEM(*rotation_vectors,float,index,t);
	}
	
  cout << "mean rotation vector is :" << endl;
	for (int f = 0; f < 3; f++) {
		cout << CV_MAT_ELEM(*rotation_vector_mean,float,0,f) << " ";
	}
	cout << endl;
	cout << "mean translation vector is :" << endl;
	for (int g = 0; g < 3; g++) {
		cout << CV_MAT_ELEM(*translation_vector_mean,float,0,g) << " ";
	}
	cout << endl;

	cout << "median rotation vector is :" << endl;
	for (int ff = 0; ff < 3; ff++) {
		cout << CV_MAT_ELEM(*rotation_vector_median,float,0,ff) << " ";
	}
	cout << endl;
	cout << "median translation vector is :" << endl;
	for (int gg = 0; gg < 3; gg++) {
		cout << CV_MAT_ELEM(*translation_vector_median,float,0,gg) << " ";
	}
	cout << endl;

  // write extrinsic parameters
	string file = dir + "RotationMean" + substring + ".xml";
  cvSave(file.c_str(), rotation_vector_mean);
	file = dir + "TranslationMean" + substring + ".xml";
	cvSave(file.c_str(), translation_vector_mean);
	file = dir + "RotationMedian" + substring + ".xml";
	cvSave(file.c_str(), rotation_vector_median);
	file = dir + "TranslationMedian" + substring + ".xml";
	cvSave(file.c_str(), translation_vector_median);
	
  // cleanup
	cvReleaseMat(&rotation_vector_mean);
	cvReleaseMat(&translation_vector_mean);
	cvReleaseMat(&rotation_vector_median);
	cvReleaseMat(&translation_vector_median);
  cout << "End calculateExtrinsics" << endl;

}

void ExtrCalibFunc(int board_w, int board_h, int start, int end, bool optical, bool chess, bool quiet, string dir, int scale) {
	int nr_img = end - start + 1;
  if (nr_img == 0) {
		cout << "ImageCount is zero!" << endl;
		return;
	}
  cvNamedWindow("Original Image", 0);
  cvResizeWindow( "Final Result", 320, 240 ); 
  cvNamedWindow("Final Result", 0);
  cvResizeWindow( "Final Result", 320, 240 ); 
	
  int corner_exp = board_w * board_h;
  CvSize board_sz = cvSize(board_w, board_h);
  CvSize size;
  ///getting number of corners in which should be detected in x and y direction depending
	/////on how the boards are positioned in the images///////////////////
	CvPoint3D32f* corners = new CvPoint3D32f[corner_exp];
	//ALLOCATE STORAGE(depending upon the number of images in(in case if command line arguments are given )
	//not on the basis of number of images in which all corner extracted/while in the other case the number is the same )

  string substring = optical? "Optical" : "";
  string file = dir + "Intrinsics" + substring + ".xml";
	cout << file << endl;
  CvMat *intrinsic = (CvMat*) cvLoad(file.c_str());
	file = dir + "Distortion" + substring + ".xml";
	CvMat *distortion = (CvMat*) cvLoad(file.c_str());
	file = dir + "mapx" + substring + ".xml";
	IplImage *mapx = (IplImage*) cvLoad(file.c_str());
	file = dir + "mapy" + substring + ".xml";
	IplImage *mapy = (IplImage*) cvLoad(file.c_str());
	////////////for storing the rotations and translation vectors
	CvMat* rotation_vectors_temp = cvCreateMat(corner_exp, 3, CV_32FC1);
	CvMat* translation_vectors_temp = cvCreateMat(corner_exp, 3, CV_32FC1);
	int successes = 0;

  cout << "xml files read" << endl;
  //////////////////////////////////////////////////////
	for (int count = start; count <= end; count++) {
		string i;
    string p;
    
    cout << "Reading data " << to_string(count, 3) << endl;
    if(optical) {
      i = dir + to_string(count, 3) + "/photo" + to_string(count, 3) + ".ppm";
    } else {
      i = dir + to_string(count, 3) + "/image" + to_string(count, 3) + ".ppm";
    }
    p = dir + to_string(count, 3) + "/scan" + to_string(count + 1,3) + ".3d";

    // Load points from scan
    readPoints(p, corners, corner_exp);
		
    // Load image and detect corners
    IplImage* image1 = cvLoadImage(i.c_str(), -1);
		if (!image1) {
			cout << "image cannot be loaded" << endl;
			return;
		}
    cvShowImage("Original Image", image1);
    
    double point_array2[corner_exp][2];
    IplImage *image;
    
    if(chess) {
      cout << "detect corners" << endl;
      image = detectCorners(image1, corner_exp, board_h, board_w, quiet, point_array2, scale);
    } else {
      cout << "detect blob" << endl;
      image = detectBlobs(image1, corner_exp, board_h, board_w, quiet, point_array2);
    }
    cout << "corners detected" <<  endl;
		
    //drawing the lines on the image now
		drawLines(point_array2, corner_exp, image);
    CvMat* image_points = cvCreateMat(corner_exp, 2, CV_32FC1);
		CvMat* object_points = cvCreateMat(corner_exp, 3, CV_32FC1);
		CvMat* point_counts = cvCreateMat(1, 1, CV_32SC1);
		CvMat* Rotation = cvCreateMat(1, 3, CV_32FC1);
		CvMat* Translation = cvCreateMat(1, 3, CV_32FC1);

		cout << "\nDo you want to use the data from this image ('y' or 'n'). 'x' aborts the calibration? : ";
		int c = cvWaitKey(100);
		if (c == 27) {
			break;
		}
		char in;
		cin >> in;
    if (in == 'y') {
      size = cvGetSize(image);
      //appending corner data to a generic data structure for all images
      for (int j = 0; j < corner_exp; ++j) {
        //doExtrinsicCalibration();
        CV_MAT_ELEM(*image_points, float,j,0) = (float) point_array2[j][0];
        CV_MAT_ELEM(*image_points, float,j,1) = (float) point_array2[j][1];
        CV_MAT_ELEM(*object_points,float,j,0) = corners[j].x;
        CV_MAT_ELEM(*object_points,float,j,1) = corners[j].y;
        CV_MAT_ELEM(*object_points,float,j,2) = corners[j].z;
      }
      CV_MAT_ELEM(*point_counts, int,0,0) = corner_exp;
      
      cvFindExtrinsicCameraParams2(object_points, image_points, intrinsic, distortion, Rotation, Translation);
     
      // append data to vectors
      if(!quiet) cout << "Rotation is:" << endl;
      for (int row = 0; row < 1; row++) {
        for (int col = 0; col < 3; col++) {
          if(!quiet) cout << CV_MAT_ELEM( *Rotation, float, row, col ) << "  ";
          CV_MAT_ELEM( *rotation_vectors_temp, float, successes, col ) = CV_MAT_ELEM( *Rotation, float, row, col );
        }
        if(!quiet) cout << endl;
      }
      if(!quiet) cout << "Translation is:" << endl;
      for (int row = 0; row < 1; row++) {
        for (int col = 0; col < 3; col++) {
          if(!quiet) cout << CV_MAT_ELEM( *Translation, float, row, col ) << "  ";
          CV_MAT_ELEM( *translation_vectors_temp, float, successes, col ) = CV_MAT_ELEM( *Translation, float, row, col );
        }
        if(!quiet) cout << endl;
      }
      successes++;
      
    } else if(in == 'x') {
      break;
    }
    cvReleaseImage(&image);
    cvReleaseImage(&image1);
		cvReleaseMat(&image_points);
		cvReleaseMat(&object_points);
		cvReleaseMat(&point_counts);
		cvReleaseMat(&Rotation);
		cvReleaseMat(&Translation);

	}//for loop for imagecount
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
  cvReleaseMat(&intrinsic);
  cvReleaseMat(&distortion);
	
  // Now calculating mean and median rotation and trans
  calculateExtrinsics(rotation_vectors_temp, translation_vectors_temp, successes, dir, quiet, substring);
	cvReleaseMat(&translation_vectors_temp);
	cvReleaseMat(&rotation_vectors_temp);
}

void ProjectAndMap(int start, int end, bool optical, bool quiet, string dir,
reader_type type, int scale, double rot_angle, double minDist, double maxDist,
bool correction, int neighborhood) {

  int nr_img = end - start + 1;
  if (nr_img < 1) {
    cout << "ImageCount is zero!" << endl;
    return;
  }
  string substring = optical? "Optical" : "";
  string file = dir + "Intrinsics" + substring + ".xml";
  CvMat *intrinsic = (CvMat*) cvLoad(file.c_str());
  file = dir + "Distortion" + substring + ".xml";
  CvMat *distortion = (CvMat*) cvLoad(file.c_str());
  file = dir + "RotationMedian" + substring + ".xml";
  CvMat *Rotation = (CvMat*) cvLoad(file.c_str());
  file = dir + "TranslationMedian" + substring + ".xml";
  CvMat *Translation = (CvMat*) cvLoad(file.c_str());
  CvMat* undistort = cvCreateMat(5,1,CV_32FC1);
	for (int hh = 0; hh < 5; hh++) {
	  CV_MAT_ELEM(*undistort, float,hh,0) = 0;
	}

  double starttime = GetCurrentTimeInMilliSec();

  // filling the rotation matrix for anticlockwise rotation
  double rPosTheta[3] = {0.0, rad(rot_angle), 0.0};
  double rPos[3] = {0.0, 0.0, 0.0};
  double alignxf[16];
  EulerToMatrix4(rPos, rPosTheta, alignxf); 

  string outdir = dir + "/labscan-map"; 
#ifdef _MSC_VER
  int success = mkdir(outdir.c_str());
#else
  int success = mkdir(outdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
  if(success == 0) { 
    cout << "Writing scans to " << outdir << endl;
  } else if(errno == EEXIST) {
    cout << "Directory " << outdir << " exists already.  CONTINUE" << endl; 
  } else {
    cerr << "Creating directory " << outdir << " failed" << endl;
    exit(1);
  }
  for (int count = start; count <= end; count++) {
    CvMat* point_3Dcloud; 
    CvMat* point_2Dcloud; 
    CvMat* undistort_2Dcloud; 
    CvMat* point_3Dcloud_2;
    CvMat* point_2Dcloud_2;
    CvMat* undistort_2Dcloud_2; 

    // loading images
    int count0;
    if(rot_angle < 180 && rot_angle > 0) {
      count0 = count % 9 == 8 ? count - 8 : count + 1;
    } else {
      count0 = count % 9 == 0 ? count + 8 : count - 1;
    }
    string t, t0;
    if(optical) {
      t = dir + "/photo" + to_string(count, 3) + ".ppm";
      t0 = dir + "/photo" + to_string(count0, 3) + ".ppm";
    } else {
      t = dir + "/image" + to_string(count, 3) + ".ppm";
      t0 = dir + "/image" + to_string(count0, 3) + ".ppm";
    }

    IplImage* image = cvLoadImage(t.c_str(), -1);
    if (!image) {
      cout << "first image " << t << " cannot be loaded" << endl;
      return;
    }
    CvSize size = cvGetSize(image); 

    IplImage* image0 = cvLoadImage(t0.c_str(), -1);

    if (!image0) {
      cout << "second image " << t0 << " cannot be loaded" << endl;
      return;
    }

    image0 = resizeImage(image0, scale);
    image = resizeImage(image, scale);
    // reading the 3D points and projecting them back to 2d
    Scan::readScans(type, count, count, dir, maxDist, minDist, 0);
    Scan::allScans[0]->calcReducedPoints(-1, 0);
    point_3Dcloud = cvCreateMat(Scan::allScans[0]->get_points_red_size(), 3, CV_32FC1);
    point_2Dcloud = cvCreateMat(Scan::allScans[0]->get_points_red_size(), 2, CV_32FC1);
    undistort_2Dcloud = cvCreateMat(Scan::allScans[0]->get_points_red_size(), 2, CV_32FC1);
    for (int j = 0; j < Scan::allScans[0]->get_points_red_size(); j++) {
      CV_MAT_ELEM(*point_3Dcloud, float,j,0) = Scan::allScans[0]->get_points_red()[j][2];
      CV_MAT_ELEM(*point_3Dcloud, float,j,1) = -Scan::allScans[0]->get_points_red()[j][0];
      CV_MAT_ELEM(*point_3Dcloud, float,j,2) = Scan::allScans[0]->get_points_red()[j][1];
    }
    
    cout << "Number of points read: " << Scan::allScans[0]->get_points_red_size() << endl;
    cvProjectPoints2(point_3Dcloud, Rotation, Translation, intrinsic,
        distortion, point_2Dcloud, NULL, NULL, NULL, NULL, NULL, 0);
		
    cvProjectPoints2(point_3Dcloud, Rotation, Translation, intrinsic,
        undistort, undistort_2Dcloud, NULL, NULL, NULL, NULL, NULL, 0);

    if(fabs(rot_angle) > 1) {
      point_3Dcloud_2 = cvCreateMat(Scan::allScans[0]->get_points_red_size(), 3, CV_32FC1);
      point_2Dcloud_2 = cvCreateMat(Scan::allScans[0]->get_points_red_size(), 2, CV_32FC1);
      undistort_2Dcloud_2 = cvCreateMat(Scan::allScans[0]->get_points_red_size(), 2, CV_32FC1);
      for (int j = 0; j < Scan::allScans[0]->get_points_red_size(); j++) {
        Point p(Scan::allScans[0]->get_points_red()[j]);
        p.transform(alignxf);

        CV_MAT_ELEM(*point_3Dcloud_2, float,j,0) = p.z;
        CV_MAT_ELEM(*point_3Dcloud_2, float,j,1) = -p.x;
        CV_MAT_ELEM(*point_3Dcloud_2, float,j,2) = p.y; 
      }

      cvProjectPoints2(point_3Dcloud_2, Rotation, Translation, intrinsic,
        distortion, point_2Dcloud_2, NULL, NULL, NULL, NULL, NULL, 0);
		
      cvProjectPoints2(point_3Dcloud_2, Rotation, Translation, intrinsic,
        undistort, undistort_2Dcloud_2, NULL, NULL, NULL, NULL, NULL, 0);
    } 
    // write colored data  
    string outname = outdir + "/scan" + to_string(count, 3) + ".3d";
    fstream outfile;
    outfile.open(outname.c_str(), ios::out);

    //for couting how many points get mapped to first and second image file
    int point_map1 = 0;  // #points mapped to first image
    int point_map2 = 0;  //    "      "    "  second image
    
    for (int k = 0; k < Scan::allScans[0]->get_points_red_size(); k++) {
      float px = CV_MAT_ELEM(*undistort_2Dcloud,float,k,0);
      float py = CV_MAT_ELEM(*undistort_2Dcloud,float,k,1);
      if (px < image->width - .5 && px >= 0 && py >= 0 && py < image->height - .5) {
        px = CV_MAT_ELEM(*point_2Dcloud,float,k,0);
        py = CV_MAT_ELEM(*point_2Dcloud,float,k,1);
        if (px < image->width - .5 && px >= 0 && py >= 0 && py < image->height - .5) {
          point_map1++;
          int ppx = 0;
          int ppy = 0;
          if (px - int(px) < .5) {
            ppx = int(px);
          } else {
            ppx = int(px) + 1;
          }
          if (py - int(py) < .5) {
            ppy = int(py);
          } else {
            ppy = int(py) + 1;
          }

          CvScalar c;
          c = cvGet2D(image, ppy, ppx);
          if(correction) {
            vector<float> temp_vec;
            float p_id = 1; //for keeping check later on if this vector belongs to the particular pixel
            temp_vec.push_back(-(CV_MAT_ELEM(*point_3Dcloud,float,k,1)));
            temp_vec.push_back((CV_MAT_ELEM(*point_3Dcloud,float,k,2)));
            temp_vec.push_back((CV_MAT_ELEM(*point_3Dcloud,float,k,0)));
            temp_vec.push_back(c.val[2]);
            temp_vec.push_back(c.val[1]);
            temp_vec.push_back(c.val[0]);
            temp_vec.push_back(p_id);
            if(neighborhood > 1) {
              int neighbors = 3;
              int limit = neighbors / 2;

              int lower_y = ppy - limit > 0 ? ppy - limit : 0;
              int upper_y = ppy + limit < size.height ? ppy + limit : size.height - 1;
              int lower_x = ppx - limit > 0 ? ppx - limit : 0;
              int upper_x = ppx + limit < size.width ? ppx + limit : size.width - 1;

              for (int y = lower_y; y < upper_y; y++) {
                for (int x = lower_x; x < upper_x; x++) {
                  if(x == ppx && y == ppy) {
                    temp_vec[6] = 1;
                  } else {
                    temp_vec[6] = 0;
                  }
                  data1[y][x].push_back(temp_vec);
                }
              }

            } else {
              data1[ppy][ppx].push_back(temp_vec);
            }
            temp_vec.clear();
          } else {

            outfile << -(CV_MAT_ELEM(*point_3Dcloud,float,k,1))<<" ";
            outfile << CV_MAT_ELEM(*point_3Dcloud,float,k,2)<<" ";
            outfile << CV_MAT_ELEM(*point_3Dcloud,float,k,0)<<" ";
            outfile << c.val[2] <<" "<< c.val[1]<<" "<<c.val[0]<<endl;

          }
        }
      } else if(fabs(rot_angle > 1)){
        px = CV_MAT_ELEM(*undistort_2Dcloud_2,float,k,0);
        py = CV_MAT_ELEM(*undistort_2Dcloud_2,float,k,1);
        if (px < image0->width - .5 && px >= 0 && py >= 0 && py < image0->height - .5) {
          px = CV_MAT_ELEM(*point_2Dcloud_2,float,k,0);
          py = CV_MAT_ELEM(*point_2Dcloud_2,float,k,1);
          if (px < image0->width - .5 && px >= 0 && py >= 0 && py < image0->height - .5) {
            point_map2++;
            int ppx = 0;
            int ppy = 0;

            if (px - int(px) < .5) {
              ppx = int(px);
            } else {
              ppx = int(px) + 1;
            }
            if (py - int(py) < .5) {
              ppy = int(py);
            } else {
              ppy = int(py) + 1;
            }
            CvScalar c;

            c = cvGet2D(image0, ppy, ppx);
            CvMat* temp_vec_in2 = cvCreateMat(3, 1, CV_32FC1);
            CvMat* temp_vec_out2 = cvCreateMat(3, 1, CV_32FC1);
            CV_MAT_ELEM(*temp_vec_in2, float,0,0) = -(CV_MAT_ELEM(*point_3Dcloud_2,float,k,1));
            CV_MAT_ELEM(*temp_vec_in2, float,1,0) = CV_MAT_ELEM(*point_3Dcloud_2,float,k,2);
            CV_MAT_ELEM(*temp_vec_in2, float,2,0) = CV_MAT_ELEM(*point_3Dcloud_2,float,k,0);
            if(correction) {
              vector<float> temp_vec;
              float p2_id = 1;
              temp_vec.push_back((CV_MAT_ELEM(*temp_vec_out2,float,0,0)));
              temp_vec.push_back((CV_MAT_ELEM(*temp_vec_out2,float,1,0)));
              temp_vec.push_back((CV_MAT_ELEM(*temp_vec_out2,float,2,0)));
              temp_vec.push_back(c.val[2]);
              temp_vec.push_back(c.val[1]);
              temp_vec.push_back(c.val[0]);
              temp_vec.push_back(p2_id);
              if(neighborhood > 1) {
                int neighbors = 3;
                int limit = neighbors / 2;

                int lower_y = ppy - limit > 0 ? ppy - limit : 0;
                int upper_y = ppy + limit < size.height ? ppy + limit : size.height - 1;
                int lower_x = ppx - limit > 0 ? ppx - limit : 0;
                int upper_x = ppx + limit < size.width ? ppx + limit : size.width - 1;

                for (int y = lower_y; y < upper_y; y++) {
                  for (int x = lower_x; x < upper_x; x++) {
                    if(x == ppx && y == ppy) {
                      temp_vec[6] = 1;
                    } else {
                      temp_vec[6] = 0;
                    }
                    data2[y][x].push_back(temp_vec);
                  }
                }

              } else {
                data2[ppy][ppx].push_back(temp_vec);
              }
              temp_vec.clear();
              cvReleaseMat(&temp_vec_in2);
              cvReleaseMat(&temp_vec_out2);
            } else {
              outfile << -(CV_MAT_ELEM(*point_3Dcloud_2,float,k,1))<<" ";
              outfile << CV_MAT_ELEM(*point_3Dcloud_2,float,k,2)<<" ";
              outfile << CV_MAT_ELEM(*point_3Dcloud_2,float,k,0)<<" ";
              outfile << c.val[2] <<" "<< c.val[1]<<" "<<c.val[0]<<endl;
            }
          }
        }
      }

    }
    CorrectErrorAndWrite(data1, outfile, size);
    if(fabs(rot_angle) > 1) {
      CorrectErrorAndWrite(data2, outfile, size);
    }

    outfile.flush();
    outfile.close();

    delete Scan::allScans[0];
    Scan::allScans.clear();

    double endtime = GetCurrentTimeInMilliSec();
    double time = endtime - starttime;
    time = time/1000.0;
    cout<<"runtime for scan " << count << " in seconds is: " << time << endl;

    cvReleaseImage(&image);
    cvReleaseImage(&image0);
    cvReleaseMat(&point_2Dcloud);
    cvReleaseMat(&point_3Dcloud);
    if(fabs(rot_angle) > 1) {
      cvReleaseMat(&point_2Dcloud_2);
      cvReleaseMat(&point_3Dcloud_2);
    }
    for (int f = 0; f < size.height; f++) {
      for (int g = 0; g < size.width; g++) {
        data1[f][g].clear();
        data2[f][g].clear();
      }
    }

  }

  cvReleaseMat(&intrinsic);
  cvReleaseMat(&distortion);
  cvReleaseMat(&Rotation);
  cvReleaseMat(&Translation);

}

void sortDistances(float ** points, int size) {
  int swapped1 = 0;
  do {
    swapped1 = 0;
    for(int a = 1; a <= size - 1; a++) {
      if(Norm3(points[a]) < Norm3(points[a - 1])) {
        float * tmp = points[a-1];
        points[a-1] = points[a];
        points[a] = tmp;
        swapped1 = 1;
      }
    }
  } while (swapped1 == 1);
}

bool clusterSearch(float ** points, int size, double thresh1, double thres2, fstream &outfile) {
  int position = 0;
  int cluster_count = 0;

  double max_cluster = 0;
  int max_position = 0;
  vector<double*> clusters;
  while (position < size) {
    double sum = 0.0;
    int j = position + 1;
    while(j < size && (Norm3(points[j]) < (Norm3(points[j-1]) + thresh1))) { 
      j++; 
      cluster_count++; 
      sum+=Norm3(points[j-1]);
    }
    double * tmp = new double[4];
    tmp[0] = position;
    tmp[1] = j - 1;
    tmp[2] = sum / (j - position);
    tmp[3] = (j - position) / (clusters.size() + 1);
    if(tmp[3] > max_cluster) {
      max_position = clusters.size();
      max_cluster = tmp[3];
    }
    clusters.push_back(tmp);
    position = j;
  }
  /* 
     cout << max_position << " " << max_cluster << endl;
     for(int i = 0; i < clusters.size(); i++) {
     cout << clusters[i][0] << " " << clusters[i][1] << endl;
     }
   */

  //if(clusters[max_position][3] > thres2) {
    /*
    for (int p = 0; p < clusters[max_position][0]; p++)//points before the cluster
    {
      points[p][3] = 0;
      points[p][4] = 0;
      points[p][5] = 255;
    }
    */
    for(int p = clusters[max_position][0]; p < clusters[max_position][1]; p++) {    
      if(points[p][6] == 1) {
			  outfile << points[p][0] << " " << points[p][1] << " " << points[p][2] << " ";
				outfile << points[p][3] << " " << points[p][4] << " " << points[p][5] << endl;
      }
    }
    /*
    for(int p = clusters[max_position][1] + 1; p < size; p++) {
      points[p][3] = 0;
      points[p][4] = 255;
      points[p][5] = 255;
    }
    */
  //} 
    
  for(unsigned int i = 0; i < clusters.size(); i++) {
    delete[] clusters[i]; 
  }
  return false;
}

void CorrectErrorAndWrite(Float2D &data, fstream &outfile, CvSize size) {
  int temp = 0;
  int max_cluster = 0;
  int min_cluster = 100000000;
  double thresh1 = 4;
  double thresh2 = 5;
  
  // getting points mapping to one pixel
  for (int i = 0; i < size.height; i++) {
    for (int j = 0; j < size.width; j++) {
      int size = data[i][j].size();
      if (size > 0) {
        max_cluster = size > max_cluster ? size : max_cluster;
        min_cluster = size < min_cluster ? size : min_cluster;

        float ** points = new float*[size];
        for (int k = 0; k < size; k++) {
          points[k] = new float[7];
          for(int l = 0; l < 7; l++) {
            points[k][l] = data[i][j][k][l];
          }
        }
        //////////sorting the points now in ascending order wrt distance
        sortDistances(points, size); 
        ///////////////once we have the sorted points looking for clusters
        clusterSearch(points, size, thresh1, thresh2, outfile);
        //////////////write the points to the file
        /*
        for(int k = 0; k < size; k++) {
          if(points[k][6] == 1) {
						outfile << points[k][0] << " " << points[k][1] << " " << points[k][2] << " ";
						outfile << points[k][3] << " " << points[k][4] << " " << points[k][5] << endl;
          }
        }
        */
        for (int k = 0; k < size; k++) {
          delete[] points[k];
        }
        delete[] points;
      }

    }
  }

}

void usage(char* prog) {
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif
  cout << endl
	  << bold << "USAGE " << normal << endl
	  << "   " << prog << " [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply})" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
	  << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
	  << "         end at scan NR" << endl
    << endl  
	  << bold << "  -x" << normal << " NR, " << bold << "--width=" << normal << "NR" << endl
	  << "         NR of lamps/corners in x direction" << endl
    << endl  
	  << bold << "  -y" << normal << " NR, " << bold << "--height=" << normal << "NR" << endl
	  << "         NR of lamps/corners in y direction" << endl
    << endl  
	  << bold << "  -o --=optical" << normal << endl
	  << "         use optical camera instead of thermal camera" << endl
    << endl
	  << bold << "  -c --=chess" << normal << endl
	  << "         use chessboard pattern for calibration instead of lightbulb pattern" << endl
    << endl
	  << bold << "  -I --=intrinsic" << normal << endl
	  << "         perform intrinsic calibration" << endl
    << endl
	  << bold << "  -E --=extrinsic" << normal << endl
	  << "         perform extrinsic calibration" << endl
    << endl
	  << bold << "  -P --=mapping" << normal << endl
	  << "         perform mapping of image data to point cloud" << endl
    << endl
	  << bold << "  -q --=quiet" << normal << endl
	  << "         " << endl
    << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -s 2 -e 10 dat" << endl << endl;
  exit(1);

}

/**
  * Parses command line arguments needed for plane detection. For details about
  * the argument see usage().
  */

int parseArgs(int argc, char **argv, string &dir, int &start, int &end, double
&maxDist, double &minDist, reader_type &type, bool &optical, bool &chess, int
&width, int &height, bool &intrinsic, bool &extrinsic, bool &mapping, bool
&correction, int &scale, int &neighborhood, double &angle, bool &quiet ) {
  // from unistd.h:
  int  c;
  extern char *optarg;
  extern int optind;
  
  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "correction",      no_argument,         0,  'C' },  
    { "scale",           required_argument,   0,  'S' },  
    { "neighborhood",    required_argument,   0,  'n' },  
    { "angle",           required_argument,   0,  'a' },  
    { "format",          required_argument,   0,  'f' },  
    { "max",             required_argument,   0,  'm' },
    { "min",             required_argument,   0,  'M' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "width",           required_argument,   0,  'x' },
    { "height",          required_argument,   0,  'y' },
    { "quiet",           no_argument,         0,  'q' },
    { "optical",         no_argument,         0,  'o' },
    { "intrinsic",       no_argument,         0,  'I' },
    { "extrinsic",       no_argument,         0,  'E' },
    { "mapping",         no_argument,         0,  'P' },
    { "chess",           no_argument,         0,  'c' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };
  
  cout << endl;
  while ((c = getopt_long(argc, argv, "f:s:e:x:y:m:M:qoIEPcCS:n:a:", longopts, NULL)) != -1) { 
  switch (c)
	{
	 case 's':
	   start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   end = atoi(optarg);
	   if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
	   if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
	   break;
	 case 'f': 
     if (!Scan::toType(optarg, type))
       abort ();
     break;
	 case 'q':
     quiet = true;
     break;
   case 'm':
	   maxDist = atoi(optarg);
	   break;
	 case 'M':
	   minDist = atoi(optarg);
	   break;
   case 'o':
    optical = true;
    break;
   case 'I':
    intrinsic = true;
    break;
   case 'E':
    extrinsic = true;
    break;
   case 'P':
    mapping = true;
    break;
   case 'c':
    chess = true;
    break;
   case 'x':
    width = atoi(optarg);
    break;
   case 'y':
    height = atoi(optarg);
    break;
   case 'S':
    scale = atoi(optarg);
    break;
   case 'a':
    angle = atof(optarg);
    break;
   case 'n':
    neighborhood = atoi(optarg);
    break;
   case 'C':
    correction = true;
    break;
   case '?':
	   usage(argv[0]);
	   return 1;
   default:
	   cout << "Abort" << endl;
     abort ();
   }
  }
  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    usage(argv[0]);
  }
  dir = argv[optind];
#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif

  if(!(intrinsic || extrinsic || mapping)) {
    cerr << "\n*** Please choose at least one method (intrinsic calibration, "
    << "extrinsic calibration, mapping of image data to point data! ***\n" <<
    endl;
    usage(argv[0]);
  }
  return 0;

}

int main(int argc, char** argv) {
  string dir;
  int start = 0;
  int end = -1;
  int width = 5;
  int height = 6;
  double maxDist = -1;
  double minDist = -1;
  reader_type type = UOS;
  bool optical = false;
  bool chess = false;
  bool intrinsic = false;
  bool extrinsic = false;
  bool mapping = false;
  bool quiet = false;
  int scale = 1;
  double rot_angle = 40;
  bool correction = false;
  int neighborhood = 1;

  parseArgs(argc, argv, dir, start, end, maxDist, minDist, type, optical, chess,
  width, height, intrinsic, extrinsic, mapping, correction, scale, neighborhood,
  rot_angle, quiet);
  int count = end - start;

  // DEFAULT PARAMETERS (to be added to parseArgs)
  if(mapping) {
    if(!quiet) cout << "Starting projecting and mapping image data to point cloud..." << endl;
    ProjectAndMap(start, end, optical, quiet, dir, type, scale, rot_angle, minDist, maxDist, correction, neighborhood);
    if(!quiet) cout << "\nDONE" << endl;
    return 0;
  }

  if(intrinsic) {
    if(!quiet) {
      cout << "Starting intrinsic calibration using ";
      if(chess) cout << "chessboard pattern..." << endl;
      else cout << "lightbulb pattern..." << endl;
    }
		CalibFunc(width, height, start, end, optical, chess, quiet, dir, scale);
    if(!quiet) cout << "\nDONE" << endl;
  }

  if(extrinsic) {
    if(!quiet) {
      cout << "Starting extrinsic calibration using ";
      if(chess) cout << "chessboard pattern..." << endl;
      else cout << "lightbulb pattern..." << endl;
    }
		ExtrCalibFunc(width, height, start, end, optical, chess, quiet, dir, scale);
    if(!quiet) cout << "\nDONE" << endl;
  }

	return 0;
}

