/**
 * 
 * Copyright (C) Jacobs University Bremen
 *
 * @author Vaibhav Kumar Mehta
 * @file normals.cc
 */

#include <iostream>
#include <string>
#include <fstream>
#include <errno.h>

#include <boost/program_options.hpp>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include "slam6d/fbr/panorama.h"
#include <scanserver/clientInterface.h>

#include <ANN/ANN.h>
#include "newmat/newmat.h"
#include "newmat/newmatap.h"

using namespace NEWMAT;

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

namespace po = boost::program_options;
using namespace std;

enum normal_method {AKNN, ADAPTIVE_AKNN, PANORAMA, PANORAMA_FAST};

/*
 * validates normal calculation method specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
		    normal_method*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  if(strcasecmp(arg.c_str(), "AKNN") == 0) v = AKNN;
  else if(strcasecmp(arg.c_str(), "ADAPTIVE_AKNN") == 0) v = ADAPTIVE_AKNN;
  else if(strcasecmp(arg.c_str(), "PANORAMA") == 0) v = PANORAMA;
  else if(strcasecmp(arg.c_str(), "PANORAMA_FAST") == 0) v = PANORAMA_FAST;
  else throw std::runtime_error(std::string("normal calculation method ") + arg + std::string(" is unknown"));
}

/// validate IO types
void validate(boost::any& v, const std::vector<std::string>& values,
		    IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

/// Parse commandline options
void parse_options(int argc, char **argv, int &start, int &end, bool &scanserver, int &max_dist, int &min_dist, string &dir,
                   IOType &iotype, int &k1, int &k2, normal_method &ntype,int &width,int &height)
{
  /// ----------------------------------
  /// set up program commandline options
  /// ----------------------------------
  po::options_description cmd_options("Usage: calculateNormals <options> where options are (default values in brackets)");
  cmd_options.add_options()
    ("help,?", "Display this help message")
    ("start,s", po::value<int>(&start)->default_value(0), "Start at scan number <arg>")
    ("end,e", po::value<int>(&end)->default_value(-1), "Stop at scan number <arg>")
    ("scanserver,S", po::value<bool>(&scanserver)->default_value(false), "Use the scanserver as an input method")
    ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
	"using shared library <arg> for input. (chose format from [uos|uosr|uos_map|"
	"uos_rgb|uos_frames|uos_map_frames|old|rts|rts_map|ifp|"
	"riegl_txt|riegl_rgb|riegl_bin|zahn|ply])")
    ("max,M", po::value<int>(&max_dist)->default_value(-1),"neglegt all data points with a distance larger than <arg> 'units")
    ("min,m", po::value<int>(&min_dist)->default_value(-1),"neglegt all data points with a distance smaller than <arg> 'units")
    ("normal,g", po::value<normal_method>(&ntype)->default_value(AKNN), "normal calculation method "
	"(AKNN, ADAPTIVE_AKNN, PANORAMA, PANORAMA_FAST)")
    ("K1,k", po::value<int>(&k1)->default_value(20), "<arg> value of K value used in the nearest neighbor search of ANN or" 								     "kmin for k-adaptation")
    ("K2,K", po::value<int>(&k2)->default_value(20), "<arg> value of Kmax for k-adaptation")
    ("width,w", po::value<int>(&width)->default_value(1280),"width of panorama image")
    ("height,h", po::value<int>(&height)->default_value(960),"height of panorama image")
    ;

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<string>(&dir), "input dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl << endl;
    cout << "SAMPLE COMMAND FOR CALCULATING NORMALS" << endl;
    cout << " bin/normals -s 0 -e 0 -f UOS -g AKNN -k 20 dat/" <<endl;
    cout << endl << endl;
    cout << "SAMPLE COMMAND FOR VIEWING CALCULATING NORMALS IN RGB SPACE" << endl;
    cout << " bin/show -c -f UOS_RGB dat/normals/" << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";

}
///////////////////////////////////////////////////////
/////////////NORMALS USING AKNN METHOD ////////////////
///////////////////////////////////////////////////////
void calculateNormalsAKNN(vector<Point> &normals,vector<Point> &points, int k, const double _rPos[3] )
{
  cout<<"Total number of points: "<<points.size()<<endl;
  int nr_neighbors = k;

  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];
  
  ANNpointArray pa = annAllocPts(points.size(), 3);
  for (size_t i=0; i<points.size(); ++i)
    {
	 pa[i][0] = points[i].x;
	 pa[i][1] = points[i].y;
	 pa[i][2] = points[i].z;
    }		
  ANNkd_tree t(pa, points.size(), 3);	
  ANNidxArray nidx = new ANNidx[nr_neighbors];
  ANNdistArray d = new ANNdist[nr_neighbors];		

  for (size_t i=0; i<points.size(); ++i)
    {
	 ANNpoint p = pa[i];
	 //ANN search for k nearest neighbors 
	 //indexes of the neighbors along with the query point 
	 //stored in the array n
	 t.annkSearch(p, nr_neighbors, nidx, d, 0.0);
	 Point mean(0.0,0.0,0.0);
	 Matrix X(nr_neighbors,3);
	 SymmetricMatrix A(3);
	 Matrix U(3,3);
	 DiagonalMatrix D(3);
	 //calculate mean for all the points	    	
	 for (int j=0; j<nr_neighbors; ++j)
	   {
		mean.x += points[nidx[j]].x;	
		mean.y += points[nidx[j]].y;
		mean.z += points[nidx[j]].z;
	   }
	 mean.x /= nr_neighbors;
	 mean.y /= nr_neighbors;
	 mean.z /= nr_neighbors;
	 //calculate covariance = A for all the points
	 for (int i = 0; i < nr_neighbors; ++i) {
	   X(i+1, 1) = points[nidx[i]].x - mean.x;
	   X(i+1, 2) = points[nidx[i]].y - mean.y;
	   X(i+1, 3) = points[nidx[i]].z - mean.z;
	 }
    
	 A << 1.0/nr_neighbors * X.t() * X;

	 EigenValues(A, D, U);

	 //normal = eigenvector corresponding to lowest 
	 //eigen value that is the 1st column of matrix U
	 ColumnVector n(3);
	 n(1) = U(1,1);
	 n(2) = U(2,1);
	 n(3) = U(3,1);
	 ColumnVector point_vector(3);
	 point_vector(1) = p[0] - rPos(1);
	 point_vector(2) = p[1] - rPos(2);
	 point_vector(3) = p[2] - rPos(3);
	 point_vector = point_vector / point_vector.NormFrobenius();
	 Real angle = (n.t() * point_vector).AsScalar();
	 if (angle < 0) {
	   n *= -1.0;
	 }
	 n = n / n.NormFrobenius();
	 normals.push_back(Point(n(1), n(2), n(3)));  	
    }

  delete[] nidx;
  delete[] d;
  annDeallocPts(pa);
}
////////////////////////////////////////////////////////////////
/////////////NORMALS USING ADAPTIVE AKNN METHOD ////////////////
////////////////////////////////////////////////////////////////
void calculateNormalsAdaptiveAKNN(vector<Point> &normals,vector<Point> &points,
						    int kmin, int kmax, const double _rPos[3])
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  cout<<"Total number of points: "<<points.size()<<endl;
  int nr_neighbors;	
  ANNpointArray pa = annAllocPts(points.size(), 3);
  for (size_t i=0; i<points.size(); ++i)
    {
	 pa[i][0] = points[i].x;
	 pa[i][1] = points[i].y;
	 pa[i][2] = points[i].z;
    }		
  ANNkd_tree t(pa, points.size(), 3);	

  Point mean(0.0,0.0,0.0);
  double temp_n[3],norm_n = 0.0;
  double e1,e2,e3;	
		
  for (size_t i=0; i<points.size(); ++i)
    {
	 Matrix U(3,3);
	 ANNpoint p = pa[i];
	 for(int kidx = kmin; kidx < kmax; kidx++)
	   {
		nr_neighbors=kidx+1;
		ANNidxArray nidx = new ANNidx[nr_neighbors];
		ANNdistArray d = new ANNdist[nr_neighbors];
		//ANN search for k nearest neighbors 
		//indexes of the neighbors along with the query point 
		//stored in the array n
		t.annkSearch(p, nr_neighbors, nidx, d, 0.0);
		mean.x=0,mean.y=0,mean.z=0;
		//calculate mean for all the points	    	
		for (int j=0; j<nr_neighbors; ++j)
		  {
		    mean.x += points[nidx[j]].x;	
		    mean.y += points[nidx[j]].y;
		    mean.z += points[nidx[j]].z;
		  }
		mean.x /= nr_neighbors;
		mean.y /= nr_neighbors;
		mean.z /= nr_neighbors;

		Matrix X(nr_neighbors,3);
		SymmetricMatrix A(3);
		DiagonalMatrix D(3);

		//calculate covariance = A for all the points
		for (int j = 0; j < nr_neighbors; ++j) {
		  X(j+1, 1) = points[nidx[j]].x - mean.x;
		  X(j+1, 2) = points[nidx[j]].y - mean.y;
		  X(j+1, 3) = points[nidx[j]].z - mean.z;
		}
			
		A << 1.0/nr_neighbors * X.t() * X;
			
		EigenValues(A, D, U);

		e1 = D(1);
		e2 = D(2);
		e3 = D(3);
		
		delete[] nidx;
		delete[] d;
		
		//We take the particular k if the second maximum eigen value 
		//is at least 25 percent of the maximum eigen value
		if ((e1 > 0.25 * e2) && (fabs(1.0 - (double)e2/(double)e3) < 0.25)) 
		  break;
	   }
	 
	 //normal = eigenvector corresponding to lowest 
	 //eigen value that is the 1rd column of matrix U
	 ColumnVector n(3);
	 n(1) = U(1,1);
	 n(2) = U(2,1);
	 n(3) = U(3,1);
	 ColumnVector point_vector(3);
	 point_vector(1) = p[0] - rPos(1);
	 point_vector(2) = p[1] - rPos(2);
	 point_vector(3) = p[2] - rPos(3);
	 point_vector = point_vector / point_vector.NormFrobenius();
	 Real angle = (n.t() * point_vector).AsScalar();
	 if (angle < 0) {
	   n *= -1.0;
	 }
	 n = n / n.NormFrobenius();
	 normals.push_back(Point(n(1), n(2), n(3)));  
    }
  annDeallocPts(pa);
}

///////////////////////////////////////////////////////
/////////////NORMALS USING IMAGE NEIGHBORS ////////////
///////////////////////////////////////////////////////
void calculateNormalsPANORAMA(vector<Point> &normals,
						vector<Point> &points,
						vector< vector< vector< cv::Vec3f > > > extendedMap,
						const double _rPos[3])
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];
  
  cout<<"Total number of points: "<<points.size()<<endl;
  points.clear();
  int nr_neighbors = 0;
  cout << "height of Image: "<<extendedMap.size()<<endl;
  cout << "width of Image: "<<extendedMap[0].size()<<endl;
	
  // as the nearest neighbors and then the same PCA method as done in AKNN 			
  //temporary dynamic array for all the neighbors of a given point	
  vector<cv::Vec3f> neighbors;
  for (size_t i=0; i< extendedMap.size(); i++)
    {
	 for (size_t j=0; j<extendedMap[i].size(); j++)
	   {
		if (extendedMap[i][j].size() == 0) continue;
		neighbors.clear();
		Point mean(0.0,0.0,0.0);
		double temp_n[3],norm_n = 0.0;
	      
		// Offset for neighbor computation
		int offset[2][5] = {{-1,0,1,0,0},{0,-1,0,1,0}};
 
		// Traversing all the cells in the extended map
		for (int n = 0; n < 5; ++n) {
		  int x = i + offset[0][n];
		  int y = j + offset[1][n];
		
		  // Copy the neighboring buckets into the vector
		  if (x >= 0 && x < (int)extendedMap.size() && 
			 y >= 0 && y < (int)extendedMap[x].size()) {
		    for (unsigned int k = 0; k < extendedMap[x][y].size(); k++) {
			 neighbors.push_back(extendedMap[x][y][k]);
		    }
		  }
		}
	      
		nr_neighbors = neighbors.size();	 
		cv::Vec3f p = extendedMap[i][j][0];
			
		//if no neighbor point is found in the 4-neighboring pixels then normal is set to zero
		if (nr_neighbors < 3) 
		  {
		    points.push_back(Point(p[0], p[1], p[2]));
		    normals.push_back(Point(0.0,0.0,0.0));
		    continue;			
		  }			

		//calculate mean for all the points						
		Matrix X(nr_neighbors,3);
		SymmetricMatrix A(3);
		Matrix U(3,3);
		DiagonalMatrix D(3);
	      
		//calculate mean for all the points	    	
		for(int k = 0; k < nr_neighbors; k++) 
		  {	 
		    cv::Vec3f pp = neighbors[k];
		    mean.x += pp[0];
		    mean.y += pp[1];
		    mean.z += pp[2];
		  }
	      
		mean.x /= nr_neighbors;
		mean.y /= nr_neighbors;
		mean.z /= nr_neighbors;
		//calculate covariance = A for all the points
		for (int i = 0; i < nr_neighbors; ++i) {
		  cv::Vec3f pp = neighbors[i];
		  X(i+1, 1) = pp[0] - mean.x;
		  X(i+1, 2) = pp[1] - mean.y;
		  X(i+1, 3) = pp[2] - mean.z;
		}
	      
		A << 1.0/nr_neighbors * X.t() * X;
	
		EigenValues(A, D, U);
		//normal = eigenvector corresponding to lowest 
		//eigen value that is the 1st column of matrix U
		ColumnVector n(3);
		n(1) = U(1,1);
		n(2) = U(2,1);
		n(3) = U(3,1);
		ColumnVector point_vector(3);
		point_vector(1) = p[0] - rPos(1);
		point_vector(2) = p[1] - rPos(2);
		point_vector(3) = p[2] - rPos(3);
		point_vector = point_vector / point_vector.NormFrobenius();
		Real angle = (n.t() * point_vector).AsScalar();
		if (angle < 0) {
		  n *= -1.0;
		}
		n = n / n.NormFrobenius();

		for (unsigned int k = 0; k < extendedMap[i][j].size(); k++) {
		  cv::Vec3f p = extendedMap[i][j][k];
		  points.push_back(Point(p[0], p[1], p[2]));
		  normals.push_back(Point(n(1), n(2), n(3)));
		}
	   }	
    }  	
}
//////////////////////////////////////////////////////////////////////////////////////////////
///////////FAST NORMALS USING PANORAMA EQUIRECTANGULAR RANGE IMAGE //////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/*
  void calculateNormalsFAST(vector<Point> &normals,vector<Point> &points,cv::Mat &img,vector<vector<vector<cv::Vec3f>>> extendedMap)
  {
  cout<<"Total number of points: "<<points.size()<<endl;
  points.clear();
  int nr_points = 0;
  //int nr_neighbors = 0,nr_neighbors_center = 0;	
  cout << "height of Image: "<<extendedMap.size()<<endl;
  cout << "width of Image: "<<extendedMap[0].size()<<endl;
  for (size_t i=0; i< extendedMap.size(); ++i)
  {
  for (size_t j=0; j<extendedMap[0].size(); j++)
  {
  double theta,phi,rho;
  double x,y,z;
  double dRdTheta,dRdPhi;
  double n[3],m;
  nr_points = extendedMap[i][j].size();
  if (nr_points == 0 ) continue;
			
  for (int k = 0; k< nr_points; k++)
  {
  cv::Vec3f p = extendedMap[i][j][k];
  x = p[0];
  y = p[1];
  z = p[2]; 
  rho = sqrt(x*x + y*y + z*z);
  theta = atan(y/x);
  phi = atan(z/x);
	
  //Sobel Filter for the derivative
  dRdTheta = dRdPhi = 0.0;

  if (i == 0 || i == extendedMap.size()-1 || j == 0 || j == extendedMap[0].size()-1)
  { 
  points.push_back(Point(x, y, z));
  normals.push_back(Point(0.0,0.0,0.0));   					
  continue; 
  }			
  dRdPhi += 10*img.at<uchar>(i-1,j);
  dRdPhi += 3 *img.at<uchar>(i-1,j-1);
  dRdPhi += 3 *img.at<uchar>(i-1,j+1);
  dRdPhi -= 10*img.at<uchar>(i+1,j);
  dRdPhi -= 3 *img.at<uchar>(i+1,j-1);
  dRdPhi -= 3 *img.at<uchar>(i+1,j+1);
    
  dRdTheta += 10*img.at<uchar>(i,j-1);
  dRdTheta += 3 *img.at<uchar>(i-1,j-1);
  dRdTheta += 3 *img.at<uchar>(i+1,j-1);
  dRdTheta -= 10*img.at<uchar>(i,j+1);
  dRdTheta -= 3 *img.at<uchar>(i-1,j+1);
  dRdTheta -= 3 *img.at<uchar>(i+1,j+1);				
				
  n[0] = cos(theta) * sin(phi) - sin(theta) * dRdTheta / rho / sin(phi) + 
  cos(theta) * cos(phi) * dRdPhi / rho;

  n[1] = sin(theta) * sin(phi) + cos(theta) * dRdTheta / rho / sin(phi) + 
  sin(theta) * cos(phi) * dRdPhi / rho;
  
  n[2] =  cos(phi) - sin(phi) * dRdPhi / rho;

  //n[2] = -n[2];
				
  m = sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
  n[0] /= m; n[1] /= m; n[2] /= m;
				
  points.push_back(Point(x, y, z));
  normals.push_back(Point(n[0],n[1],n[2]));
  }				
  }
  }	
  }
*/

/*
 * retrieve a cv::Mat with x,y,z,r from a scan object
 * functionality borrowed from scan_cv::convertScanToMat but this function
 * does not allow a scanserver to be used, prints to stdout and can only
 * handle a single scan
 */
cv::Mat scan2mat(Scan *source)
{
  DataXYZ xyz = source->get("xyz");

  DataReflectance xyz_reflectance = source->get("reflectance");

  unsigned int nPoints = xyz.size();
  cv::Mat scan(nPoints,1,CV_32FC(4));
  scan = cv::Scalar::all(0);

  cv::MatIterator_<cv::Vec4f> it;

  it = scan.begin<cv::Vec4f>();
  for(unsigned int i = 0; i < nPoints; i++){
    float x, y, z, reflectance;
    x = xyz[i][0];
    y = xyz[i][1];
    z = xyz[i][2];
    if(xyz_reflectance.size() != 0)
	 {
	   reflectance = xyz_reflectance[i];
		
	   //normalize the reflectance
	   reflectance += 32;
	   reflectance /= 64;
	   reflectance -= 0.2;
	   reflectance /= 0.3;
	   if (reflectance < 0) reflectance = 0;
	   if (reflectance > 1) reflectance = 1;
	 }	
	
    (*it)[0] = x;
    (*it)[1] = y;
    (*it)[2] = z;
    if(xyz_reflectance.size() != 0)
	 (*it)[3] = reflectance;
    else
	 (*it)[3] = 0;	
	
    ++it;
  }
  return scan;
}
/*
 * convert a matrix of float values (range image) to a matrix of unsigned
 * eight bit characters using different techniques
 */
cv::Mat float2uchar(cv::Mat &source, bool logarithm, float cutoff)
{
  cv::Mat result(source.size(), CV_8U, cv::Scalar::all(0));
  float max = 0;
  // find maximum value
  if (cutoff == 0.0) {
    // without cutoff, just iterate through all values to find the largest
    for (cv::MatIterator_<float> it = source.begin<float>();
	    it != source.end<float>(); ++it) {
	 float val = *it;
	 if (val > max) {
	   max = val;
	 }
    }
  } else {
    // when a cutoff is specified, sort all the points by value and then
    // specify the max so that <cutoff> values are larger than it
    vector<float> sorted(source.cols*source.rows);
    int i = 0;
    for (cv::MatIterator_<float> it = source.begin<float>();
	    it != source.end<float>(); ++it, ++i) {
	 sorted[i] = *it;
    }
    std::sort(sorted.begin(), sorted.end());
    max = sorted[(int)(source.cols*source.rows*(1.0-cutoff))];
    cout << "A cutoff of " << cutoff << " resulted in a max value of " << max << endl;
  }

  cv::MatIterator_<float> src = source.begin<float>();
  cv::MatIterator_<uchar> dst = result.begin<uchar>();
  cv::MatIterator_<float> end = source.end<float>();
  if (logarithm) {
    // stretch values from 0 to max logarithmically over 0 to 255
    // using the logarithm allows to represent smaller values with more
    // precision and larger values with less
    max = log(max+1);
    for (; src != end; ++src, ++dst) {
	 float val = (log(*src+1)*255.0)/max;
	 if (val > 255)
	   *dst = 255;
	 else
	   *dst = (uchar)val;
    }
  } else {
    // stretch values from 0 to max linearly over 0 to 255
    for (; src != end; ++src, ++dst) {
	 float val = (*src*255.0)/max;
	 if (val > 255)
	   *dst = 255;
	 else
	   *dst = (uchar)val;
    }
  }
  return result;
}
/// Write a pose file with the specofied name
void writePoseFiles(string dir, const double* rPos, const double* rPosTheta,int scanNumber)
{
  string poseFileName = dir + "/scan" + to_string(scanNumber, 3) + ".pose";
  ofstream posout(poseFileName.c_str());

  posout << rPos[0] << " "
	    << rPos[1] << " "
	    << rPos[2] << endl
	    << deg(rPosTheta[0]) << " "
	    << deg(rPosTheta[1]) << " "
	    << deg(rPosTheta[2]) << endl;
  posout.clear();
  posout.close();
}

/// write scan files for all segments
void writeScanFiles(string dir, vector<Point> &points, vector<Point> &normals, int scanNumber)
{
  string ofilename = dir + "/scan" + to_string(scanNumber, 3) + ".3d";
  ofstream normptsout(ofilename.c_str());
	
  for (size_t i=0; i<points.size(); ++i)
    {
	 int r,g,b;
	 r = (int)(normals[i].x * (127.5) + 127.5);
	 g = (int)(normals[i].y * (127.5) + 127.5);
	 b = (int)(fabs(normals[i].z) * (255.0)); 
	 normptsout <<points[i].x<<" "<<points[i].y<<" "<<points[i].z<<" "<<r<<" "<<g<<" "<<b<<" "<<endl;
    }
  normptsout.clear();
  normptsout.close();
}

/// =============================================
/// Main
/// =============================================
int main(int argc, char** argv)
{
  int start, end;
  bool scanserver;
  int max_dist, min_dist;
  string dir;
  IOType iotype;
  int k1, k2;
  normal_method ntype;
  int width, height;

  parse_options(argc, argv, start, end, scanserver, max_dist, min_dist,
			 dir, iotype, k1, k2, ntype, width, height);

  /// ----------------------------------
  /// Prepare and read scans
  /// ----------------------------------
  if (scanserver) {
    try {
	 ClientInterface::create();
    } catch(std::runtime_error& e) {
	 cerr << "ClientInterface could not be created: " << e.what() << endl;
	 cerr << "Start the scanserver first." << endl;
	 exit(-1);
    }
  }

  /// Make directory for saving the scan segments
  string normdir = dir + "normals";

#ifdef _MSC_VER
  int success = mkdir(normdir.c_str());
#else
  int success = mkdir(normdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
  if(success == 0) {
    cout << "Writing segments to " << normdir << endl;
  } else if(errno == EEXIST) {
    cout << "WARN: Directory " << normdir << " exists already. Contents will be overwriten" << endl;
  } else {
    cerr << "Creating directory " << normdir << " failed" << endl;
    exit(1);
  }

  /// Read the scans
  Scan::openDirectory(scanserver, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }
	
  cv::Mat img;

  /// --------------------------------------------
  /// Initialize and perform segmentation
  /// --------------------------------------------
  std::vector<Scan*>::iterator it = Scan::allScans.begin();
  int scanNumber = 0;

  for( ; it != Scan::allScans.end(); ++it) {
    Scan* scan = *it;

    // apply optional filtering
    scan->setRangeFilter(max_dist, min_dist);
	
    const double* rPos = scan->get_rPos();
    const double* rPosTheta = scan->get_rPosTheta();

    /// read scan into points
    DataXYZ xyz(scan->get("xyz"));
    vector<Point> points;
    points.reserve(xyz.size());
    vector<Point> normals;
    normals.reserve(xyz.size());
	
    for(unsigned int j = 0; j < xyz.size(); j++) {
	 points.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
    }
	
    if(ntype == AKNN)
	 calculateNormalsAKNN(normals,points, k1, rPos);
    else if(ntype == ADAPTIVE_AKNN)	
	 calculateNormalsAdaptiveAKNN(normals,points, k1, k2, rPos);
    else 
	 {
	   // create panorama
	   fbr::panorama fPanorama(width, height, fbr::EQUIRECTANGULAR, 1, 0, fbr::EXTENDED);
	   fPanorama.createPanorama(scan2mat(scan));
	
	   // the range image has to be converted from float to uchar
	   img = fPanorama.getRangeImage();
	   img = float2uchar(img, 0, 0.0);

	   if(ntype == PANORAMA)
		calculateNormalsPANORAMA(normals,points,fPanorama.getExtendedMap(), rPos);
	   else if(ntype == PANORAMA_FAST)
		cout << "PANORAMA_FAST is not working yet" << endl;
	   //   calculateNormalsFAST(normals,points,img,fPanorama.getExtendedMap());
	 }

    // pose file (repeated for the number of segments
    writePoseFiles(normdir, rPos, rPosTheta, scanNumber);
    // scan files for all segments
    writeScanFiles(normdir, points,normals,scanNumber);

    scanNumber++;
  }

  // shutdown everything
  if (scanserver)
    ClientInterface::destroy();
  else
    Scan::closeDirectory();

  cout << "Normal program end" << endl;

  return 0;
}

