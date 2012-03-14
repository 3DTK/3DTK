#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/scan_io_rxp.h"
#include "riegl/scanlib.hpp"

using namespace cv;
using namespace std;

typedef Vec<float, 7> Vec7f;

//Vertical angle of view of scanner
#define MAX_ANGLE 60.0
#define MIN_ANGLE -40.0
//projection methods
#define EQUIRECTANGULAR 1
#define CYLINDRICAL 2
#define MERCATOR 3
#define RECTILINEAR 4
#define PANNINI 5
#define STEREOGRAPHIC 6
#define ZAXIS 24
//feature detecting methods
#define SURF 7
#define SIFT 8
#define ORB 9
#define FAST 10
#define STAR 11
//input data scan or panorama image
#define SCAN 12
#define IMAGE 13
//matching method
#define BRUTEFORCE 14
#define FLANN 15
#define KNN 16
#define RADIUS 17
#define RATIO 18
//scan format
#define TXT 19
#define RXP 20
#define ZF 25
//registration format
#define ALL 21
#define RANSAC 22
#define DISABLE 23
//RANSAC iteration
#define RANSACITR 20000
//Inlier influence
#define iInfluence 0.5

struct information{
  string local_time;
  string input_1, input_2, output_1, output_2, fOutput, pOutput, mOutput, dir, rOutput;
  int pMethod, iWidth, iHeight, nImage, fMethod, dMethod, iMethod, mMethod, minD, minI, sFormat, fScan, sScan, knn, rMethod, num_pts_1, num_pts_2, num_features_1, num_features_2, num_matches, num_filter_matches;
  double dPannini, rStereographic, minE, radius;
  double iTime_1, iTime_2, pTime_1, pTime_2, fTime_1, fTime_2, dTime_1, dTime_2, mTime, rTime; 
  double zmax;
  double zmin;
} info;
 
/**
 * usage : explains how to use the program CMD
 */
void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s -i[SCAN|IMAGE] input1 input2 \t IMAGE only for feature detection and description and matching - SCAN for all the processes\n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-F sFormat\t\t input scan file format [TXT|RXP]\n");
  printf("\t\t-w iWidth\t\t image width\n");
  printf("\t\t-h iHeight\t\t image height\n");
  printf("\t\t-p pMethod\t\t projection method [EQUIRECTANGULAR|CYLINDRICAL|MERCATOR|RECTILINEAR|PANNINI|STEREOGRAPHIC|ZAXIS]\n");
  printf("\t\t-n nImage\t\t number of images used for some projections\n");
  printf("\t\t-P dPannini\t\t special pannini projection parameter\n");
  printf("\t\t-S rStereographic\t special stereographic parameter\n");
  printf("\t\t-f fMethod\t\t feature detection method [SURF|SIFT|ORB|FAST|STAR]\n");
  printf("\t\t-d dMethod\t\t feature description method [SURF|SIFT|ORB]\n");
  printf("\t\t-m mMethod\t\t feature matching method [BRUTEFORCE|FLANN|KNN|RADIUS|RATIO]\n");
  printf("\t\t-D minD \t\t threshold for min distance in registration process\n");
  printf("\t\t-E minE \t\t threshold for min error in registration process\n");
  printf("\t\t-I minI \t\t threshold for min number of inliers in registration process\n");
  printf("\t\t-k knn  \t\t the number of K best matches for KNN method\n");
  printf("\t\t-r radius \t\t the max distance for feature matching in radius method\n");
  printf("\t\t-R registration \t registration method [ALL|ransac]\n");
  printf("\n");
  exit(1);
}

/**
 * parseArgs : reade the comand line options if available otherwise use default values
 * @param input_1 first scan
 * @param input_2 second scan
 * @param pMethod projection Method
 * @param iWidth image width
 * @param iHeight image height
 * @param nImgae number of images used for some projections
 * @param dPannini special pannini projection parameter
 * @param rStereographic special stereographic projection parameter
 * @param fMethod feature detection method
 * @param dMethid feature description method
 * @param mMethid feature matching method
 * @param output_1 name of first output
 * @param output_2 name of second output
 * @param fOutput featurte information for output
 * @param pOutput panorama information for output
 * @param mOutput feature matcher information for output
 * @param minD threshold for min distance in registratiuon process
 * @param minE threshold for min Error in registratiuon process
 * @param minI threshold for min Iniler in registratiuon process
 * @param dir directory of the input files for the output 
 * @param sFormat input scan file format 
 * @param fScan starting scan (first scan)
 * @param sScan ending scan (second scan)
 * @param knn number of k best matches for KNN method
 * @param radius max distance for feature matching in radius method
 * @param rMethod registration method
 * @param rOutput registration information for output
 */
int parseArgs(int argc, char **argv, information& info)
	      // string& input_1, string& input_2, int &pMethod, int& iWidth, int& iHeight, int &nImage, double &dPannini, double &rStereographic, int &fMethod, int &dMethod, int& iMethod, int& mMethod, string& output_1, string& output_2, string& fOutput, string& pOutput, string& mOutput, int& minD, double& minE, int& minI, string& dir, int& sFormat, int& fScan, int& sScan, int& knn, double& radius, int& rMethod, string& rOutput)
{
  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  char time[50];
  timeinfo = localtime (&rawtime);
  sprintf(time, "%d-%d-%d-%d:%d:%d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  info.local_time = time;
  cout<<info.local_time<<endl;
  
  int c;
  //default values
  info.iWidth = 3600;
  info.iHeight = 1000;
  info.nImage = 3;
  info.dPannini = 1;
  info.rStereographic = 2;
  info.minD = 50;
  info.minE = 50;
  info.minI = 5;
  info.fScan = 0;
  info.sScan = 0;
  info.knn = 3;
  info.radius = 100;
  
  string projection, feature, descriptor, input, matcher, format, registration;
  opterr = 0;
  //reade the command line and get the options
  while ((c = getopt (argc, argv, "i:F:s:e:w:h:p:n:P:S:f:d:m:D:E:I:k:r:R:")) != -1)
    switch (c)
      {
      case 'i':
	input = optarg;
	break;
      case 'w':
	info.iWidth = atoi(optarg);
	break;
      case 'h':
        info.iHeight = atoi(optarg);
	break;
      case 'p':
	projection = optarg;
	break;
      case 'n':
	info.nImage = atoi(optarg);
	break;
      case 'P':
	info.dPannini = atoi(optarg);
	break;
      case 'S':
	info.rStereographic = atoi(optarg);
	break;
      case 'f':
	feature = optarg;
	break;
      case 'd':
	descriptor = optarg;
	break;
      case 'm':
	matcher = optarg;
	break;
      case 'D':
	info.minD = atoi(optarg);
	break;
      case 'E':
	info.minE = atoi(optarg);
	break;
      case 'I':
	info.minI = atoi(optarg);
	break;
      case 'F':
	 format = optarg;
	break;
      case 's':
	info.fScan = atoi(optarg);
	break;
      case 'e':
	info.sScan = atoi(optarg);
	break;
      case 'k':
	info.knn = atoi(optarg);
	break;
      case 'r':
	info.radius = atoi(optarg);
	break;
      case 'R':
	registration = optarg;
	break;
      case '?':
	cout<<"Unknown option character "<<optopt<<endl;
	return 1;
      case ':':
      default:
	usage(argc, argv);
      }
  
  //check for input method
  if(input.empty() || ((input.compare("IMAGE") != 0) && (input.compare("SCAN") != 0)))
    {
      usage(argc, argv);
    }
  if(input.compare("IMAGE") == 0)
    {
      info.iMethod = IMAGE;
      cout<<"iMethod: Image"<<endl;
    }
  if(input.compare("SCAN") == 0)
    {
      info.iMethod = SCAN;
      cout<<"iMethod: Scan"<<endl;
    }

  //check for projection
  if (projection.empty()|| ( (projection.compare("EQUIRECTANGULAR") != 0) && (projection.compare("CYLINDRICAL") != 0) && (projection.compare("MERCATOR") != 0) && (projection.compare("RECTILINEAR") != 0) && (projection.compare("PANNINI") != 0) && (projection.compare("STEREOGRAPHIC") != 0) && (projection.compare("ZAXIS") != 0)))   
    {
      projection = "EQUIRECTANGULAR";
    }
  if(projection.compare("EQUIRECTANGULAR") == 0)
    {
      info.pMethod = EQUIRECTANGULAR;
      if(info.iMethod == SCAN)
	cout<<"pMethod: Equirectangular"<<endl;
      info.pOutput = "_p-EQUIRECTANGULAR";
    }
  if(projection.compare("CYLINDRICAL") == 0)
    {
      info.pMethod = CYLINDRICAL;
      if(info.iMethod == SCAN)
	cout<<"pMethod: Cylindrical"<<endl;
      info.pOutput = "_p-CYLINDRICAL";	 
    }
  if(projection.compare("MERCATOR") == 0)
    {
      info.pMethod = MERCATOR;
      if(info.iMethod == SCAN)
	cout<<"pMethod: Mercator"<<endl;
      info.pOutput = "_p-MERCATOR";
    }
  if(projection.compare("RECTILINEAR") == 0)
    {
      info.pMethod = RECTILINEAR;
      if(info.iMethod == SCAN)
	{
	  cout<<"pMethod: Rectilinear"<<endl;
	  cout<<"nImage: "<<info.nImage<<endl;
	}
      info.pOutput = "_p-RECTILINEAR";
    }
  if(projection.compare("PANNINI") == 0)
    {
      info.pMethod = PANNINI;
      if(info.iMethod == SCAN)
	{
	  cout<<"pMethod: Pannini"<<endl;
	  cout<<"nImage: "<<info.nImage<<endl;
	  cout<<"dPannini: "<<info.dPannini<<endl;
	}
      info.pOutput = "_p-PANNINI";
    }
  if(projection.compare("STEREOGRAPHIC") == 0)
    {
      info.pMethod = STEREOGRAPHIC;
      if(info.iMethod == SCAN)
	{
	  cout<<"pMethod: Stereographic"<<endl;
	  cout<<"nImage: "<<info.nImage<<endl;
	  cout<<"rStereographic: "<<info.rStereographic<<endl;
	}
      info.pOutput = "_p-STEREOGRAPHIC";
    }
  if(projection.compare("ZAXIS") == 0)
    {
      info.pMethod = ZAXIS;
      if(info.iMethod == SCAN)
	cout<<"pMethod: ZAXIS"<<endl;
      info.pOutput = "_p-ZAXIS";
    }

  //check for input scan format
  char size [50];
  if(info.iMethod == SCAN)
    {
      if(format.empty() || ((format.compare("TXT") != 0) && (format.compare("RXP") != 0) && (format.compare("ZF") != 0)))
	{
	  usage(argc, argv);
	}
      if(format.compare("TXT") == 0)
	{
	  info.sFormat = TXT;
	  cout<<"sFormat: TXT"<<endl;
	}
      if(format.compare("RXP") == 0)
	{
	  info.sFormat = RXP;
	  cout<<"sFormat: RXP"<<endl;
	}
      if(format.compare("ZF") == 0)
	{
	  info.sFormat = ZF;
	  cout<<"sFormat: ZF"<<endl;
	}
      cout<<"iWidth: "<<info.iWidth<<endl;
      cout<<"iHeight: "<<info.iHeight<<endl;
      cout<<"minD: "<<info.minD<<endl;
      cout<<"minI: "<<info.minI<<endl;
      cout<<"minE: "<<info.minE<<endl;
    }

  info.pOutput += "_"; 
  sprintf(size, "%dx%d", info.iWidth, info.iHeight);
  info.pOutput += size;
  
  //check for feature detector
  if (feature.empty() || ((feature.compare("SURF") != 0) && (feature.compare("SIFT") != 0) && (feature.compare("ORB") != 0) && (feature.compare("FAST") != 0) && (feature.compare("STAR") != 0)))
    {
      feature = "SIFT";
    }
  if(feature.compare("SURF") == 0)
    {
      info.fMethod = SURF;
      cout<<"fMethod: SURF"<<endl;
      info.fOutput = "_f-SURF";
    }
  if(feature.compare("SIFT") == 0)
    {
      info.fMethod = SIFT;
      cout<<"fMethod: SIFT"<<endl;
      info.fOutput = "_f-SIFT";
    }
  if(feature.compare("ORB") == 0)
    {
      info.fMethod = ORB;
      cout<<"fMethod: ORB"<<endl;
      info.fOutput = "_f-ORB";
    }
  if(feature.compare("FAST") == 0)
    {
      info.fMethod = FAST;
      cout<<"fMethod: FAST"<<endl;
      info.fOutput = "_f-FAST";
    }
  if(feature.compare("STAR") == 0)
    {
      info.fMethod = STAR;
      cout<<"fMethod: STAR"<<endl;
      info.fOutput = "_f-STAR";
    }

  //check for feature descriptor
  if (descriptor.empty() || ((descriptor.compare("SURF") != 0) && (descriptor.compare("SIFT") != 0) && (descriptor.compare("ORB") != 0)))
    {
      descriptor = "SIFT";
    }
  if(descriptor.compare("SURF") == 0)
    {
      info.dMethod = SURF;
      cout<<"dMethod: SURF"<<endl;
      info.fOutput += "_d-SURF";
    }
  if(descriptor.compare("SIFT") == 0)
    {
      info.dMethod = SIFT;
      cout<<"dMethod: SIFT"<<endl;
      info.fOutput += "_d-SIFT";
    }
  if(descriptor.compare("ORB") == 0)
    {
      info.dMethod = ORB;
      cout<<"dMethod: ORB"<<endl;
      info.fOutput += "_d-ORB";
    }
  if(descriptor.compare("ORB") == 0 && feature.compare("SIFT") == 0)
    {
      cout<<"ERROR: SIFT feature doesn't work with ORB descriptors."<<endl;
      usage(argc, argv);
    }
  
  //check for feature matcher
  if (matcher.empty() || ((matcher.compare("BRUTEFORCE") != 0) && (matcher.compare("FLANN") != 0) && (matcher.compare("KNN") != 0) && (matcher.compare("RADIUS") != 0) && (matcher.compare("RATIO") != 0)))
    {
      matcher = "BRUTEFORCE";
    }
  if(matcher.compare("BRUTEFORCE") == 0)
    {
      info.mMethod = BRUTEFORCE;
      cout<<"mMethod: BRUTEFORCE"<<endl;
      info.mOutput += "_m-BRUTEFORCE";
    }
  if(matcher.compare("FLANN") == 0)
    {
      info.mMethod = FLANN;
      cout<<"mMethod: FLANN"<<endl;
      info.mOutput += "_m-FLANN";
    }
  if(matcher.compare("KNN") == 0)
    {
      info.mMethod = KNN;
      cout<<"mMethod: KNN with K: "<<info.knn<<endl;
      info.mOutput += "_m-KNN";
    }
  if(matcher.compare("RATIO") == 0)
    {
      info.mMethod = RATIO;
      cout<<"mMethod: RATIO"<<endl;
      info.mOutput += "_m-RATIO";
    }
  if(matcher.compare("RADIUS") == 0)
    {
      info.mMethod = RADIUS;
      cout<<"mMethod: RADIUS with maxDistance: "<<info.radius<<endl;
      info.mOutput += "_m-RADIUS";
    }
  if(matcher.compare("FLANN") == 0 && descriptor.compare("ORB") == 0)
    {
      cout<<"ERROR: ORB descriptor only works with BRUTEFORCE matcher."<<endl;
      usage(argc, argv);
    }
  //check for registration method
  if(registration.empty() || ((registration.compare("ALL") != 0) && (registration.compare("RANSAC") != 0) && (registration.compare("DISABLE") != 0)))
    {
      registration = "DISABLE";
    }
  if(registration.compare("DISABLE") == 0)
    {
      info.rMethod = DISABLE;
      cout<<"rMethod: DISABLE"<<endl;
    }
  if(registration.compare("ALL") == 0)
    {
      info.rMethod = ALL;
      cout<<"rMethod: ALL"<<endl;
      info.rOutput +="_r-ALL";
    }
  if(registration.compare("RANSAC") == 0)
    {
      info.rMethod = RANSAC;
      cout<<"rMethod: RANSAC"<<endl;
      info.rOutput +="_r-RANSAC";
    }
  //check for input format
  if(info.sFormat == TXT || info.iMethod == IMAGE || info.sFormat == ZF)
    {
      if (optind > argc - 2)
	{
	  cout<<"Too few input files. At least two are required."<<endl;
	  usage(argc, argv);
	}
      info.input_1 = argv[optind];
      info.input_2 = argv[optind+1];
      string key_1 ("/");
      string key_2 (".");
      size_t pos_1;
      size_t pos_2;
      pos_1 = info.input_1.rfind(key_1);
      pos_2 = info.input_1.rfind(key_2);
      info.output_1 = info.input_1.substr((pos_1)+1, (pos_2)-(pos_1)-1);
      info.dir = info.input_1.substr(0, pos_1);
      cout<<"DIR: "<<info.dir<<endl;
      cout<<"input_1:"<<info.input_1<<endl;
      cout<<"output_1:"<<info.output_1<<endl;
      pos_1 = info.input_2.rfind(key_1);
      pos_2 = info.input_2.rfind(key_2);
      info.output_2 = info.input_2.substr((pos_1)+1, (pos_2)-(pos_1)-1);
      cout<<"input_2:"<<info.input_2<<endl;
      cout<<"output_2:"<<info.output_2<<endl;
    }
  if(info.sFormat == RXP)
    {
      if (optind > argc - 1)
	{
	  cout<<"Too few input files. At least one is required."<<endl;
	  usage(argc, argv);
	}     
      if(optind == argc - 1)
	{
	  info.input_1 = argv[optind];
	  string key_1 ("/");
	  size_t pos_1;
	  pos_1 = info.input_1.rfind(key_1);
	  info.output_1 = "scan" + to_string(info.fScan,3);
	  info.dir = info.input_1.substr(0, pos_1);
	  cout<<"DIR: "<<info.dir<<endl;
	  cout<<"input_1:"<<info.input_1<<endl;
	  cout<<"output_1:"<<info.output_1<<endl;
	  info.input_2 = argv[optind];
	  info.output_2 = "scan" + to_string(info.sScan,3);
	  cout<<"output_2:"<<info.output_2<<endl;
	}
      else
	{
	  info.input_1 = argv[optind];
	  info.input_2 = argv[optind+1];
	  string key_1 ("/");
	  string key_2 (".");
	  size_t pos_1;
	  size_t pos_2;
	  pos_1 = info.input_1.rfind(key_1);
	  pos_2 = info.input_1.rfind(key_2);
	  info.output_1 = info.input_1.substr((pos_1)+1, (pos_2)-(pos_1)-1);
	  info.dir = info.input_1.substr(0, pos_1);
	  cout<<"DIR: "<<info.dir<<endl;
	  cout<<"input_1:"<<info.input_1<<endl;
	  cout<<"output_1:"<<info.output_1<<endl;
	  pos_1 = info.input_2.rfind(key_1);
	  pos_2 = info.input_2.rfind(key_2);
	  info.output_2 = info.input_2.substr((pos_1)+1, (pos_2)-(pos_1)-1);
	  cout<<"input_2:"<<info.input_2<<endl;
	  cout<<"output_2:"<<info.output_2<<endl;
	  info.sScan = 0;
	  info.fScan = 0;
	}
    }
  return 1;
}

/**
 * scan_reader : reads the scan files and add them to a mat data type also normalize the reflectance
 * @param scan_name name of the input scan file
 * @param S Mat data type which contains the 3D Scan
 * @param sFormat input scan file format
 * @param sNumber input scan number
 * @param num_pts Number of points
 * @param zmax max value in z direction
 * @param zmin min value in z direction
 * @return 0 if there is an error otherwise it return 1
 */
int scan_reader(string scan_name, Mat& S, int sFormat, int sNumber, int& num_pts, double& zmax, double& zmin)
{
  if(sFormat == TXT)
    {
      ifstream scan_in;
      
      scan_in.open(scan_name.c_str(), ios::in);
      if (scan_in.good())
	{
	  scan_in >> num_pts;
	  cout<<"loading "<<scan_name<<" with "<<num_pts<<" points."<<endl;
	}
      else
	return 0;
      
      S.create(num_pts,1,CV_32FC(7));
      MatIterator_<Vec7f> it, end;
      it = S.begin<Vec7f>();
      end = S.end<Vec7f>();
      while(scan_in.good() && (it != end))
	{
	  float x,y,z,range,theta,phi,reflectance;
	  scan_in >> z >> x >> y >> range >> theta >> phi >> reflectance;
	  
	  //horizantal angle of view of [0:360] and vertical of [-40:60]
	  phi = 360.0 - phi;
	  phi = phi * 2.0 * M_PI / 360.0;
	  theta -= 90;
	  theta *= -1;
	  theta *= 2.0 * M_PI / 360.0;
	  //normalize the reflectance
	  reflectance += 32;
	  reflectance /= 64;
	  reflectance -= 0.2;
	  reflectance /= 0.3;
	  if (reflectance < 0) reflectance = 0;
	  if (reflectance > 1) reflectance = 1;
	  
	  (*it)[0] = x * -100;
	  (*it)[1] = y * 100;
	  (*it)[2] = z * 100;
	  (*it)[3] = range;
	  (*it)[4] = theta;
	  (*it)[5] = phi;
	  (*it)[6] = reflectance;
	  
	  //finding min and max of z
	  if (z * 100 > zmax) zmax = z * 100;
	  if (z * 100 < zmin) zmin = z * 100;

	  ++it;
	}
      scan_in.close();
      scan_in.clear();
    }
  if(sFormat == RXP)
    {
      double euler[6];
      vector < ::Point> ptss;
      ScanIO_rxp *rxp = new ScanIO_rxp();
      rxp->readScan(sNumber, scan_name, -1, -1, euler, ptss);
      num_pts = ptss.size();
      cout<<"loading "<<scan_name<<" with scan number " <<sNumber<<" with "<<num_pts<<" points."<<endl;
      S.create(ptss.size(),1,CV_32FC(7));
      MatIterator_<Vec7f> it, end;
      it = S.begin<Vec7f>();
      end = S.end<Vec7f>();
      for(unsigned int i = 0; i < ptss.size(); i++)
	{
	  float x,y,z,range,theta,phi,reflectance;
	  double kart[3], polar[3];
	  x = ptss[i].x;
	  y = ptss[i].y;
	  z = ptss[i].z;
	  kart[0] = z/100;
	  kart[1] = x/-100;
	  kart[2] = y/100;
	  toPolar(kart, polar);
	  theta = polar[0]*180/M_PI;
	  phi = polar[1]*180/M_PI;
	  range = polar[2];
	  reflectance = ptss[i].reflectance;
	  //horizantal angle of view of [0:360] and vertical of [-40:60]
	  phi = 360.0 - phi;
	  phi = phi * 2.0 * M_PI / 360.0;
	  theta -= 90;
	  theta *= -1;
	  theta *= 2.0 * M_PI / 360.0;
	  //normalize the reflectance
	  reflectance += 32;
	  reflectance /= 64;
	  reflectance -= 0.2;
	  reflectance /= 0.3;
	  if (reflectance < 0) reflectance = 0;
	  if (reflectance > 1) reflectance = 1;
	  
	  (*it)[0] = x;
	  (*it)[1] = y;
	  (*it)[2] = z;
	  (*it)[3] = range;
	  (*it)[4] = theta;
	  (*it)[5] = phi;
	  (*it)[6] = reflectance;

	  //finding min and max of z
	  if (z > zmax) zmax = z;
	  if (z < zmin) zmin = z;
	  
	  ++it;
	} 
      delete rxp;
    }
  if(sFormat == ZF)
    {
      ifstream scan_in;
      vector < ::Point> ptss;
      ::Point pt;
      scan_in.open(scan_name.c_str(), ios::in);
      if (scan_in.good())
      {
        scan_in >> num_pts;
	//cout<<"loading "<<scan_name<<" with "<<num_pts<<" points."<<endl;
      }
      else
	return 0;
      while(scan_in.good())
	{
	  scan_in >> pt.x >> pt.z >> pt.y >> pt.reflectance;// range >> theta >> phi >> reflectance;
	  ptss.push_back(pt);
	} 
      num_pts = ptss.size();
      cout<<"loading "<<scan_name<<" with "<<num_pts<<" points."<<endl;
      S.create(ptss.size(),1,CV_32FC(7));
      MatIterator_<Vec7f> it, end;
      it = S.begin<Vec7f>();
      end = S.end<Vec7f>();
      for(unsigned int i = 0; i < ptss.size(); i++)
	{
	  float x,y,z,range,theta,phi,reflectance;
	  double kart[3], polar[3];
	  x = ptss[i].x;
	  y = ptss[i].y;
	  z = ptss[i].z;
	  kart[0] = x;
	  kart[1] = z;
	  kart[2] = y;
	  toPolar(kart, polar);
	  theta = polar[0]*180/M_PI;
	  phi = polar[1]*180/M_PI;
	  range = polar[2];
	  reflectance = ptss[i].reflectance;
	  //horizantal angle of view of [0:360] and vertical of [-40:60]
	  phi = 360.0 - phi;
	  phi = phi * 2.0 * M_PI / 360.0;
	  theta -= 90;
	  theta *= -1;
	  theta *= 2.0 * M_PI / 360.0;
	  if (reflectance != 0.195)
	    {  
	      //normalize the reflectance
	      //reflectance += 32;
	      //reflectance /= 64;
	      //reflectance -= 0.2;
	      //reflectance /= 0.3;
	      //enhancement
	      reflectance -= 0.2;
	      reflectance = reflectance * 1.25;
	      
	      if (reflectance < 0) reflectance = 0;
	      if (reflectance > 1) reflectance = 1;
	      
	      (*it)[0] = x * 100;
	      (*it)[1] = y * 100;
	      (*it)[2] = z * 100;
	      (*it)[3] = range;
	      (*it)[4] = theta;
	      (*it)[5] = phi;
	      (*it)[6] = reflectance;
	   
	      //finding min and max of z
	      if (z * 100 > zmax) zmax = z * 100;
	      if (z * 100 < zmin) zmin = z * 100;
	      
	      ++it;
	    }
	}
      scan_in.close();
      scan_in.clear();
    }
  return 1;
}

/**
 * panorama_creator : create panorama images with different projection methods from input scan files(Mat from scan_reader) in Mat format 
 * @param S input scan in Mat format from scan_reader step
 * @param iReflectance panorama image from reflectance data
 * @param I panorama map of 3D cartesian coordinate of input scan(same points as iRange and iReflectance)
 * @param iWidth width of the panorama image (cols in opencv)
 * @param iHeight height of panorama image (rows in opencv)
 * @param pMethod projection method for panorama creation
 * @param nImage number of images per scan for Rectilinear, Pannini and Stereographic projections
 * @param dPannini special d parameter of Pannini projection (Master Thesis for more info)
 * @param rStereographic special R parameter of Stereographic projection (Master Thesis for more info)
 * @param zmax max value of z direction for zaxis projection
 * @param zmin min value of z direction for zaxis projection
 * @return 1 on success 0 on failure
 */
int panorama_creator(Mat S, Mat& iReflectance, Mat& I,  int iWidth, int iHeight, int pMethod, int nImage, double dPannini, double rStereographic, double& zmax, double& zmin)
{
  Mat iRange(iHeight,iWidth,CV_32FC(1), Scalar::all(0));

  //EQUIRECTANGULAR projection
  if(pMethod == EQUIRECTANGULAR)
    {
      //adding the longitude to x axis and latitude to y axis
      double xmaxt = (double) iWidth / 2 / M_PI;
      int xmax = iWidth - 1;
      double ymaxt = (double) iHeight / ((MAX_ANGLE - MIN_ANGLE) / 360 * 2 * M_PI);
      //shift all the valuse to positive points on image 
      double ylow =(0 - MIN_ANGLE) / 360 * 2 * M_PI;
      int ymax = iHeight - 1;
      
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
	  int x = (int) ( xmaxt * (*it)[5]);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ( ymaxt * ((*it)[4] + ylow) );
	  y = ymax - y;
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  
	  //adding the point with max distance
	  if( iRange.at<float>(y,x) < (*it)[3] )
            {
	      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
	      iRange.at<float>(y,x) = (*it)[3];//range
	      _I(y,x)[0] = (*it)[0];//x
	      _I(y,x)[1] = (*it)[1];//y
	      _I(y,x)[2] = (*it)[2];//z
	    }
        }
      I = _I;
      iRange.release();
      return 1;
    }
  
  //CYLINDRICAL projection
  if(pMethod == CYLINDRICAL)
    {
      //adding the longitude to x and tan(latitude) to y
      //find the x and y range
      double xmaxt = (double) iWidth / 2 / M_PI;
      int xmax = iWidth - 1;
      double ymaxt = (double) iHeight / (tan(MAX_ANGLE / 360 * 2 * M_PI) - tan(MIN_ANGLE / 360 * 2 * M_PI));
      double ylow = (MIN_ANGLE) / 360 * 2 * M_PI;
      int ymax = iHeight - 1;
      
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
	  int x = (int) ( xmaxt * (*it)[5]);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ((double) ymaxt * (tan((*it)[4]) - tan(ylow)));
	  y = ymax - y;
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  
	  //adding the point with max distance
	  if( iRange.at<float>(y,x) < (*it)[3] )
            {
	      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
	      iRange.at<float>(y,x) = (*it)[3];//range
	      _I(y,x)[0] = (*it)[0];//x
	      _I(y,x)[1] = (*it)[1];//y
	      _I(y,x)[2] = (*it)[2];//z
	    }
        }
      I = _I;
      iRange.release();
      return 1;
    }
  
  //Mercator Projection
  if( pMethod == MERCATOR)
    {
      //find the x and y range
      double xmaxt = (double) iWidth / 2 / M_PI;
      int xmax = iWidth - 1;
      double ymaxt = (double) iHeight / ( log( tan( MAX_ANGLE / 360 * 2 * M_PI ) + ( 1 / cos( MAX_ANGLE / 360 * 2 * M_PI ) ) ) - log ( tan( MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI) ) ) );
      double ylow = log(tan(MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI)));
      int ymax = iHeight - 1;
      
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
	  int x = (int) ( xmaxt * (*it)[5]);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  int y = (int) ( ymaxt * (log(tan((*it)[4]) + (1/cos((*it)[4]))) - ylow) );
	  y = ymax - y;
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;

	  //adding the point with max distance
	  if( iRange.at<float>(y,x) < (*it)[3] )
            {
	      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
	      iRange.at<float>(y,x) = (*it)[3];//range
	      _I(y,x)[0] = (*it)[0];//x
	      _I(y,x)[1] = (*it)[1];//y
	      _I(y,x)[2] = (*it)[2];//z
	    }
        }
      I = _I;
      iRange.release();
      return 1;
    }

  //RECTILINEAR projection
  if(pMethod == RECTILINEAR)
    {
      //default value for nImage
      if(nImage == 0) nImage = 3;
      cout<<"Number of images per scan is: "<<nImage<<endl;
      double l0, p1, iMinx, iMaxx, iMiny, iMaxy, interval;
      interval = 2 * M_PI / nImage;
      //iMiny = -2*M_PI/9;
      //iMiny = -M_PI/6;
      iMiny = -M_PI/9;
      //iMaxy = M_PI/3;
      iMaxy = 2*M_PI/9;
      //latitude of projection center
      p1 = 0;

      //go through all points 
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
     	  for(int j = 0 ; j < nImage ; j++)
	    {
	      iMinx = j * interval;
	      iMaxx = (j + 1) * interval;
	      //check for point in interval
	      if((*it)[5] < iMaxx && (*it)[5] > iMinx)
		{
		  double max, min, coscRectilinear;
		  //the longitude of projection center
		  l0 = iMinx + interval / 2;
		  //finding the min and max of the x direction
		  coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
		  max = (cos(iMaxy) * sin(iMaxx -l0) / coscRectilinear);
		  coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
		  min = (cos(iMiny) * sin(iMinx - l0) / coscRectilinear);
		  double xmaxt = (double) (iWidth / nImage) / (max - min);
		  double xlow = min;
		  int xmax = (iWidth / nImage) - 1;
		  //finding the min and max of y direction
		  coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
		  max = ( (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0) )/ coscRectilinear);
		  coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
		  min = ( (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0) )/ coscRectilinear);
		  double ymaxt = (double) iHeight / (max - min);
		  double ylow = min;
		  int ymax = iHeight - 1;
		  //project the points and add them to image
		  coscRectilinear = sin(p1) * sin((*it)[4]) + cos(p1) * cos((*it)[4]) * cos((*it)[5] - l0);
		  int x = (int)(xmaxt) * ((cos((*it)[4]) * sin((*it)[5] - l0) / coscRectilinear) - xlow);
		  if (x < 0) x = 0;
		  if (x > xmax) x = xmax;
		  x = x + (j * iWidth / nImage);
		  int y = (int) (ymaxt) * (( (cos(p1) * sin((*it)[4]) - sin(p1) * cos((*it)[4]) * cos((*it)[5] - l0)) / coscRectilinear) - ylow);
		  y = ymax - y;
		  if (y < 0) y = 0;
		  if (y > ymax) y = ymax;
		  
		  //adding the point with max distance
		  if( iRange.at<float>(y,x) < (*it)[3] )
		    {
		      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
		      iRange.at<float>(y,x) = (*it)[3];//range
		      _I(y,x)[0] = (*it)[0];//x
		      _I(y,x)[1] = (*it)[1];//y
		      _I(y,x)[2] = (*it)[2];//z
		    }
		}
	    }
	}
      I = _I;
      iRange.release();
      return 1;
    }
  
  //PANNINI projection
  if(pMethod == PANNINI)
    {
      //default values for nImage and dPannini
      if(dPannini == 0) dPannini = 1;
      if(nImage == 0) nImage = 3;
      cout << "Parameter d is:" << dPannini <<", Number of images per scan is:" << nImage << endl;
      double l0, p1, iMinx, iMaxx, iMiny, iMaxy, interval;
      interval = 2 * M_PI / nImage;
      //iMiny = -2*M_PI/9;
      //iMiny = -M_PI/6;
      iMiny = -M_PI/9;
      //iMaxy = M_PI/3;
      iMaxy = 2*M_PI/9;
      //latitude of projection center
      p1 = 0;
      
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
	  for(int j = 0 ; j < nImage ; j++)
	    {
	      iMinx = j * interval;
	      iMaxx = (j + 1) * interval;
	      //check for point in interval
	      if((*it)[5] < (iMaxx) && (*it)[5] > (iMinx))
		{
		  double max, min, sPannini;
		  //the longitude of projection center
		  l0 = iMinx + interval / 2;
		  //use the S variable of pannini projection mentioned in the thesis
		  //finding the min and max of the x direction
		  sPannini = (dPannini + 1) / (dPannini + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
		  max = sPannini * (sin(iMaxx - l0));
		  sPannini = (dPannini + 1) / (dPannini + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
		  min = sPannini * (sin(iMinx - l0));
		  double xmaxt = (double) (iWidth / nImage) / (max - min);
		  double xlow = min;
		  int xmax = (iWidth / nImage) - 1;
		  //finding the min and max of y direction
		  sPannini = (dPannini + 1) / (dPannini + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
		  max = sPannini * (tan(iMaxy) * (cos(p1) - sin(p1) * 1/tan(iMaxy) * cos(iMaxx - l0)));
		  sPannini = (dPannini + 1) / (dPannini + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
		  min = sPannini * (tan(iMiny) * (cos(p1) - sin(p1) * 1/tan(iMiny) * cos(iMinx - l0)));
		  double ymaxt = (double) iHeight / (max - min);
		  double ylow = min;
		  int ymax = iHeight - 1;
		  //project the points and add them to image
		  sPannini = (dPannini + 1) / (dPannini + sin(p1) * tan((*it)[4]) + cos(p1) * cos((*it)[5] - l0));
		  int x = (int)(xmaxt) * (sPannini * sin((*it)[5] - l0) - xlow);
		  if (x < 0) x = 0;
		  if (x > xmax) x = xmax;
		  x = x + (j * iWidth / nImage);
		  int y = (int) (ymaxt) * ( (sPannini * tan((*it)[4]) * (cos(p1) - sin(p1) * (1/tan((*it)[4])) * cos((*it)[5] - l0) ) ) - ylow );
		  y = ymax - y;
		  if (y < 0) y = 0;
		  if (y > ymax) y = ymax;
		  
		  //adding the point with max distance
		  if( iRange.at<float>(y,x) < (*it)[3] )
		    {
		      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
		      iRange.at<float>(y,x) = (*it)[3];//range
		      _I(y,x)[0] = (*it)[0];//x
		      _I(y,x)[1] = (*it)[1];//y
		      _I(y,x)[2] = (*it)[2];//z
		    }
		}
	    }
	}
      I = _I;
      iRange.release();
      return 1;
    }

  //STEREOGRAPHIC projection
  if(pMethod == STEREOGRAPHIC)
    {
      //default values for nImage and dPannini
      if(rStereographic == 0) rStereographic = 2;
      if(nImage == 0) nImage = 3;
      cout << "Paremeter R is:" << rStereographic << ", Number of images per scan is:" << nImage << endl;
      // l0 and p1 are the center of projection iminx, imaxx, iminy, imaxy are the bounderis of intervals
      double l0, p1, iMinx, iMaxx, iMiny, iMaxy, interval;
      interval = 2 * M_PI / nImage;
      //iMiny = -2*M_PI/9;
      //iMiny = -M_PI/6;
      iMiny = -M_PI/9;
      //iMaxy = M_PI/3;
      iMaxy = 2*M_PI/9;
      //latitude of projection center
      p1 = 0;
      
      //go through all points
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
	  for ( int j = 0 ; j < nImage ; j++)
	    {
	      iMinx = j * interval;
	      iMaxx = (j + 1) * interval;
	      //check for point in intervals
	      if((*it)[5] < (iMaxx) && (*it)[5] > (iMinx))
		{
		  double max, min, k;
		  //longitude of projection center
		  l0 = iMinx + interval / 2;
		  //use the R variable of stereographic projection mentioned in the thesis
		  //finding the min and max of x direction
		  k = (2 * rStereographic) / (1 + sin(p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMaxx - l0));
		  max = k * cos(p1) * sin (iMaxx - l0);
		  k = (2 * rStereographic) / (1 + sin (p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMinx -l0));
		  min = k * cos(p1) * sin (iMinx -l0);
		  double xmaxt = (double) (iWidth / nImage) / (max - min);
		  double xlow = min;
		  int xmax = (iWidth / nImage) - 1;
		  //finding the min and max of y direction
		  k = (2 * rStereographic) / (1 + sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0));
		  max = k * (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0));
		  k = (2 * rStereographic) / (1 + sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0));
		  min = k * (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0));
		  double ymaxt = (double) iHeight / (max - min);
		  double ylow = min;
		  int ymax = iHeight - 1;
		  //project the points and add them to image
		  k = (2 * rStereographic) / (1 + sin(p1) * sin((*it)[4]) + cos(p1) * cos((*it)[4]) * cos((*it)[5] - l0));
		  int x = (int) (xmaxt) * (k * cos((*it)[4]) * sin((*it)[5] - l0) - xlow);
		  if (x < 0) x = 0;
		  if (x > xmax) x = xmax;
		  x = x + (j * iWidth / nImage);
		  int y = (int) (ymaxt) * (k * ( cos(p1) * sin((*it)[4]) - sin(p1) * cos((*it)[4]) * cos((*it)[5] - l0) ) - ylow);
		  y = ymax - y;
		  if (y < 0) y = 0;
		  if (y > ymax) y = ymax;
		  
		  //adding the point with max distance
		  if( iRange.at<float>(y,x) < (*it)[3] )
		    {
		      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
		      iRange.at<float>(y,x) = (*it)[3];//range
		      _I(y,x)[0] = (*it)[0];//x
		      _I(y,x)[1] = (*it)[1];//y
		      _I(y,x)[2] = (*it)[2];//z
		    }
		}
	    }
	}
      I = _I;
      iRange.release();
      return 1;
    }

  //ZAXIS projection
  if(pMethod == ZAXIS)
    {
      zmin = -200;
      zmax = 4000;
      //adding the longitude to x axis and latitude to y axis
      double xmaxt = (double) iWidth / 2 / M_PI;
      int xmax = iWidth - 1;
      cout << "ZMAX= " << zmax << " ZMIN= "<< zmin << endl;
      double ymaxt = (double) iHeight / (zmax - zmin);
      //shift all the valuse to positive points on image 
      double ylow = zmin;
      int ymax = iHeight - 1;
      
      MatIterator_<Vec7f> it, end; 
      Mat_<Vec3f> _I = I;
      
      for( it = S.begin<Vec7f>(), end = S.end<Vec7f>(); it != end; ++it)
	{
	  int x = (int) ( xmaxt * (*it)[5]);
	  if (x < 0) x = 0;
	  if (x > xmax) x = xmax;
	  ///////////////////check this
	  int y = (int) ( ymaxt * ((*it)[1] - ylow) );
	  y = ymax - y;
	  if (y < 0) y = 0;
	  if (y > ymax) y = ymax;
	  
	  //adding the point with max distance
	  if( iRange.at<float>(y,x) < (*it)[3] )
            {
	      iReflectance.at<uchar>(y,x) = (*it)[6]*255;//reflectance
	      iRange.at<float>(y,x) = (*it)[3];//range
	      _I(y,x)[0] = (*it)[0];//x
	      _I(y,x)[1] = (*it)[1];//y
	      _I(y,x)[2] = (*it)[2];//z
	    }
        }
      I = _I;
      iRange.release();
      return 1;
    }

  return 0;
}

/**
 * feature_detector : detect the keypoints on panorama images with different detectors
 * @param pImage panorama image of input scan (Mat format)
 * @keypoints vector holding the detected features
 * @dMethod feature detection method
 * @return 1 on success 0 on failure
 */
int feature_detector(Mat pImage, vector<KeyPoint>& keypoints, int dMethod)
{
  //Detect the keypoints using SURF Detector
  if(dMethod == SURF)
    {
      double minHessian = 400;
      SurfFeatureDetector detector(minHessian);
      detector.detect(pImage, keypoints);
      return 1;
    }
  
  //Detect the keypoints using SIFT Detector
  if(dMethod == SIFT)
    {
      SiftFeatureDetector detector;
      detector.detect(pImage, keypoints);
      return 1;
    }

  //Detect the keypoints using ORB Detector
  if(dMethod == ORB)
    {
      OrbFeatureDetector detector;
      detector.detect(pImage, keypoints);
      return 1;
    }
  
  //Detect the keypoints using FAST Detector
  if(dMethod == FAST)
    {
      FastFeatureDetector detector;
      detector.detect(pImage, keypoints);
      return 1;
    }
  
  //Detect the keypoints using STAR Detector
  if(dMethod == STAR)
    {
      StarFeatureDetector detector;
      detector.detect(pImage, keypoints);
      return 1;
    }

  return 0;
}

/**
 * feature_descriptor : create descriptor for keypoints detected on panorama images
 * @param pImage panorama image of input scan (Mat format)
 * @param keypoints vector holding the detected features
 * @param descriptor Mat holding the descriptors of keypoints
 * @param dMethod feature detection method
 * @return 1 on success 0 on failure
 */
int feature_descriptor(Mat pImage, vector<KeyPoint> keypoints, Mat& descriptor, int dMethod)
{
  //Create descriptor using SURF
  if(dMethod == SURF)
    {
      SurfDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptor);
      return 1;
    }
  
  //Create descriptor using SIFT
  if(dMethod == SIFT)
    {
      SiftDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptor);
      return 1;
    }

  //Create descriptor using ORB
  if(dMethod == ORB)
    {
      OrbDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptor);
      return 1;
    }

  return 0;
}

/**
 * descriptor_matcher : matching descriptor vectors
 * @param qDescriptor the query descriptor (Mat format)
 * @param tDescriptor the train descriptor (Mat format)
 * @param matches vector<DMatch> holding found matches
 * @param dMethod descriptor method
 * @param mMethod matching method
 * @param knn special parameter for KNN matching method, the number if nearest neighbor
 * @param radius special parameter for radius matching method, max distance to find matches
 * @param num_matches number of matches
 * @param num_filter_matches number of matches after filteration
 * @return 1 on success 0 on failure
 */
int descriptor_matcher(Mat qDescriptor, vector<KeyPoint> qKeypoints, Mat tDescriptor, vector<KeyPoint> tKeypoints, vector<DMatch>& matches, int dMethod, int mMethod, int knn, double radius, int& num_matches, int& num_filter_matches )
{
  vector< DMatch > qtInitialMatches, tqInitialMatches, gMatches;
  vector<vector<DMatch> > qtInitialMatchesVector, tqInitialMatchesVector;
  //Matching descriptors using one of the mMethods for SURF and SIFT feature descriptors
  if(dMethod == SURF || dMethod == SIFT)
    {
      if(mMethod == KNN)
	{
	  FlannBasedMatcher matcher;
	  matcher.knnMatch(qDescriptor, tDescriptor, qtInitialMatchesVector, knn);
	  matcher.knnMatch(tDescriptor, qDescriptor, tqInitialMatchesVector, knn);
	}
      if(mMethod == RADIUS)
	{
	  FlannBasedMatcher matcher;
	  matcher.radiusMatch(qDescriptor, tDescriptor, qtInitialMatchesVector, radius);
	  matcher.radiusMatch(tDescriptor, qDescriptor, tqInitialMatchesVector, radius);
	}
      if(mMethod == KNN || mMethod == RADIUS)
	{
	  //find the matches that has been found in both way
	  for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++)
	    {
	      for(unsigned int j = 0; j < qtInitialMatchesVector[i].size(); j++)
		{
		  DMatch forward = qtInitialMatchesVector[i][j];
		  for(unsigned int k = 0; k < tqInitialMatchesVector[forward.trainIdx].size(); k++)
		    {
		      DMatch backward = tqInitialMatchesVector[forward.trainIdx][k];
		      if(backward.trainIdx == forward.queryIdx)
			matches.push_back(forward);
		    }
		  //matches.push_back(qtInitialMatchesVector[i][j]);
		}
	    }
	}
      if(mMethod == RATIO)
	{
	  FlannBasedMatcher matcher;
	  matcher.knnMatch(qDescriptor, tDescriptor, qtInitialMatchesVector, 2);
	  for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++)
	    {
	      float ratio = qtInitialMatchesVector[i][0].distance/qtInitialMatchesVector[i][1].distance;
	      if(ratio < 0.8)
		matches.push_back(qtInitialMatchesVector[i][0]);
	    }	
	  matcher.knnMatch(tDescriptor, qDescriptor, tqInitialMatchesVector, 2);
	  for(unsigned int i = 0; i < tqInitialMatchesVector.size(); i++)
	    {
	      float ratio = tqInitialMatchesVector[i][0].distance/tqInitialMatchesVector[i][1].distance;
	      if(ratio < 0.8)
		{
		  DMatch tq_qt;
		  tq_qt.queryIdx = tqInitialMatchesVector[i][0].trainIdx;
		  tq_qt.trainIdx = tqInitialMatchesVector[i][0].queryIdx;
		  tq_qt.imgIdx = tqInitialMatchesVector[i][0].imgIdx;
		  tq_qt.distance = tqInitialMatchesVector[i][0].distance;
		  matches.push_back(tq_qt);
		}
	    }	
	}
      if(mMethod == BRUTEFORCE)
	{
	  BruteForceMatcher< L2<float> > matcher;
	  matcher.match(qDescriptor, tDescriptor, qtInitialMatches);
	  matcher.match(tDescriptor, qDescriptor, tqInitialMatches);
	}
      if(mMethod == FLANN)
	{
	  FlannBasedMatcher matcher;
	  matcher.match(qDescriptor, tDescriptor, qtInitialMatches);
	  matcher.match(tDescriptor, qDescriptor, tqInitialMatches);
	}
      if(mMethod == FLANN || mMethod == BRUTEFORCE)
	{
	  //add the intersection of both way matches
	  for(unsigned int i = 0; i < qtInitialMatches.size(); i++)
	    {
	      for(unsigned int j =0 ; j<tqInitialMatches.size(); j++)
		{
		  if(qtInitialMatches[i].queryIdx == tqInitialMatches[j].trainIdx && qtInitialMatches[i].trainIdx == tqInitialMatches[j].queryIdx)
		    {
		      matches.push_back(qtInitialMatches[i]);    
		    }
		}
	    }
	}
    }
       
  //Matching descriptors using BruteFore with Hamming distance for ORB descriptor
  if(dMethod == ORB)
    {
      if(mMethod == KNN)
	{
	  BruteForceMatcher< Hamming > matcher;
	  matcher.knnMatch(qDescriptor, tDescriptor, qtInitialMatchesVector, knn);
	  matcher.knnMatch(tDescriptor, qDescriptor, tqInitialMatchesVector, knn);
	}
      if(mMethod == RADIUS)
	{
	  BruteForceMatcher< Hamming > matcher;
	  matcher.radiusMatch(qDescriptor, tDescriptor, qtInitialMatchesVector, radius);
	  matcher.radiusMatch(tDescriptor, qDescriptor, tqInitialMatchesVector, radius);
	}
      if(mMethod == KNN || mMethod == RADIUS)
	{
	  for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++)
	    {
	      for(unsigned int j = 0; j < qtInitialMatchesVector[i].size(); j++)
		{
		  DMatch forward = qtInitialMatchesVector[i][j];
		  for(unsigned int k = 0; k < tqInitialMatchesVector[forward.trainIdx].size(); k++)
		    {
		      DMatch backward = tqInitialMatchesVector[forward.trainIdx][k];
		      if(backward.trainIdx == forward.queryIdx)
			matches.push_back(forward);
		    }
		  //matches.push_back(qtInitialMatchesVector[i][j]);
		}
	    }
	}
      if(mMethod == RATIO)
	{
	  BruteForceMatcher< Hamming > matcher;
	  matcher.knnMatch(qDescriptor, tDescriptor, qtInitialMatchesVector, 2);
	  for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++)
	    {
	      float ratio = qtInitialMatchesVector[i][0].distance/qtInitialMatchesVector[i][1].distance;
	      if(ratio < 0.8)
		matches.push_back(qtInitialMatchesVector[i][0]);
	    }
	  matcher.knnMatch(tDescriptor, qDescriptor, tqInitialMatchesVector, 2);
	  for(unsigned int i = 0; i < tqInitialMatchesVector.size(); i++)
	    {
	      float ratio = tqInitialMatchesVector[i][0].distance/tqInitialMatchesVector[i][1].distance;
	      if(ratio < 0.8)
		{
		  DMatch tq_qt;
		  tq_qt.queryIdx = tqInitialMatchesVector[i][0].trainIdx;
		  tq_qt.trainIdx = tqInitialMatchesVector[i][0].queryIdx;
		  tq_qt.imgIdx = tqInitialMatchesVector[i][0].imgIdx;
		  tq_qt.distance = tqInitialMatchesVector[i][0].distance;
		  matches.push_back(tq_qt);
		}
	    }
	}
      if(mMethod == BRUTEFORCE)
	{
	  BruteForceMatcher< Hamming > matcher;
	  matcher.match(qDescriptor, tDescriptor, qtInitialMatches);
	  matcher.match(tDescriptor, qDescriptor, tqInitialMatches);

	  for(unsigned int i = 0; i < qtInitialMatches.size(); i++)
	    {
	      for(unsigned int j =0 ; j<tqInitialMatches.size(); j++)
		{
		  if(qtInitialMatches[i].queryIdx == tqInitialMatches[j].trainIdx && qtInitialMatches[i].trainIdx == tqInitialMatches[j].queryIdx)
		    {
		      matches.push_back(qtInitialMatches[i]);    
		    }
		}
	    }
	}
    }
  //filter the matches with RANSAC and FundementalMatrix
  vector<Point2f> points_1, points_2;
  for( unsigned int i = 0; i < matches.size(); i++ )
    {
      //Get the keypoints from the intersection of both matches
      points_1.push_back( qKeypoints[ matches[i].queryIdx ].pt );
      points_2.push_back( tKeypoints[ matches[i].trainIdx ].pt );
    }
  //calculating the fundemental matrix
  Mat fStatus;
  Mat fundementalMatrix = findFundamentalMat( points_1, points_2, FM_RANSAC, 3, 0.99, fStatus);
  MatIterator_<uchar> it, end; 
  int counter = 0;
  //get the inliers from fundemental matrix
  for( it = fStatus.begin<uchar>(), end = fStatus.end<uchar>(); it != end; ++it)
    {
      if(*it == 1)
	gMatches.push_back(matches[counter]);
      counter++;
    }
  cout<<"number of Matches: "<<matches.size()<<" number of Matches after filteration: "<<gMatches.size()<<endl;
  num_matches = matches.size();
  matches = gMatches;
  num_filter_matches = matches.size();
  return 1;
}

/**
 * get_coord : get 3D coordinate of query scan and train scan for each match
 * @param keypoints_1 vector of KeyPoints from query (first) scan
 * @param keypoints_2 vector of KeyPoints from train (second) scan
 * @param matches vector<DMatch> holding found matches
 * @param I_1 Mat containing the 3D coordinate of each match from query scan
 * @param I_2 Mat containing the 3D coordinate of each match from train scan
 * @param idx index of the match
 * @param cq Point3f for returning the 3D coordinate of query scan
 * @param ct Point3f for returning the 3D coordinate of train scan
 * @return 1 on success 0 on failure
 */
int get_coord(vector<KeyPoint> keypoints_1, vector<KeyPoint> keypoints_2, vector<DMatch> matches, Mat I_1, Mat I_2, int idx, Point3f& cq, Point3f& ct)
{
  int x, y;
  y = keypoints_1[matches[idx].queryIdx].pt.x;
  x = keypoints_1[matches[idx].queryIdx].pt.y;

  if(keypoints_1[matches[idx].queryIdx].pt.x - x > 0.5)
    x++;
  if(keypoints_1[matches[idx].queryIdx].pt.y - y > 0.5)
    y++;
  Mat_<Vec3f> _I_1 = I_1;
  float sqr = sqrt (_I_1(x,y)[0] * _I_1(x,y)[0] + _I_1(x,y)[1] * _I_1(x,y)[1] + _I_1(x,y)[2] * _I_1(x,y)[2]);
  if(sqr != 0)
    {
      cq.x = _I_1(x,y)[0];
      cq.y = _I_1(x,y)[1];
      cq.z = _I_1(x,y)[2]; 
    }
  else
    return 0;
   
  y = keypoints_2[matches[idx].trainIdx].pt.x;
  x = keypoints_2[matches[idx].trainIdx].pt.y;
  if(keypoints_2[matches[idx].trainIdx].pt.x - x > 0.5)
    x++;
  if(keypoints_2[matches[idx].trainIdx].pt.y - y > 0.5)
    y++;
  Mat_<Vec3f> _I_2 = I_2;
  sqr = sqrt (_I_2(x,y)[0] * _I_2(x,y)[0] + _I_2(x,y)[1] * _I_2(x,y)[1] + _I_2(x,y)[2] * _I_2(x,y)[2]);
  if(sqr != 0)
    {
      ct.x = _I_2(x,y)[0];
      ct.y = _I_2(x,y)[1];
      ct.z = _I_2(x,y)[2];
    }
  else
    return 0;
  
  return 1;
}

/**
 * point_array : convert 3D point3f to double array
 * @param c 3D point whith Point3f type
 * @param cd returning 3d point as double array
 */
void point_array(Point3f c, double* cd)
{
  cd[0] = c.x;
  cd[1] = c.y;
  cd[2] = c.z;
}

/**
 * coord_transform : transform a point with the align
 * @param p input train 3D point
 * @param align input transformation matrix
 * @return Point3f trasformed 3D point
 */
Point3f coord_transform(Point3f p, double* align)
{
  Point3f tp;
  tp.x = align[0]*p.x + align[4]*p.y + align[8]*p.z + align[12];
  tp.y = align[1]*p.x + align[5]*p.y + align[9]*p.z + align[13];
  tp.z = align[2]*p.x + align[6]*p.y + align[10]*p.z + align[14];
  return tp;
}

/**
 * find_align : find the best align
 * @param I_1 input Mat wich contains the 3D coordinates of each pixel of first panorama image
 * @param keypoints_1 vector<KeyPoints> containing the features of first panorama image
 * @param I_2 input Mat wich contains the 3D coordinates of each pixel of second panorama image
 * @param keypoints_2 vector<KeyPoints> containing the features of second panorama image
 * @param matches vector<DMatches> containing the matched features of both images
 * @param minD threshold for min distance between the each three points for registration process
 * @param minE threshold for min error after transformation of a point from second coordinate to first to determin the inliers
 * @param minI threshold fir min inlier to consider the align as positive 
 * @param bAlign matrix of best align
 * @param bError min error of bAlign
 * @param bErrorIdx the number of inliers for bAlign
 * @return 1 on success 0 on failure
 */
int find_align(unsigned int i, unsigned int j, unsigned int k, Mat I_1, vector<KeyPoint> keypoints_1, Mat I_2, vector<KeyPoint> keypoints_2, vector<DMatch> matches, int minD, double minE, int minI, double* bAlign, double& bError, double& bErrorIdx)
{
  Point3f c1q, c2q, c3q, c1t, c2t, c3t;
  if(i == j || i == k || j == k)
    return 0;
  //get the coordinates
  if(get_coord(keypoints_1, keypoints_2, matches, I_1, I_2, i, c1q, c1t) == 0)
    return 0;
  if(get_coord(keypoints_1, keypoints_2, matches, I_1, I_2, j, c2q, c2t) == 0)
    return 0;
  if(get_coord(keypoints_1, keypoints_2, matches, I_1, I_2, k, c3q, c3t) == 0)
    return 0;
  //check for min distance
  if(norm(c1q - c2q) < minD || norm(c1q - c3q) < minD || norm(c2q - c3q) < minD || norm(c1t - c2t) < minD || norm(c1t - c3t) < minD || norm(c2t - c3t) < minD)
    return 0;
  //calculate the centroids
  Point3f centroidq, centroidt;
  centroidq = (c1q + c2q + c3q);
  centroidt = (c1t + c2t + c3t);
  centroidq.x = centroidq.x / 3;
  centroidq.y = centroidq.y / 3;
  centroidq.z = centroidq.z / 3;
  centroidt.x = centroidt.x / 3;
  centroidt.y = centroidt.y / 3;
  centroidt.z = centroidt.z / 3;
  //put each point into double array
  double c1qd[3], c2qd[3], c3qd[3], c1td[3], c2td[3], c3td[3];
  Point3f temp;
  point_array(c1q, c1qd);
  point_array(c2q, c2qd);
  point_array(c3q, c3qd);
  point_array(c1t, c1td);
  point_array(c2t, c2td);
  point_array(c3t, c3td);
  //create PtPair and calc the align with icp6D_QUAT
  vector<PtPair> pairs;
  pairs.push_back(PtPair(c1qd, c1td));
  pairs.push_back(PtPair(c2qd, c2td));
  pairs.push_back(PtPair(c3qd, c3td));
  double align[16];
  double centroidqd[3], centroidtd[3];
  point_array(centroidq, centroidqd);
  point_array(centroidt, centroidtd);
  icp6D_QUAT q(true);
  q.Point_Point_Align(pairs, align, centroidqd, centroidtd);
  //transform the matches with align if the error is less than minerror
  double iError = 0;
  int eIdx = 0;
  for(unsigned int p = 0; p < matches.size(); p++)
    {
      if(p == i || p == j || p == k)
	continue;
      Point3f cq, ct, ct_trans;
      if(get_coord(keypoints_1, keypoints_2, matches, I_1, I_2, p, cq, ct) == 0)
	continue;
      ct_trans = coord_transform(ct, align);
      if(norm(ct_trans - cq) < minE)
	{
	  //cout<<"norm: "<<norm(ct_trans - cq)<<" eIdx: "<<eIdx<<endl;
	  iError += norm(ct_trans - cq);
	  eIdx++;
	}
    }
  //check for mininlier and find the best align
  if(eIdx > minI)
    { 
      //cout<<"eIdx: "<<eIdx<<endl;
      double aError = iError / eIdx;
      if(aError - iInfluence*eIdx < bError - iInfluence*bErrorIdx)
	{
	  bError = aError;
	  bErrorIdx = eIdx;
	  for(int a = 0; a < 16; a++)
	    bAlign[a] = align[a];
	}
    } 
  return 1;
}

/**
 * scan_registration : register pair scans and find the rotation and translation matrices
 * @param I_1 input Mat wich contains the 3D coordinates of each pixel of first panorama image
 * @param keypoints_1 vector<KeyPoints> containing the features of first panorama image
 * @param I_2 input Mat wich contains the 3D coordinates of each pixel of second panorama image
 * @param keypoints_2 vector<KeyPoints> containing the features of second panorama image
 * @param matches vector<DMatches> containing the matched features of both images
 * @param mind threshold for min distance between the each three points for registration process
 * @param mine threshold for min error after transformation of a point from second coordinate to first to determin the inliers
 * @param mini threshold fir min inlier to consider the align as positive 
 * @param output_1 outputname for first scan for outputung the .dat and .frames
 * @param output_2 outputname for second scan for outputung the .dat and .frames
 * @param dir directory of the first input to create the .dat and .frames file
 * @param rMethod registration Method
 * @param rOutput information of registration for output
 * @param pOutput information of projection for output
 * @param fOutput information of feature for output
 * @param mOutput information of matching for output
 * @param sFormat input scan Format
 * @return 1 on success 0 on failure
 */
int scan_registration(Mat I_1, vector<KeyPoint> keypoints_1, Mat I_2, vector<KeyPoint> keypoints_2, vector<DMatch> matches, int minD, double minE, int minI, string output_1, string output_2, string dir, int rMethod, string rOutput, string pOutput, string fOutput, string mOutput, int sFormat, double* bAlign, double& bError, double& bErrorIdx, string local_time)
{
  bError = minE;
  bErrorIdx = 0;
  //go through all matches
  if(rMethod == ALL)
    {
      for(unsigned int i = 0; i < matches.size(); i++)
	for(unsigned int j = 0; j < matches.size(); j++)
	  for(unsigned int k = 0; k < matches.size(); k++)
	    {
	      find_align(i, j, k, I_1, keypoints_1, I_2, keypoints_2, matches, minD, minE, minI, bAlign, bError, bErrorIdx);
	    }
    }
  //RANSAC
  if(rMethod == RANSAC)
    for(int r = 0; r < RANSACITR; r++)
      {
	if((r % (RANSACITR/10)) == 0)
	  {
	    cout<<"RANSAC iteration: "<<(r / (RANSACITR/10) + 1) * 10 <<"%"<<endl;
	  }
	unsigned int i = rand() % matches.size();
	unsigned int j = rand() % matches.size();
	unsigned int k = rand() % matches.size();
	find_align(i, j, k, I_1, keypoints_1, I_2, keypoints_2, matches, minD, minE, minI, bAlign, bError, bErrorIdx);
      }
  
  cout<<"Registration finished with besterror of: "<<bError<<" and best error index of: "<<bErrorIdx<<endl;
  cout<<"align Matrix:"<<endl;
  if(bErrorIdx > 0)
    {
      cout<<bAlign[0]<<"  "<<bAlign[4]<<"  "<<bAlign[8]<<"  "<<bAlign[12]<<endl;
      cout<<bAlign[1]<<"  "<<bAlign[5]<<"  "<<bAlign[9]<<"  "<<bAlign[13]<<endl;
      cout<<bAlign[2]<<"  "<<bAlign[6]<<"  "<<bAlign[10]<<"  "<<bAlign[14]<<endl;
      cout<<bAlign[3]<<"  "<<bAlign[7]<<"  "<<bAlign[11]<<"  "<<bAlign[15]<<endl;

      string fOutDat = dir + "/out/" + local_time + "_" + output_1 + "_" + output_2 + "_" + pOutput + "_" + fOutput + "_" + mOutput + "_" + rOutput + ".dat";
      string fOutFrames = dir + "/out/" + local_time + "_" + output_1 + "_" + output_2 + "_" + pOutput + "_" + fOutput + "_" + mOutput + "_" + rOutput + ".frames";
      cout<<"Dat Output: "<<fOutDat.c_str()<<endl;
      cout<<"Frames Output: "<<fOutFrames.c_str()<<endl;
      ofstream frames(fOutFrames.c_str());
      ofstream dat(fOutDat.c_str());
      dat << bAlign[0] << " " << bAlign[4] << " " << bAlign[8] << " " << bAlign[12] <<endl;
      dat << bAlign[1] << " " << bAlign[5] << " " << bAlign[9] << " " << bAlign[13] <<endl;
      dat << bAlign[2] << " " << bAlign[6] << " " << bAlign[10] << " " << bAlign[14] <<endl;
      dat << bAlign[3] << " " << bAlign[7] << " " << bAlign[11] << " " << bAlign[15] <<endl;

      for (int n = 0 ; n < 2 ; n++)
	{
	  for(int i = 0; i < 16; i++)
	    frames << bAlign[i] <<" ";
	  frames << "2" << endl;
	}
      frames.close();
      dat.close();
    }
  return 1;
}

//experimental
int feature_drawer(Mat& pImage_1, vector<KeyPoint>& keypoints_1, Mat& pImage_2, vector<KeyPoint>& keypoints_2, string output_1, string output_2, string pOutput, string fOutput, int iMethod, string dir, string local_time)
{
  Mat pImageKeypoints_1;
  drawKeypoints(pImage_1, keypoints_1, pImageKeypoints_1, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  Mat pImageKeypoints_2;
  drawKeypoints(pImage_2, keypoints_2, pImageKeypoints_2, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  string out;
  out = dir + "/out/" + local_time + "_" + output_1;
  if(iMethod == SCAN)
    out += pOutput;
  out += fOutput + "_keypoints.jpg";
  imwrite(out.c_str(), pImageKeypoints_1);
  //Imwrite("out/image_panorama_1_keypoints.jpg", pImageKeypoints_1);
  //out = "out/";
  out = dir + "/out/" + local_time + "_" + output_2;
  if(iMethod == SCAN)
    out += pOutput;
  out += fOutput + "_keypoints.jpg";
  imwrite(out.c_str(), pImageKeypoints_2);
  //imwrite("out/image_panorama_2_keypoints.jpg", pImageKeypoints_2);
  return 1;
}

//experimental
int match_drawer(Mat& pImage_1, vector<KeyPoint>& keypoints_1, Mat& pImage_2, vector<KeyPoint>& keypoints_2, vector<DMatch>& matches, string output_1, string output_2, string pOutput, string fOutput, int iMethod,  string mOutput, string dir, string local_time)
{
  Mat pImageMatches;
  drawMatches(pImage_1, keypoints_1, pImage_2, keypoints_2, matches, pImageMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  string out;
  out = dir + "/out/" + local_time + "_" + output_1 + "_" + output_2;
  if(iMethod == SCAN)
    out += pOutput;
  out += fOutput + mOutput + "_matches.jpg";
  imwrite(out.c_str(), pImageMatches);
  //imwrite("out/image_panorama_matches.jpg", pImageMatches);
  //for(unsigned int i = 0; i < matches.size(); i++ )
    //cout<<"Good Match "<<i<<" Keypoint 1: "<<matches[i].queryIdx<<" -- Keypoint 2: "<<matches[i].trainIdx<<endl;
  return 1;
}

//experimental
void info_yml(information& info, double bError, double bErrorIdx, double* bAlign)
{
  Mat align(16, 1, CV_32FC(1), Scalar::all(0));
  for(int i = 0 ; i < 16 ; i++)
    align.at<float>(i,0) = bAlign[i];
  
  string yml;
  yml = info.dir + "/out/opencv-yml.yml";
  FileStorage fs(yml.c_str(), FileStorage::APPEND);
  fs << "opencv_panorama" << "{";
  
  fs << "pair" << "[" << info.output_1 << info.output_2 << "]";
  
  fs << "time" << "{" << "local_time" << info.local_time << "}";
  
  fs << "param" << "{";
  fs << "DIR" << info.dir;
  if(info.iMethod == IMAGE)
    fs << "iMethod" << "IMAGE";
  if(info.iMethod == SCAN)
    {
      fs << "iMethod" << "SCAN";
      if(info.sFormat == TXT)
	fs << "sFormat" << "TXT";
      if(info.sFormat == RXP)
	fs << "sFormat" << "RXP";
    }
  if(info.pMethod == EQUIRECTANGULAR)
    fs << "pMethod" << "EQUIRECTANGULAR";
  if(info.pMethod == CYLINDRICAL)
    fs << "pMethod" << "CYLINDRICAL";
  if(info.pMethod == MERCATOR)
    fs << "pMethod" << "MERCATOR";
  if(info.pMethod == RECTILINEAR)
    {
      fs << "pMethod" << "RECTILINEAR";
      fs << "nImage" << info.nImage;
    }
  if(info.pMethod == PANNINI)
    {
      fs << "pMethod" << "PANNINI";
      fs << "nImage" << info.nImage;
      fs << "dPannini" << info.dPannini;
    }
  if(info.pMethod == STEREOGRAPHIC)
    {
      fs << "pMethod" << "STEREOGRAPHIC";
      fs << "nImage" << info.nImage;
      fs << "rStereographic" << info.rStereographic;
    }
  fs << "iWidth" << info.iWidth;
  fs << "iHeight" << info.iHeight;
  if(info.fMethod == SURF)
    fs << "fMethod" << "SURF";
  if(info.fMethod == SIFT)
    fs << "fMethod" << "SIFT";
  if(info.fMethod == ORB)
    fs << "fMethod" << "ORB";
  if(info.fMethod == FAST)
    fs << "fMethod" << "FAST";
  if(info.fMethod == STAR)
    fs << "fMethod" << "STAR";
  if(info.dMethod == SURF)
    fs << "dMethod" << "SURF";
  if(info.dMethod == SIFT)
    fs << "dMethod" << "SIFT";
  if(info.dMethod == ORB)
    fs << "dMethod" << "ORB";
  if(info.mMethod == BRUTEFORCE)
    fs << "mMethod" << "BRUTFORCE";
  if(info.mMethod == FLANN)
     fs << "mMethod" << "FLANN";
  if(info.mMethod == KNN)
    {
      fs << "mMethod" << "KNN";
      fs << "KNN" << info.knn;
    }
  if(info.mMethod == RADIUS)
    {
      fs << "mMethod" << "RADIUS";
      fs << "RADIUS" << info.radius;
    }
  if(info.mMethod == RATIO)
     fs << "mMethod" << "RATIO";
  if(info.rMethod == RANSAC)
    fs <<"rMethod" << "RANSAC";
  if(info.rMethod == ALL)
    fs <<"rMethod" << "ALL";
  fs << "minD" << info.minD;
  fs << "minI" << info.minI;
  fs << "minE" << info.minE;
  fs << "}";

  fs <<"input"<< "{";
  fs << "first_input" << "{";
  fs << "name" << info.input_1;
  fs << "output" << info.output_1;
  fs << "point" << "{" << "amount" << info.num_pts_1 << "time" << info.iTime_1 << "}";
  fs << "projection" << "{" << "time" << info.pTime_1 << "}";
  fs << "feature" << "{" << "amount" << info.num_features_1 << "fTime" << info.fTime_1 << "dTime" << info.dTime_1 << "}";
  fs <<"}";
  fs << "second_input" << "{";
  fs << "name" << info.input_2;
  fs << "output" << info.output_2;
  fs << "point" << "{" << "amount" << info.num_pts_2 << "time" << info.iTime_2 << "}";
  fs << "projection" << "{" << "time" << info.pTime_2 << "}";
  fs << "feature" << "{" << "amount" << info.num_features_2 << "fTime" << info.fTime_2 << "dTime" << info.dTime_2 << "}";
  fs << "}";
  fs <<"}";
  
  fs << "matches" << "{";
  fs << "amount" << info.num_matches << "filteration" << info.num_filter_matches << "time" << info.mTime << "}";
  
  if(info.rMethod != DISABLE)
    {
      fs << "reg" << "{";
      fs << "bestError" << bError << "bestErrorIdx" << bErrorIdx << "time" << info.rTime << "bAlign" << align << "}";
    }

  fs << "}";
}

int main(int argc, char** argv)
{
  parseArgs(argc, argv, info);
	    //input_1, input_2, pMethod, iWidth, iHeight, nImage, dPannini, rStereographic, fMethod, dMethod, iMethod, mMethod, output_1, output_2, fOutput, pOutput, mOutput, minD, minE, minI, dir, sFormat, fScan, sScan, knn, radius, rMethod, rOutput);

  int evl;
  Mat S_1, S_2;
  Mat I_1(info.iHeight,info.iWidth,CV_32FC(3), Scalar::all(0));
  Mat I_2(info.iHeight,info.iWidth,CV_32FC(3), Scalar::all(0));
  Mat iReflectance_1(info.iHeight,info.iWidth,CV_8U, Scalar::all(0));
  Mat iReflectance_2(info.iHeight,info.iWidth,CV_8U, Scalar::all(0));
  double bAlign[16];
  double bError, bErrorIdx;
  info.zmax = numeric_limits<double>::min();
  info.zmin = numeric_limits<double>::max();
   
  if(info.iMethod == SCAN)
    {
      //reading first scan
      info.iTime_1 = (double)getTickCount();
      evl = scan_reader(info.input_1, S_1, info.sFormat, info.fScan, info.num_pts_1, info.zmax, info.zmin);
      info.iTime_1 = ((double)getTickCount() - info.iTime_1)/getTickFrequency();
      cout << "Times passed in seconds for reading first scan: " << info.iTime_1 << endl;
      if(evl != 1)
	{
	  cout<<"Error in reading the first scan"<<endl;
	  if (evl == 0)
	    cout<<"can not open the file"<<endl;
	}
      //reading second scan
      info.iTime_2 = (double)getTickCount();
      evl = scan_reader(info.input_2, S_2, info.sFormat, info.sScan, info.num_pts_2, info.zmax, info.zmin);
      info.iTime_2 = ((double)getTickCount() - info.iTime_2)/getTickFrequency();
      cout << "Times passed in seconds for reading second scan: " << info.iTime_2 << endl;
      if(evl != 1)
	{
	  cout<<"Error in reading the second scan"<<endl;
	  if (evl == 0)
	    cout<<"can not open the file"<<endl;
	}  
      
      //creating first panorama image
      info.pTime_1 = (double)getTickCount();
      evl = panorama_creator(S_1, iReflectance_1, I_1, info.iWidth, info.iHeight, info.pMethod, info.nImage, info.dPannini, info.rStereographic, info.zmax, info.zmin);
      string out_1;
      out_1 = info.dir + "/out/" + info.local_time + "_" + info.output_1 + info.pOutput + ".jpg";
      cout<<"output_1:"<<out_1<<endl;
      imwrite(out_1.c_str(), iReflectance_1);
      info.pTime_1 = ((double)getTickCount() - info.pTime_1)/getTickFrequency();
      cout << "Times passed in seconds for creating first panorama: " << info.pTime_1 << endl;
      //creating second panorama image
      info.pTime_2 = (double)getTickCount();
      evl = panorama_creator(S_2, iReflectance_2, I_2, info.iWidth, info.iHeight, info.pMethod, info.nImage, info.dPannini, info.rStereographic, info.zmax, info.zmin);
      string out_2;
      out_2 = info.dir + "/out/" + info.local_time + "_" + info.output_2 + info.pOutput + ".jpg";
      cout<<"output_2:"<<out_2<<endl;
      imwrite(out_2.c_str(), iReflectance_2);
      info.pTime_2 = ((double)getTickCount() - info.pTime_2)/getTickFrequency();
      cout << "Times passed in seconds for creating second panorama: " << info.pTime_2 << endl;
      
      iReflectance_1 = imread( out_1.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
      iReflectance_2 = imread( out_2.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
      
      //clearing mat
      S_1.release();
      S_2.release();
    }
  else
    {
      iReflectance_1 = imread( info.input_1.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
      iReflectance_2 = imread( info.input_2.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
    }
  vector<KeyPoint> keypoints_1, keypoints_2;
  //generating features of first image/scan
  info.fTime_1 = (double)getTickCount();
  evl = feature_detector(iReflectance_1, keypoints_1, info.fMethod);
  info.fTime_1 = ((double)getTickCount() - info.fTime_1)/getTickFrequency();
  info.num_features_1 = keypoints_1.size();
  cout<<"image 1 Features: "<<info.num_features_1<<" in: "<<info.fTime_1<<" seconds."<<endl;
  //generating features of second image/scan 
  info.fTime_2 = (double)getTickCount();
  evl = feature_detector(iReflectance_2, keypoints_2, info.fMethod);
  info.fTime_2 = ((double)getTickCount() - info.fTime_2)/getTickFrequency();
  info.num_features_2 = keypoints_2.size();
  cout<<"image 2 Features: "<<info.num_features_2<<" in: "<<info.fTime_2<<" seconds."<<endl;
  
  Mat descriptor_1, descriptor_2;
  //creating descriptor of first image/scan
  info.dTime_1 = (double)getTickCount();
  evl = feature_descriptor(iReflectance_1, keypoints_1, descriptor_1, info.dMethod);
  info.dTime_1 = ((double)getTickCount() - info.dTime_1)/getTickFrequency();
  cout<<"image 1 Descriptor in: "<<info.dTime_1<<" seconds."<<endl;
  //creating descriptor of second image/scan
  info.dTime_2 = (double)getTickCount();
  evl = feature_descriptor(iReflectance_2, keypoints_2, descriptor_2, info.dMethod);
  info.dTime_2 = ((double)getTickCount() - info.dTime_2)/getTickFrequency();
  cout<<"image 2 Descriptor in: "<<info.dTime_2<<" seconds."<<endl;
 
  vector <DMatch> matches;
  //matching the features 
  info.mTime = (double)getTickCount();
  evl = descriptor_matcher(descriptor_1, keypoints_1, descriptor_2, keypoints_2, matches, info.dMethod, info.mMethod, info.knn, info.radius, info.num_matches, info.num_filter_matches);
  info.mTime = ((double)getTickCount() - info.mTime)/getTickFrequency();
  cout<<"image 1 and image 2 Matches: "<<matches.size()<<" in: "<<info.mTime<<" seconds."<<endl;

  //experimental
  feature_drawer(iReflectance_1, keypoints_1, iReflectance_2, keypoints_2, info.output_1, info.output_2, info.pOutput, info.fOutput, info.iMethod, info.dir, info.local_time);
  match_drawer(iReflectance_1, keypoints_1, iReflectance_2, keypoints_2, matches, info.output_1, info.output_2, info.pOutput, info.fOutput, info.iMethod, info.mOutput, info.dir, info.local_time);

  if(info.iMethod == SCAN && info.rMethod != DISABLE)
    {
      //registering scan 1 and 2
      info.rTime = (double)getTickCount();
      evl = scan_registration(I_1, keypoints_1, I_2, keypoints_2, matches, info.minD, info.minE, info.minI, info.output_1, info.output_2, info.dir, info.rMethod, info.rOutput, info.pOutput, info.fOutput, info.mOutput, info.sFormat, bAlign, bError, bErrorIdx, info.local_time);
      info.rTime = ((double)getTickCount() - info.rTime)/getTickFrequency();
      cout<<"Registration of Scan 1 and 2 in: "<<info.rTime<<" seconds."<<endl;
    }


  //experimental
  info_yml(info, bError, bErrorIdx, bAlign);
}

