#include <stdio.h>
#include <fstream>
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/scan_cv.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/feature.h"
#include "slam6d/fbr/feature_matcher.h"
#include "slam6d/fbr/registration.h"
#include "slam6d/fbr/feature_drawer.h"
#include "show/scancolormanager.h"
#include "show/show_Boctree.h"
#include "slam6d/point_type.h"
#include "show/show.h"
#include "slam6d/data_types.h"
#include "slam6d/Boctree.h"
#include "slam6d/basicscan.h"

using namespace std;
using namespace fbr;

struct information{
  string dir, outDir;
  int start, end;
  int pWidth, pHeight, numberOfImages;
  double pParam;
  IOType sFormat;
  projection_method pMethod;
  panorama_map_method mapMethod;
  double red;
  double voxelSize;
  unsigned int types;
  bool saveOct, loadOct;
  double scale;
  ScanColorManager *scm;
  bool reflectance, range, color;
} info;

void usage(int argc, char** argv){
  printf("\n");
  printf("USAGE: %s dir -s start -e end \n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-f scanFormat\t\t input scan file format [RIEGL_TXT|RXP|ALL SLAM6D SCAN_IO]\n");
  printf("\t\t-W pWidth\t\t panorama image width\n");
  printf("\t\t-H pHeight\t\t panorama image height\n");
  printf("\t\t-p pMethod\t\t projection method [EQUIRECTANGULAR|CONIC|CYLINDRICAL|MERCATOR|RECTILINEAR|PANNINI|STEREOGRAPHIC|ZAXIS]\n");
  printf("\t\t-N numberOfImage\t\t number of images used for some projections\n");
  printf("\t\t-P pParam\t\t special projection parameter (d for Pannini and r for stereographic)\n");
  printf("\t\t-O outDir \t\t output directory if not stated same as input\n");
  printf("\t\t-r scan reduction \t\t reduces the scan size\n");
  printf("\t\t-R reflectance \t\t uses reflactance and creates the Reflectance image\n");
  printf("\t\t-C color \t\t uses color and creates the Color image\n");
  printf("\t\t-A range \t\t creates the Range image\n");
  printf("\t\t-S scale \t\t Scale\n");
  printf("\t\t-l loadOct \t\t load the Octtree\n");
  printf("\t\t-o saveOct \t\t save the Octtree\n");
  printf("\n");
  exit(1);
}

void parssArgs(int argc, char** argv, information& info){
  info.reflectance = false;
  info.range = false;
  info.color = false;
  
  info.red = -1.0;
  info.types = PointType::USE_NONE;
  info.scale = 0.01;
  info.saveOct = false;
  info.loadOct = false;
  //////////////////////
  info.voxelSize = 0.20 / info.scale;
  info.scm =0 ;
  info.mapMethod = fbr::EXTENDED;  
  /////////////////////
  //default values
  info.pWidth = 3600;
  info.pHeight = 1000;
  info.numberOfImages = 1;
  //depend on the projection method
  info.pParam = 0;
  //===============================
  info.sFormat = RIEGL_TXT;
  info.pMethod = EQUIRECTANGULAR;
  info.outDir = "";

  int c;
  opterr = 0;
  //reade the command line and get the options
  while ((c = getopt (argc, argv, "W:H:p:N:P:f:O:s:e:r:RCAS:lo")) != -1)
    switch (c)
      {
      case 's':
	info.start = atoi(optarg);
	break;
      case 'e':
	info.end = atoi(optarg);
	break;
      case 'f':
	info.sFormat = stringToScanFormat(optarg);
	break;
      case 'W':
	info.pWidth = atoi(optarg);
	break;
      case 'H':
        info.pHeight = atoi(optarg);
	break;
      case 'p':
	info.pMethod = stringToProjectionMethod(optarg);
	break;
      case 'N':
	info.numberOfImages = atoi(optarg);
	break;
      case 'P':
	info.pParam = atoi(optarg);
	break;
      case 'O':
	info.outDir = optarg;
	break;
      case 'r':
	info.red = atoi(optarg);
	break;
      case 'R':
	info.types |= PointType::USE_REFLECTANCE;
	info.reflectance = true;
	break;
      case 'C':
	info.types |= PointType::USE_COLOR;
	info.color = true;
	break;
      case 'A':
	info.range = true;
	break;
      case 'S':
	info.scale = atoi(optarg);
	break;
      case 'l':
	info.loadOct = true;
	break;
      case 'o':
	info.saveOct = true;
	break;
      case '?':
	cout<<"Unknown option character "<<optopt<<endl;
	usage(argc, argv);
	break;
      default:
	usage(argc, argv);
      }

  if(info.pMethod == PANNINI && info.pParam == 0){
    info.pParam = 1;
    if(info.numberOfImages < 3) info.numberOfImages = 3;
  }
  if(info.pMethod == STEREOGRAPHIC && info.pParam == 0){
    info.pParam = 2;
    if(info.numberOfImages < 3) info.numberOfImages = 3;
  }
  if(info.pMethod == RECTILINEAR && info.numberOfImages < 3)
    info.numberOfImages = 3;

  if (optind > argc - 1)
    {
      cout<<"Too few input arguments. At least dir and two scan numbers are required."<<endl;
      usage(argc, argv);
    }
    
  info.dir = argv[optind];
  if(info.outDir.empty()) info.outDir = info.dir;
  else if(info.outDir.compare(info.outDir.size()-1, 1, "/") != 0) info.outDir += "/";
}

void printInfo(information info){
  PointType t = PointType(info.types);
  cout<<"Dir= "<<info.dir<<endl;
  cout<<"Start= "<<info.start<<endl;
  cout<<"End= "<<info.end<<endl;
  cout<<"Scan Format= "<<scanFormatToString(info.sFormat)<<endl;
  cout<<"Width= "<<info.pWidth<<endl;
  cout<<"Height= "<<info.pHeight<<endl;
  cout<<"Projection Method= "<<projectionMethodToString(info.pMethod)<<endl;
  cout<<"Number of Images= "<<info.numberOfImages<<endl;
  cout<<"Projection Param= "<<info.pParam<<endl;
  cout<<"Out Dir= "<<info.outDir<<endl;
  cout<<"Scan Reduction= "<<info.red<<endl;
  cout<<"Reflectance= "<<t.hasReflectance()<<endl;
  cout<<"Color= "<<t.hasColor()<<endl;
  cout<<"Range= "<<info.range<<endl;
  cout<<"Scale= "<<info.scale<<endl;
  cout<<"Load Oct= "<<info.loadOct<<endl;
  cout<<"Save Oct= "<<info.saveOct<<endl;
  cout<<"Voxel Size= "<<info.voxelSize<<endl;
  cout<<"Map Method= "<<panoramaMapMethodToString(info.mapMethod)<<endl;
}

int main(int argc, char** argv)
{
  parssArgs(argc, argv, info);
  
  printInfo(info);

  //opendirectory of the scans
  bool scanserver = false;
  Scan::openDirectory(scanserver, info.dir, info.sFormat, info.start, info.end);  
  
  int number = info.start;
  //get the firs scan
  for (unsigned int s = 0 ; s < Scan::allScans.size(); s++){
    PointType pointtype = PointType(info.types);
    //if we want to load display file get pointtypes from the files first
    if(info.loadOct) {
      string scanFileName = info.dir + "scan" + to_string(number,3) + ".oct";
      pointtype = BOctTree<sfloat>::readType(scanFileName);
    }

    info.scm = new ScanColorManager(4096, pointtype);  

    Scan * source = Scan::allScans[s];
    //set the parameters for the oct
    source->setOcttreeParameter(info.red, info.voxelSize, pointtype, info.loadOct, info.saveOct);
    
    //loading oct
    BOctTree<float>* btree = ((BasicScan*)source)->convertScanToShowOcttree();
    
    // show structures
    // associate show octtree with the scan and hand over octtree pointer ownership
    Show_BOctTree<sfloat>* tree = new Show_BOctTree<sfloat>(btree, info.scm);
    
    //get points from octtree and create panorama images
    vector<float*> points;
    tree->getTree()->AllPoints(points);
        
    unsigned int nPoints = points.size();
    
    //create the scan and color Mats
    cv::Mat scan, scanColor;
    cv::MatIterator_<cv::Vec4f> it;
    scan.create(nPoints,1,CV_32FC(4));
    scan = cv::Scalar::all(0);
    it = scan.begin<cv::Vec4f>();

    cv::MatIterator_<cv::Vec3f> itColor;
    if(pointtype.hasColor()){
      scanColor.create(nPoints,1,CV_32FC(3));
      scanColor = cv::Scalar::all(0);
      itColor = scanColor.begin<cv::Vec3f>();
    }

    float zMax = numeric_limits<double>::min(); 
    float zMin = numeric_limits<double>::max();

    //get the info from points
    for(unsigned int i = 0; i < nPoints; i++){
      (*it)[0] = points[i][0]; // x
      (*it)[1] = points[i][1]; // y
      (*it)[2] = points[i][2]; // z

      //finding min and max of z                                      
      if (points[i][2]  > zMax) zMax = points[i][2];
      if (points[i][2]  < zMin) zMin = points[i][2];

      float reflectance;
      if(pointtype.hasReflectance()){
	int idx = pointtype.getReflectance();
	reflectance = points[i][idx];
      }
      else
	reflectance = 255;

      //normalize the reflectance
      //reflectance += 32;
      //reflectance /= 64;
      //reflectance -= 0.2;
      //reflectance /= 0.3;
      //reflectance -= 1200;
      //reflectance /= 600;

      if (reflectance < 0) reflectance = 0;
      if (reflectance > 1) reflectance = 1;
      (*it)[3] = reflectance;

      if(pointtype.hasColor())
	{
	  int idx = pointtype.getColor();
	  (*itColor)[2] = ((unsigned char*) &(points[i][idx]))[0];
	  (*itColor)[1] = ((unsigned char*) &(points[i][idx]))[1];
	  (*itColor)[0] = ((unsigned char*) &(points[i][idx]))[2];
	  ++itColor;
	}
      ++it;
    }
    
    //print the zMin and zMax
    cout<<"zMin = "<<zMin<<endl;
    cout<<"zMax = "<<zMax<<endl;
    
    //init the panorama
    fbr::panorama pImage;
    pImage.init(info.pWidth, info.pHeight, info.pMethod, info.numberOfImages, info.pParam, info.mapMethod, zMin, zMax);
    
    //create panorama
    pImage.createPanorama(scan, scanColor);  
    
    //write panorama to file
    string out;
    
    if(info.range){
      out = info.outDir+to_string(number, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.pWidth)+"x"+to_string(info.pHeight)+"_Range.jpg";
      imwrite(out, pImage.getRangeImage());
    }

    if(info.reflectance){
      out = info.outDir+to_string(number, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.pWidth)+"x"+to_string(info.pHeight)+"_Reflectance.jpg";
      imwrite(out, pImage.getReflectanceImage());
    }
    
    if(info.color){
      out = info.outDir+to_string(number, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.pWidth)+"x"+to_string(info.pHeight)+"_Color.jpg";
      imwrite(out, pImage.getReflectanceImage());
    }

    number++;
  }
  Scan::closeDirectory();
}
