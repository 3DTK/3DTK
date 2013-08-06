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
#include "slam6d/basicScan.h"

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
  scanner_type sType;
  int MIN_ANGLE, MAX_ANGLE;
  bool iSizeOptimization;
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
  printf("\t\t-N numberOfImage\t\t number of Horizontal images used for some projections\n");
  printf("\t\t-P pParam\t\t special projection parameter (d for Pannini and r for stereographic)\n");
  printf("\t\t-O outDir \t\t output directory if not stated same as input\n");
  printf("\t\t-r scan reduction \t\t reduces the scan size\n");
  printf("\t\t-R reflectance \t\t uses reflactance and creates the Reflectance image\n");
  printf("\t\t-C color \t\t uses color and creates the Color image\n");
  printf("\t\t-A range \t\t creates the Range image\n");
  printf("\t\t-S scale \t\t Scale\n");
  printf("\t\t-l loadOct \t\t load the Octtree\n");
  printf("\t\t-o saveOct \t\t save the Octtree\n");
  printf("\t\t-t sType \t\t scanner type\n");
  printf("\t\t-n MIN_ANGLE \t\t Scanner vertical view MIN_ANGLE \n");
  printf("\t\t-x MAX_ANGLE \t\t Scanner vertical view MAX_ANGLE \n");
  printf("\t\t-i iSizeOptimization \t\t Optimize the panorama image size based on projection \n");
  printf("\n");
  exit(1);
}

void parssArgs(int argc, char** argv, information& info){
  info.iSizeOptimization = false;
  info.reflectance = false;
  info.range = false;
  info.color = false;
  info.sType = NONE;
  info.MIN_ANGLE = -40;
  info.MAX_ANGLE = 60;
  
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
  while ((c = getopt (argc, argv, "W:H:p:N:P:f:O:s:e:r:RCAS:lot:n:x:i")) != -1)
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
      case 't':
	info.sType = stringToScannerType(optarg);
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
      case 'n':
	info.MIN_ANGLE = atoi(optarg);
	break;
      case 'x':
	info.MAX_ANGLE = atoi(optarg);
	break;
      case 'i':
	info.iSizeOptimization = true;
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
    if(info.numberOfImages < 2) info.numberOfImages = 2;
  }
  if(info.pMethod == STEREOGRAPHIC && info.pParam == 0){
    info.pParam = 2;
    if(info.numberOfImages < 2) info.numberOfImages = 2;
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
  for(int s = info.start; s <= info.end; s++){
    scan_cv scan(info.dir, s, info.sFormat, scanserver, info.sType, info.loadOct, info.saveOct, info.reflectance, info.color);

    scan.convertScanToMat();

    //init the panorama
    fbr::panorama pImage;
    pImage.init(info.pWidth, info.pHeight, info.pMethod, info.numberOfImages, info.pParam, info.mapMethod, scan.getZMin(), scan.getZMax(), info.MIN_ANGLE, info.MAX_ANGLE, info.iSizeOptimization);
    
    //create panorama
    pImage.createPanorama(scan.getMatScan(), scan.getMatScanColor());  
    
    //get the new panorama image size incase of optimized panorama size
    info.pWidth = pImage.getImageWidth();
    info.pHeight = pImage.getImageHeight();

    //write panorama to file
    string out;
    
    if(info.range){
      out = info.outDir+to_string(s, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.pWidth)+"x"+to_string(info.pHeight)+"_Range.jpg";
      imwrite(out, pImage.getRangeImage());
    }

    if(info.reflectance){
      out = info.outDir+to_string(s, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.pWidth)+"x"+to_string(info.pHeight)+"_Reflectance.jpg";
      imwrite(out, pImage.getReflectanceImage());
    }
    
    if(info.color){
      out = info.outDir+to_string(s, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.pWidth)+"x"+to_string(info.pHeight)+"_Color.jpg";
      imwrite(out, pImage.getColorImage());
    }

  }

}
