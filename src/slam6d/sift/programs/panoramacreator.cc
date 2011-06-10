/**
 * @file panoramacreator.cc
 * @brief reads ppc files and create .map and .japg files
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <ctime>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif

#include "slam6d/sift/library/Reader_RIEGL.h"
#include "slam6d/sift/library/PanoramaMap.h"

using namespace std;

struct size 
{
  int width;
  int height;
};

/**
 *   usage - explains how to use the program CMD
 */
void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s [-m] [-j] [-d] [-v] [-n] [-R] [-p] -s <WIDTHxHEIGHT> [-s <WIDTHxHEIGHT> ...] FILE1.ppc [FILE2.ppc] ...\n",argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-m \t\t\twrite to a .map file (serialization of PanoramaMap class):\n");
  printf("\t\t-d \t\t\twrite an Range image to Range.jpg file:\n");
  printf("\t\t-j \t\t\twrite an Refelectance image to Refelectance.jpg file:\n");
  printf("\t\t-p \t\t\tselect the projection for create the map:\n");
  printf("\t\t-v \t\t\tthe d parameter for pannini projection,default is 1:\n");
  printf("\t\t-R \t\t\tthe R parameter for stereographic projection,default is 1:\n");
  printf("\t\t-n \t\t\tthe number of images per scan ,default is 4:\n"); 
  printf("\t\t-s <WxH>\t\tsize of output in widthxheight\n");
  printf("\t\t\t\t\tinput files should be serialization of PolarPointCloud class returned by readscan\n");
  printf("\n");
  printf("\tExample:\n");
  printf("\t\t%s -m -j -d -v -p PANNINI -s 500x150 example.file\n");
  printf("\n");
  exit(1);
}

/**
 *   parseArgs - reade the comand line options
 */
int parseArgs(int argc, char **argv, string &reader, int &method, string &projection, bool &writeMaps, bool &writeRangeImage, bool &writeRefelectanceJpeg, string &input,vector<struct size> &sizes, double &d, int &n, double &r)
{
  int c; 
  int width;
  int height;
  opterr = 0;
  //reade the command line and get the options
  while ((c = getopt (argc, argv, "r:mjdR:v:n:s:p:")) != -1)
    switch (c)
      {
      case 'p':
	projection = optarg;
	break;
      case 'v':
	d = atof(optarg);
	break;
      case 'R':
	r = atof(optarg);
	break;
      case 'n':
	n = atoi(optarg);
	break;
      case 'r':
	reader = optarg;
	break;
      case 'd':
	writeRangeImage = true;
	break;
      case 'm':
	writeMaps = true;
	break;
      case 'j':
	writeRefelectanceJpeg = true;
	break;
      case 's':
	char *pch;
	pch = strtok(optarg, "x");
	width = atoi(pch);
	if (width <= 0 || pch == NULL) {printf("Invalid size\n"); usage(argc, argv);}
	pch = strtok(NULL, "x");
	if (pch == NULL) {printf("Invalid size\n"); usage(argc, argv);}
	height = atoi(pch);
	if (height <= 0) {printf("Invalid size\n"); usage(argc, argv);}
	size sz;
	sz.width = width;
	sz.height = height;
	sizes.push_back(sz);
	break;
      case '?':
	if (optopt == 'r' || optopt == 'w' || optopt == 'h')
	  fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	else
	  fprintf (stderr,"Unknown option character `%c'.\n",optopt);
	return 1;
      case ':':
      default:
	usage(argc, argv);
      }
  
  if (optind == argc)
    {
      usage(argc, argv);
    }
  
  //check the reader type for now only RIEGL
  if (reader.empty() || reader.compare("RIEGL") != 0)
    {
      reader = "RIEGL";
    }
 
  //check for projection
  if (projection.empty()|| ( (projection.compare("STANDARD") != 0) && (projection.compare("CYLINDRICAL") != 0) && (projection.compare("NEW") != 0) && (projection.compare("MERCATOR") != 0) && (projection.compare("RECTILINEAR") != 0) && (projection.compare("PANNINI") != 0) && (projection.compare("STEREOGRAPHIC") != 0)))   
    {
      projection = "STANDARD";
    }
  if(projection.compare("STANDARD") == 0)
    method = STANDARD;
  if(projection.compare("CYLINDRICAL") == 0)
    method = CYLINDRICAL;
  if(projection.compare("NEW") == 0)
    method = NEW;
  if(projection.compare("MERCATOR") == 0)
    method = MERCATOR;
  if(projection.compare("RECTILINEAR") == 0)
    method = RECTILINEAR;
  if(projection.compare("PANNINI") == 0)
    method = PANNINI;
  if(projection.compare("STEREOGRAPHIC") == 0)
    method = STEREOGRAPHIC;
  if (!writeMaps && !writeRefelectanceJpeg && !writeRangeImage)
    {
      printf("One of the options -m or -j or -d must be included\n");
      usage(argc, argv);
    }
  input = argv[optind];
  return 1;
}

/**
 *   main - reads the inputs and create the panorama images and map file
 */
int main(int argc, char** argv)
{  
  string reader;
  int method;
  string projection;
  bool writeMaps = false;
  bool writeRangeImage = false;
  bool writeRefelectanceJpeg = false;
  string input;
  vector<struct size> sizes;
  double d = 1;
  int n = 4;
  double r = 1;
  
  parseArgs(argc, argv, reader, method, projection, writeMaps, writeRangeImage, writeRefelectanceJpeg, input, sizes, d, n, r);
  cout<<endl;
  cout<<"panoramacreator will proceed with the following parameters: "<<endl;
  cout<<"Reader: "<<reader<<endl;
  cout<<"Projection: "<<projection<<endl;
  cout<<"Maps: "<<writeMaps<<endl;  
  cout<<"RangeImage: "<<writeRangeImage<<endl;
  cout<<"RefelectanceImage: "<<writeRefelectanceJpeg<<endl;  
  cout<<"Input: "<<input<<endl;
  cout<<endl;

  clock_t start, end;
  vector<struct size>::iterator it;
  char outputmap[1000];
  char outputjpg[1000];
  PolarPointCloud polarcloud(input.c_str());
  //creating the map and refelectance and range images
  for (it = sizes.begin() ; it != sizes.end() ; it++ )
    {
      cout << "Creating Map " << it->width << " " << it->height << endl;
      start = clock();
      PanoramaMap map(&polarcloud, it->width, it->height, method, d, n, r);
      end = clock();
      double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;
      cout << "Finished creating map with time: " << time << endl;
      if (writeMaps)
	{
	  sprintf(outputmap, "%s_%dx%d.map", input.c_str(), it->width, it->height);
	  map.serialize(outputmap);
	  printf("Wrote map to: %s\n", outputmap);
	}
      if (writeRangeImage)
	{
	  sprintf(outputjpg, "%s_%dx%d_Range_%d.jpg", input.c_str(), it->width, it->height, method);
	  map.toJpeg(outputjpg, RANGE);
	  printf("Wrote image to: %s\n", outputjpg);
	}
      if (writeRefelectanceJpeg)
	{
	  sprintf(outputjpg, "%s_%dx%d_Refelectance_%d.jpg", input.c_str(), it->width, it->height, method);
	  map.toJpeg(outputjpg, REFELCTANCE);
	  printf("Wrote image to: %s\n", outputjpg);
	}
    }
  return 0;
}
