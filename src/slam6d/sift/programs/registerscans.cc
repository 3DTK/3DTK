/**
 * @file registerscans.cc
 * @brief Rgister two or more scans with specific parameters like:
 *        minimum distance between each two ransac points, minimun number of inliers, minimum error,...
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <ctime>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif

#include "slam6d/sift/library/FeatureMatchSetGroup.h"
#include "slam6d/sift/library/PanoramaMap.h"
#include "slam6d/sift/library/Register.h"

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
  printf("USAGE: %s -m FILE.matches MAP1.map MAP2.map [MAP3.map ...]\n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-m FILE.matches\t\tinput .matches file (serialization of a FeatureMatchSetGroup class object)\n");
  printf("\t\t-d <int>\t\t minimum number of pairs that should fit the model (default: 2)\n");
  printf("\t\t-t <float>\t\t minimum threshold distance that determines if it is a fit or not (default: 1.0)\n");
  printf("\t\t-i <float>\t\t influence of the numbers of inliers to the error term (default: 0.05)\n");
  printf("\t\t-r <float>\t\t minimum distance between any two points in a triangle (default: 5)\n");
  printf("\t\tFILE.map\t\tinput .map file (serialization of a PanoramaMap class object)\n");
  printf("\n");
   exit(1);
}

/** 
 *   parseArgs - reade the comand line options
 */
int parseArgs(int argc, char **argv, vector<char*> &input, int &d, double &t, double &i, double &r, double &a, int &modemax, vector<string> &smap)
{
  opterr = 0;
  int c;
  
  while ((c = getopt (argc, argv, "m:d:t:i:r:a:x:")) != -1)
    switch (c)
      {
      case 'x':
	modemax = atoi(optarg);
	break;
      case 'm':
	input.push_back(optarg);
	break;
      case 'd':
	d = atoi(optarg);
	if (d <= 0) 
	  {
	    cout << "Invalid integer for d\n";
	    usage(argc, argv);
	  }
	break;
      case 't':
	t = atof(optarg);
	if (t <= 0) 
	  {
	    cout << "Invalid integer for t\n";
	    usage(argc, argv);
	  }
	break;
      case 'i':
	i = atof(optarg);
	if (i <= 0) 
	  {
	    cout << "Invalid value for i\n";
	    usage(argc, argv);
	  }
	break;
      case 'r':
	r = atof(optarg);
	if (r <= 0) 
	  {
	    cout << "Invalid integer for r\n";
	    usage(argc, argv);
	  }
	break;
      case 'a':
	a = atof(optarg);
	if (a <= 0) 
	  {
	    cout << "Invalid integer for a\n";
	    usage(argc, argv);
	  }
	break;
      case '?':
	if ( optopt == 'm' || optopt == 'd' || optopt == 't' || optopt == 'i' || optopt == 'r' || optopt == 'a' )
	  fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	else
	  fprintf (stderr,"Unknown option character `%c'.\n", optopt);
	return 1;
      case ':':
      default:
	usage(argc, argv);
      }
  
  if (input.size() == 0) 
    {
      usage(argc, argv);
    }
   for (int index = optind; index < argc ; index++) 
    {
      smap.push_back(argv[index]);
    }
  return 1;
}

/**
 *  main - use match features to register scans
 */
int main(int argc, char** argv)
{
  vector<char*> input;  
  int d = 0;
  double t = 0;
  double i = 0;
  double r = 0;
  double a = 0;
  int modemax = 0;
  vector<string> smap;

  parseArgs(argc, argv, input, d, t, i, r, a, modemax, smap);
  cout<<endl;
  cout<<"registerscans will proceed with the following parameters: "<<endl;
  cout<<"input: "<<endl;
  for(int i = 0 ; i< (int) input.size() ; i++)
    {
      cout<<"   "<<input[i]<<endl;
    }
  cout<<"Minimum number of inliers 'd': "<<d<<endl;
  cout<<"Minimum threshold 't': "<<t<<endl;
  cout<<"Influence of the inliers to error 'i': "<<i<<endl;
  cout<<"Minimum distance for two point in RANSAC 'r': "<<r<<endl;
  cout<<"Modemax for RANSAC: "<<modemax<<endl;
  cout<<"Maps: "<<endl;
  for(int i = 0 ; i < (int) smap.size() ; i++)
    {
      cout<<"   "<<smap[i]<<endl;
    }
  cout<<endl;
    
  FeatureMatchSetGroup group;
  for (int i = 0 ; i < (int) input.size() ; i++) 
    {
      cout << "Reading match set group: " << input[i] << endl;
      FeatureMatchSetGroup group_2(input[i]);
      group.matchsets.merge(group_2.matchsets);
    }
  cout << group.matchsets.size() << endl;

  vector<PanoramaMap*> maps;
  for (int index = 0 ; index < (int) smap.size() ; index++)
    {
      cout << "Reading map: " << smap[index] << endl;
      PanoramaMap *map = new PanoramaMap(smap[index].c_str());
      maps.push_back(map);
    }
  
  cout << "Checking..\n";
  Register reg(&group, maps);
  if (modemax != 0) reg.mode_maximum = modemax;
  if (d != 0) reg.d = d;
  if (t != 0) reg.t = t;
  if (i != 0) reg.dinfluence = i;
  if (r != 0) reg.mind = r;
  if (a != 0) reg.mina = a / 360.0 * 2 * M_PI;
  
  cout << "Registering..\n";
  clock_t start = clock();
  reg.registerScans();
  clock_t end = clock();
  double time = ((double)((int) end - (int) start)) / CLOCKS_PER_SEC;
  
  cout << "Time needed for registering: " << time << endl;
  cout << "Freeing data...\n";
  for (int i = 0 ; i < maps.size() ; i++) 
    {
      delete maps[i];
    }
  return 0; 
}
