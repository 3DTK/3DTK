/**
* readerscan.cpp
*
* Created by: Darko
*      Maintained : HamidReza May 10, 2011
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif

#include "Reader_RIEGL.h"

using namespace std;

/**
 *   usage - explains how to use the program CMD
 */
void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s [ -r RIEGL ] [ -d SCAN_ID ]-o OUTPUT.ppc INPUT_FILE\n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-r RIEGL\t\ttype of scans (for now just RIEGL is supported)\n");
  printf("\t\t-d scanid\t\tscan id, default is the scan filename\n");
  printf("\t\t-o \t\t\twrite to a .ppc file (serialization of PolarPointCloud class):\n");
  printf("\n");
  printf("\n");
  
  exit(1);
}

/**
 *   main - reads the input and create polarpointcloud and write it to output
 */
int main(int argc, char** argv)
{
  char *reader = NULL;
  char *input = NULL;
  char *output = NULL;
  char *scanid = NULL;
  
  int c;
  
  //reade the comand line and get the options
  while ((c = getopt (argc, argv, "o:d:")) != -1)
    switch (c)
      {
      case 'o':
	output = optarg;
	break;
      case 'd':
	scanid = optarg;
	break;
      case '?':
	if (optopt == 'o' || optopt == 'd')
	  fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	else
	  fprintf (stderr, "Unknown option character `%c'.\n", optopt);
	return 1;
      case ':':
      default:
	usage(argc, argv);
      }
  
  //check the input data of comand line
  if (output == NULL)
    {
      usage(argc, argv);
    }
  if (optind != argc - 1)
    {
      usage(argc, argv);
    }
  
  //put the non option parameter to input
  if (reader == NULL || !(strcmp(reader, "RIEGL") == 0))
    {
      reader = "RIEGL";
    }
  input = argv[optind];
  if (scanid == NULL)
    {
      scanid = input;
    }
  cout<<scanid<<endl;
  
  //create the polarpoin cloud from input flie and write it to .ppc file
  PolarPointCloud polarcloud = Reader_RIEGL::readPolarPointCloud(scanid, input, 1);
  polarcloud.serialize(output);
  
  //experimental------
  int ind[100];
  for (int i = 0 ; i < 100 ; i++) 
    {
      ind[i] = 0;
    }
  for (int i = 0 ; i < polarcloud.length ; i++) 
    {
      int index = (int)((*polarcloud.data)[i].r * 100.0);
      if (index < 0) index = 0;
      if (index > 99) index = 99;
      ind[index]++;
    }
  ofstream of("plot");
  for (int i = 0 ; i < 100 ; i++) 
    {
      of << i << " " << ind[i] << endl;
    }
  of.close();
  //-----
  
  return 0;
}
