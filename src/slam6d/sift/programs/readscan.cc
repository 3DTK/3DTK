/**
 * @file readscan.cc
 * @brief read scans and create polarpointcloud
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
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif

#include "slam6d/sift/library/Reader_RIEGL.h"

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
 *   parseArgs - reade the comand line options
 */
int parseArgs(int argc, char **argv, string &reader, string &input, string &output, string &scanid)
{
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
  if(output.empty())  
    {
      usage(argc, argv);
    }
  if (optind != argc - 1)
    {
      usage(argc, argv);
    }
  
  //put the non option parameter to input
  if (reader.empty() || reader.compare("RIEGL") != 0 )
    {
      reader = "RIEGL";
    }
  input = argv[optind];
  if (scanid.empty())
    {
      scanid = input;
    }
  return 1;
}

/**
 *   main - reads the input and create polarpointcloud and write it to output
 */
int main(int argc, char** argv)
{
  string reader;
  string input;
  string output;
  string scanid;

  parseArgs(argc, argv, reader, input, output, scanid); 
  cout<<endl;
  cout<<"readerscan will proceed with the following parameters:"<<endl;
  cout<<"reader: "<<reader<<endl;
  cout<<"input: "<<scanid<<endl;
  cout<<"output: "<<output<<endl;
  cout<<"scanid: "<<scanid<<endl<<endl;
  
  //create the polarpoin cloud from input flie and write it to .ppc file
  PolarPointCloud polarcloud = Reader_RIEGL::readPolarPointCloud(scanid, input, 1);
  polarcloud.serialize(output.c_str());
  
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
