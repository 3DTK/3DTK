/*
 * kitti2scan implementation
 *
 * Copyright (C) Shitong Du
 *
 * Released under the GPL version 3.
 *
 */


#include <fstream>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::string;
using std::ios;

#include "slam6d/globals.icc"
#include <string.h>
#include <string>
#ifndef _MSC_VER
#include <unistd.h>
#else
#include "XGetopt.h"
#endif

#if WIN32
#define snprintf sprintf_s
#endif

int parseArgs(int argc,char **argv, char dir[255], int& start, int& end,int& sequence){
  start   = 0;
  end     = -1; // -1 indicates no limitation
  sequence=0;
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  cout << endl;
  while ((c = getopt (argc, argv, "s:e:q:")) != -1)
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
   case 'q':
     sequence = atoi(optarg);
     if (sequence < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
     if (sequence > 21) { cerr << "Error: There are only 21 point clouds sequences.\n"; exit(1); }
     break;
   }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***\n" << endl;
    cout << endl
	  << "Usage: " << argv[0] << "  [-s NR] [-e NR] directory" << endl << endl;

    cout << "  -s NR   start at scan NR (i.e., neglects the first NR scans)" << endl
       << "          [ATTENTION: counting starts with 0]" << endl
	  << "  -e NR   end after scan NR" << "" << endl
            << "  -q NR   set sequence NR" << "" << endl
	  << endl;
    cout << "Reads bin files from directory/???.bin and converts them to directory/scan???.3d." << endl;
    abort();
  }
  strncpy(dir,argv[optind],255);


#ifndef _MSC_VER
  if (dir[strlen(dir)-1] != '/') strcat(dir,"/");
#else
  if (dir[strlen(dir)-1] != '\\') strcat(dir,"\\");
#endif
  return 0;
}


int main(int argc, char **argv)
{
  int start = 0, end =-1;
  int sequence=0;
  char dir[255];

  float calibMatrix[12];
  float  inputpoint[3],outpoint[3];
  parseArgs(argc, argv, dir, start,end,sequence);
  int fileCounter=start;
  string inDir=dir;
  string calibFileName = inDir +to_string(sequence,2)+'/'+"calib"+ ".txt";
  string b="tr";
  string a;
  ifstream pose_in;


  pose_in.open(calibFileName);
  if (!pose_in.good())  // no more files in the directory

   { cerr << "Error: no a calibration file in the directory.\n"; exit(1); }

    while(pose_in.good()) {

    getline(pose_in,a,':');
    if(b.compare(a))

    for (unsigned int i = 0; i < 12; pose_in >> calibMatrix[i++]);
    }

    pose_in.close();
    pose_in.clear();
    for(int i=0;i<12;i++)
    cout << "Reading calibfile " << calibMatrix[i] << "..." << endl;

    for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read

    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));
  // pointers
   float *px = data+0;
   float *py = data+1;
   float *pz = data+2;
   float *pr = data+3;
   string str = inDir +to_string(sequence,2)+'/'+"velodyne/"+to_string(fileCounter,6) + ".bin" ;
   const char *pointFileName = str.c_str();
   // char *pointFileName = "/media/achim/D46CA2BB6CA2982E/dataset/KITTI/KITTI_data_set/00/velodyne/0000000000.bin";
   string FileName3d = inDir +to_string(sequence,2)+'/'+"scan"+to_string(fileCounter,3) + ".3d" ;

    FILE *stream;
    stream = fopen(pointFileName,"rb");
    num = fread(data,sizeof(float),num,stream)/4;

     cout << "Reading bin " << pointFileName << "..." << endl;
    ofstream pose_out;
    pose_out.open(FileName3d, ios::out);
    for (int32_t j=0; j<num; j++)
    {

     inputpoint[0]=(*px);
     inputpoint[1]=(*py);
     inputpoint[2]=(*pz) ;
     //cout << inputpoint[0]<< endl;
     outpoint[0] = calibMatrix[0]*inputpoint[0]+calibMatrix[1]*inputpoint[1]+calibMatrix[2]*inputpoint[2]+calibMatrix[3]*1.0;
     outpoint[1] = calibMatrix[4]*inputpoint[0]+calibMatrix[5]*inputpoint[1]+calibMatrix[6]*inputpoint[2]+calibMatrix[7]*1.0;
     outpoint[2] = calibMatrix[8]*inputpoint[0]+calibMatrix[9]*inputpoint[1]+calibMatrix[10]*inputpoint[2]+calibMatrix[11]*1.0;
     //cout << outpoint[0]<< endl;

     pose_out<< outpoint[0]*100<<" ";
     pose_out<<-(outpoint[1]*100)<<" ";
     pose_out<< outpoint[2]*100<<" "<<endl;//正常写入

     //cout << outpoint[0] <<" "<<outpoint[1]<<" "<<outpoint[2]<<endl;
     px+=4; py+=4; pz+=4; pr+=4;

    }

      pose_out.close();
     //pose_out.clear();
      fileCounter++;
      fclose(stream);
      free(data);

    }
     return 0;
}

