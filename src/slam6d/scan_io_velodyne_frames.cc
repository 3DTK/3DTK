/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH
 * @author Dorit Borrmann. Smart Systems Group, Jacobs University Bremen gGmbH, Germany. 
 */

#include "slam6d/scan_io_velodyne_frames.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

#include <algorithm>
using std::swap;

#include<cstdio>
#include<cstdlib>
#include<cmath>
#include<cstring>
#include <errno.h>

using namespace std;


#ifdef _MSC_VER
#include <windows.h>
#endif

#ifdef _MSC_VER
#include "XGetopt.h"
#else
#include <getopt.h>
#endif


#define BLOCK_OFFSET 42+16

#define BLOCK_SIZE 1206
#define CIRCLELENGTH 260
#define VELODYNE_NUM_LASERS 64
#define CircleBufferSize CIRCLELENGTH*32*12
#define CIRCLEROUND CIRCLELENGTH*6


#define RADIANS_PER_LSB 0.0174532925
#define METERS_PER_LSB 0.002
#define METERS_PER_CM 0.01

#define TWOPI_INV (0.5/M_PI)
#define TWOPI (2*M_PI)

typedef struct raw_packet
  {
    unsigned char dat[1200];
    unsigned short revolution;
    unsigned char status[4]; 
  } raw_packet_t;


typedef unsigned char BYTE;



struct velodyne_sample
{
    float  xyz[3];             // calibrated, projected into velodyne coordinate system
    float  distance;          
    float  corredistance;      

    unsigned short rot;       
    float  rotational;        
    BYTE   intensity;       

    float  theta,phi;             
    int    offset;           

   
    float   normal;         
    float   normal_theta;     
    float   smooth;

    float   scanline_drtZ;
    float   scanline_drtD;
    float   scanline_countZ;
    float   scanline_countD;

    /////////////////////////Type of Points cloud/////////////////////////////////
    int classType;  // 0 nused , 1 road , 2 build , 3 Tree , 4 person ,5 car , 6 ground
};


long CountOfLidar = 0;


double velodyne_calibrated[VELODYNE_NUM_LASERS][5] =
{
    //vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    {  -7.158120,  -4.954240, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 0
    {  -6.817820,  -2.814700, 0.000000 ,  0.000000 ,   4.000000 }, // laser 1
    {   0.317822,   2.814740, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 2
    {   0.658119,   4.954230, 0.000000 ,  0.000000 ,   4.000000 }, // laser 3
    {  -6.477650,  -0.679162, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 4
    {  -6.137590,   1.455470, 0.000000 ,  0.000000 ,   4.000000 }, // laser 5
    {  -8.520810,  -1.455470, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 6
    {  -8.179890,   0.679162, 0.000000 ,  0.000000 ,   4.000000 }, // laser 7
    {  -5.797640,   3.592120, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 8
    {  -5.457770,   5.733800, 0.000000 ,  0.000000 ,   4.000000 }, // laser 9
    {  -7.839140,   2.814740, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 10
    {  -7.498560,   4.954200, 0.000000 ,  0.000000 ,   4.000000 }, // laser 11
    {  -3.080210,  -4.954240, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 12
    {  -2.740630,  -2.814700, 0.000000 ,  0.000000 ,   4.000000 }, // laser 13
    {  -5.117980,  -5.733800, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 14
    {  -4.778260,  -3.592120, 0.000000 ,  0.000000 ,   4.000000 }, // laser 15
    {  -2.401040,  -0.679162, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 16
    {  -2.061410,   1.455470, 0.000000 ,  0.000000 ,   4.000000 }, // laser 17
    {  -4.438590,  -1.455470, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 18
    {  -4.098960,   0.679162, 0.000000 ,  0.000000 ,   4.000000 }, // laser 19
    {  -1.721740,   3.592120, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 20
    {  -1.382020,   5.733800, 0.000000 ,  0.000000 ,   4.000000 }, // laser 21
    {  -3.759370,   2.814740, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 22
    {  -3.419790,   4.954240, 0.000000 ,  0.000000 ,   4.000000 }, // laser 23
    {   0.998555,  -4.954240, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 24
    {   1.339140,  -2.814740, 0.000000 ,  0.000000 ,   4.000000 }, // laser 25
    {  -1.042230,  -5.733800, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 26
    {  -0.702363,  -3.592120, 0.000000 ,  0.000000 ,   4.000000 }, // laser 27
    {   1.679890,  -0.679162, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 28
    {   2.020810,   1.455470, 0.000000 ,  0.000000 ,   4.000000 }, // laser 29
    {  -0.362407,  -1.455470, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 30
    {  -0.022350,   0.679162, 0.000000 ,  0.000000 ,   4.000000 }, // laser 31
    { -22.737886,  -7.443011, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 32
    { -22.226072,  -4.224233, 0.000000 ,  0.000000 ,   4.000000 }, // laser 33
    { -11.513928,   4.224233, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 34
    { -11.002114,   7.443011, 0.000000 ,  0.000000 ,   4.000000 }, // laser 35
    { -21.714685,  -1.018773, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 36
    { -21.203688,   2.183498, 0.000000 ,  0.000000 ,   4.000000 }, // laser 37
    { -24.790272,  -2.183498, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 38
    { -24.276321,   1.018773, 0.000000 ,  0.000000 ,   4.000000 }, // laser 39
    { -20.693031,  5.3926148, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 40
    { -20.182682,  8.6188126, 0.000000 ,  0.000000 ,   4.000000 }, // laser 41
    { -23.762968,  4.2242332, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 42
    { -23.250172,  7.4430108, 0.000000 ,  0.000000 ,   4.000000 }, // laser 43
    { -16.615318, -7.4430108, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 44
    { -16.105938, -4.2242332, 0.000000 ,  0.000000 ,   4.000000 }, // laser 45
    { -19.672594, -8.6188126, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 46
    { -19.162729, -5.3926148, 0.000000 ,  0.000000 ,   4.000000 }, // laser 47
    { -15.596496,  -1.018773, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 48
    { -15.086954,  2.1834979, 0.000000 ,  0.000000 ,   4.000000 }, // laser 49
    { -18.653046, -2.1834979, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 50
    { -18.143503,   1.018773, 0.000000 ,  0.000000 ,   4.000000 }, // laser 51
    { -14.577271,  5.3926148, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 52
    { -14.067405,  8.6188126, 0.000000 ,  0.000000 ,   4.000000 }, // laser 53
    { -17.634062,  4.2242332, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 54
    { -17.124681,  7.4430108, 0.000000 ,  0.000000 ,   4.000000 }, // laser 55
    { -10.489829, -7.4430108, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 56
    { -9.9770317, -4.2242332, 0.000000 ,  0.000000 ,   4.000000 }, // laser 57
    { -13.557318, -8.6188126, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 58
    { -13.046968, -5.3926148, 0.000000 ,  0.000000 ,   4.000000 }, // laser 59
    { -9.4636793,  -1.018773, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 60
    {  -8.949728,  2.1834979, 0.000000 ,  0.000000 ,   4.000000 }, // laser 61
    { -12.536313, -2.1834979, 0.000000 ,  0.000000 ,  -4.000000 }, // laser 62
    { -12.025314,   1.018773, 0.000000 ,  0.000000 ,   4.000000 }, // laser 63
};



double rotCorrection[VELODYNE_NUM_LASERS];
double vertCorrection[VELODYNE_NUM_LASERS];
double distCorrection[VELODYNE_NUM_LASERS];
double vertoffsetCorrection[VELODYNE_NUM_LASERS];
double horizdffsetCorrection[VELODYNE_NUM_LASERS];

int physical2logical[VELODYNE_NUM_LASERS];
int logical2physical[VELODYNE_NUM_LASERS];

double absf ( double a )
{
    if ( a < 0 )
        return -a;
    return a;
}


int velodyne_physical_to_logical ( int phys )
{
    return physical2logical[phys];
}

int velodyne_logical_to_physical ( int logical )
{
    return logical2physical[logical];
}

int laser_phi_compare(const void *_a, const void *_b)
{
    int a = *((int*) _a);
    int b = *((int*) _b);

    if (velodyne_calibrated[a][0] < velodyne_calibrated[b][0]) 
        return -1;

    return 1;
}

/** valid only for v > 0 **/
static inline double mod2pi_positive(double vin)
{
    double q = vin * TWOPI_INV + 0.5;
    int qi = (int) q;

    return vin - qi*TWOPI;
}

/** Map v to [-PI, PI] **/
static inline double mod2pi(double vin)
{
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

/** Return vin such that it is within PI degrees of ref **/
static inline double mod2pi_ref(double ref, double vin)
{
    return ref + mod2pi(vin - ref);
}




int velodyne_calib_precompute()
{
    int i;
    int logical;
    for ( i = 0; i < VELODYNE_NUM_LASERS; i++ )
        logical2physical[i] = i;

    qsort ( logical2physical, VELODYNE_NUM_LASERS, sizeof ( int ), laser_phi_compare );

    for ( logical = 0; logical < VELODYNE_NUM_LASERS; logical++ )
    {
        physical2logical[logical2physical[logical]] = logical;
    }

    //vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    for ( i = 0; i < VELODYNE_NUM_LASERS; i++ )
    {
       
        vertCorrection[i] = ( velodyne_calibrated[i][0] ) * RADIANS_PER_LSB;
        
        rotCorrection[i] = ( velodyne_calibrated[i][1] ) * RADIANS_PER_LSB;
       
        distCorrection[i] = velodyne_calibrated[i][2] * METERS_PER_CM;
       
        vertoffsetCorrection[i] = velodyne_calibrated[i][3] * METERS_PER_CM;
        
        horizdffsetCorrection[i] = velodyne_calibrated[i][4] * METERS_PER_CM;  
    }

    return 0;
}





int read_one_packet ( FILE *fp, vector<Point> &ptss, int maxDist, int minDist )
{
  int maxDist2 = sqr(maxDist);
  int minDist2 = sqr(minDist);
  
    int  c, i, j;
    unsigned char Head = 0;
    BYTE buf[BLOCK_SIZE];
    Point point;
    BYTE *p;
    unsigned short *ps;
    unsigned short *pshort;
    short *pt;


    double ctheta;
    double theta, phi;

    double sin_ctheta, cos_ctheta;
    double sin_theta, cos_theta;
    double sin_phi, cos_phi;

    unsigned short physicalNO;
    //unsigned short logicalNO;

    double rotational;
    double distance;
    double corredistance;
    int intensity;
    //    int physical;
	int size;


    
    //unsigned short rot;
  

    double x, y, z;
/*
    int circle_col = 0;
    int circle_row = 0;
    int circle_col_other = 0; 
    int circle_row_other = 0;
  */ 


    for ( c = 0 ; c < CIRCLELENGTH; c++ )
    {

		fseek(fp , BLOCK_OFFSET, SEEK_CUR);
        size=fread ( buf, 1, BLOCK_SIZE, fp );
		
		if(size<BLOCK_SIZE)
			return -1;
		
		
        ps = ( unsigned short * ) buf;
        p = buf;
        physicalNO = 0;


        for ( i = 0; i < 12; i++ )
        {
            //Each frame start with 0xEEFF || 0xDDFF
            if ( *ps == 0xEEFF )
                Head = 0;
            else if ( *ps == 0xDDFF )
                Head = 32;


            pshort = ( unsigned short * ) ( p + 2 );
//            rot = ( ( unsigned short ) ( *pshort ) / 100.0 );


            //circle_col =  c * 6 + i / 2;
            rotational = ( ( double ) ( *pshort ) ) / 100.0;

    


            for ( j = 0; j < 32; j++ )
            {
               
                physicalNO  = j + Head;
                //logicalNO = velodyne_physical_to_logical ( physicalNO );
/*
                circle_col =  c * 6 + i / 2;
                circle_row =  logicalNO;


                circle_col_other =  c * 6 + i / 2;  /////????
                circle_row_other =  logicalNO / 2;
*/
             
                
                pt = ( short * ) ( p + 4 + j * 3 );
                
                distance = absf ( ( *pt ) * 0.002 );

                BYTE *inty = ( BYTE * ) ( p + 4 + j * 3 + 2 );
                intensity = *inty;

                ctheta = 2 * M_PI - rotational * RADIANS_PER_LSB;
                if ( ctheta == 2*M_PI )
                    ctheta = 0;

                
                sin_ctheta = sin ( ctheta );
                cos_ctheta = cos ( ctheta );

                //vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
                corredistance = ( distance + distCorrection[physicalNO] ) * ( 1.0 + vertoffsetCorrection[physicalNO] );
                theta     = mod2pi_ref ( M_PI, ctheta + rotCorrection[physicalNO] ); /////////????////////
                phi    = vertCorrection[physicalNO];  ////////?????/////////////////


                sin_theta = sin ( theta );
                cos_theta = cos ( theta );
                sin_phi = sin ( phi );
                cos_phi = cos ( phi );

                /////////////////////?a??¡Á?¡À¨º/////////////////////
                x = corredistance * cos_theta * cos_phi;
                y = corredistance * sin_theta * cos_phi;
                z = corredistance * sin_phi;

                x -= horizdffsetCorrection[physicalNO] * cos_ctheta;
                y -= horizdffsetCorrection[physicalNO] * sin_ctheta;

         
				point.x=y;
				point.y=z;
				point.z=x;
				point.x *= -100;
                    point.y *= 100;
                    point.z *= 100;
				
				point.reflectance=intensity/256.0;

				if ((maxDist == -1 || sqr(point.x) + sqr(point.y) + sqr(point.z) < maxDist2*1.0)
				  && (minDist == -1 || sqr(point.x) + sqr(point.y) + sqr(point.z) > minDist2*1.0))
				  {
				    ptss.push_back(point);
				    //printf("%f %f %f %f %f %f\n",point.x,point.y,point.z,point.reflectance,minDist2*1.0,minDist2*1.0);
				  }
            }
            p = p + 100;
            ps = ( unsigned short * ) p;
        }


    }

    

    return 0;
}


/**
 * Reads specified scans from given directory in
 * the file format Riegl Laser Measurement GmbH 
 * uses. It will be compiled as shared lib.
 *
 * Scan poses will NOT be initialized after a call
 * to this function. Initial pose estimation works 
 * only with the -p switch, i.e., trusting the initial
 * estimations by Riegl.
 *
 * The scans have to be exported from the Riegl software
 * as follows:
 * 1. Export point cloud data to ASCII
 *    Use Scanners own Coordinate System (SOCS)
 *    X Y Z Range Theta Phi Reflectance
 * 2. Export acqusition location (after you have registered
 *    with the Riegl software)
 *    Export SOP
 *    Write out as .dat file 
 * 
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_velodyne_frame::readScans(int start, int end, string &dir, int maxDist, int minDist,
						  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;
  string framesFileName;
  

  FILE *scan_in = 0;
  FILE *pose_in = 0;
  ifstream  frames_in;
  //  int my_fileNr = fileCounter;
  double transMat[16];
  int type;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  
  scanFileName = dir + "scan"  + ".bin";
  poseFileName = dir + "scan"  + ".pose";
  framesFileName = dir + "scan" + to_string(fileCounter,3) + ".frames";
  
  scan_in = fopen(scanFileName.c_str(),"rb");  
  if(scan_in==NULL)
  {
	cerr << "ERROR: Missing file " << scanFileName <<" "<<strerror(errno)<< endl; exit(1); 
	return 0;
  }

  //pose_in = fopen(poseFileName.c_str(),"rb");  
  if(pose_in==NULL)
  {
	//cout << "no file " << poseFileName <<" "<<strerror(errno)<< endl; ; 
	
  }
  else
  {
     // cout<<"we get pose info"<<endl;
  }

  frames_in.open(framesFileName.c_str());

  // read 3D scan
 
  if (!frames_in.good()) { cerr << "ERROR: Missing file " << framesFileName << endl; exit(1); }
  
  while(frames_in) {
    try {
      frames_in >> transMat >> type;
    }
    catch(const exception &e) {
      break;
    }
  }
  Matrix4ToEuler(transMat, &euler[3], euler);

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;
  
  velodyne_calib_precompute();
  cout << "Processing Scan " << scanFileName;
  cout.flush();
  
 



#if 0
  raw_packet_t raw_tmp;
  fseeko(scan_in, 0, SEEK_SET);
  fread(&raw_tmp,sizeof(raw_packet_t),1,scan_in);
  int revolution=raw_tmp.revolution+fileCounter+1;
  int i=0;

  while(raw_tmp.revolution!=revolution)
  {
      printf("%d %d\n",i++,raw_tmp.revolution);
      fread(&raw_tmp,sizeof(raw_packet_t),1,scan_in);
  }
#else
  
   
  
  cout.flush();
  ptss.reserve(12*32*CIRCLELENGTH);
  
 #if 0
  fseeko(scan_in, 0, SEEK_SET);
  fseeko(scan_in, BLOCK_SIZE*CIRCLELENGTH*fileCounter, SEEK_CUR);
 #else
  fseek(scan_in, 24, SEEK_SET);
  fseek(scan_in, (BLOCK_SIZE+BLOCK_OFFSET)*CIRCLELENGTH*fileCounter, SEEK_CUR);
 #endif
  
#endif
  read_one_packet(scan_in, ptss, maxDist, minDist);
  cout << " with " << ptss.size() << " Points";

  
  cout << " done " << fileCounter<<endl;
  if(pose_in)
  {
      double poseinfo[6];
      fseek(pose_in, 0, SEEK_SET);
      fseek(pose_in, sizeof(double)*6*fileCounter, SEEK_CUR);
	  fread(poseinfo,sizeof(double)*6,1,pose_in);
	  euler[0] = poseinfo[0]*100 - 135701 ;
      euler[1] = poseinfo[1]*100- 842.154;
      euler[2] = poseinfo[2]*100+93003.5;
	  euler[3] = poseinfo[3];
      euler[4] = poseinfo[4];
      euler[5] = poseinfo[5];
	  
      printf("%f %f %f %f %f %f\n",poseinfo[0],poseinfo[1],poseinfo[2],poseinfo[3],poseinfo[4],poseinfo[5]);

    fclose(pose_in);
  }
  fclose(scan_in);
  fileCounter++;
  frames_in.close();
  frames_in.clear();
  
  return fileCounter-1;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) ScanIO* create()
#else
extern "C" ScanIO* create()
#endif
{
  return new ScanIO_velodyne_frame;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) void destroy(ScanIO *sio)
#else
extern "C" void destroy(ScanIO *sio)
#endif
{
  delete sio;
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	return TRUE;
}
#endif

