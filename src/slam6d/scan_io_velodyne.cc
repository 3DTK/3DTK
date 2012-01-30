/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH
 * @author Dorit Borrmann. Smart Systems Group, Jacobs University Bremen gGmbH, Germany. 
 */

#include "slam6d/scan_io_velodyne.h"
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

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif


#define BLOCK_OFFSET 42+16

#define BLOCK_SIZE 1206
#define CIRCLELENGTH 360
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

#if 0
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
#else
double velodyne_calibrated[VELODYNE_NUM_LASERS][5] = 
{
{	-7.1581192,	-4.5,	102,	21.560343,	-2.5999999},
{	-6.8178215,	-3.4000001,	125,	21.516994,	2.5999999},
{	0.31782165,	3,	130,	20.617426,	-2.5999999},
{	0.65811908,	4.5999999,	128,	20.574717,	2.5999999},
{	-6.4776502,	-0.5,	112,	21.473722,	-2.5999999},
{	-6.1375928,	1,	125,	21.430525,	2.5999999},
{	-8.520812,	-1.5,	106,	21.734608,	-2.5999999},
{	-8.1798887,	0.40000001,	127,	21.690901,	2.5999999},
{	-5.797637,	4,	111,	21.387396,	-2.5999999},
{	-5.4577708,	5.5,	126,	21.34433,	2.5999999},
{	-7.8391404,	3.0999999,	113,	21.647291,	-2.5999999},
{	-7.4985547,	4.5,	123,	21.603773,	2.5999999},
{	-3.0802133,	-4.5,	105,	21.044245,	-2.5999999},
{	-2.7406337,	-3.2,	133,	21.001518,	2.5999999},
{	-5.1179824,	-5.5,	110,	21.301321,	-2.5999999},
{	-4.7782598,	-4,	129,	21.258366,	2.5999999},
{	-2.4010365,	-0.2,	111,	20.958813,	-2.5999999},
{	-2.0614092,	1,	130,	20.916126,	2.5999999},
{	-4.4385905,	-1.2,	115,	21.215462,	-2.5999999},
{	-4.0989642,	0,	133,	21.172602,	2.5999999},
{	-1.7217404,	3.8,	113,	20.873451,	-2.5999999},
{	-1.3820176,	5,	130,	20.830786,	2.5999999},
{	-3.7593663,	3,	117,	21.129782,	-2.5999999},
{	-3.4197867,	4.5,	129,	21.086998,	2.5999999},
{	0.998555,	-4.5,	107,	20.531982,	-2.5999999},
{	1.339141,	-3.2,	131,	20.489222,	2.5999999},
{	-1.0422293,	-5.4000001,	128,	20.788124,	-2.5999999},
{	-0.70236301,	-4,	134,	20.745461,	2.5999999},
{	1.679889,	-0.5,	124,	20.446428,	-2.5999999},
{	2.0208123,	1,	136,	20.403601,	2.5999999},
{	-0.36240739,	-1.5,	131,	20.702793,	-2.5999999},
{	-0.022349782,	0.2,	136,	20.660116,	2.5999999},
{	-22.737886,	-7.8000002,	101,	16.019152,	-2.5999999},
{	-22.226072,	-5,	88,	15.954137,	2.5999999},
{	-11.513928,	4.5,	121,	14.680806,	-2.5999999},
{	-11.002114,	7.4000001,	88,	14.623099,	2.5999999},
{	-21.714685,	-1,	94,	15.889649,	-2.5999999},
{	-21.203688,	2,	88,	15.82566,	2.5999999},
{	-24.790272,	-2.5,	114,	16.284933,	-2.5999999},
{	-24.276321,	0.5,	89,	16.217583,	2.5999999},
{	-20.693031,	6,	98,	15.762167,	-2.5999999},
{	-20.182682,	9,	92,	15.699132,	2.5999999},
{	-23.762968,	4.5,	107,	16.15085,	-2.5999999},
{	-23.250172,	7.5,	80,	16.084715,	2.5999999},
{	-16.615318,	-7.5,	121,	15.26925,	-2.5999999},
{	-16.105938,	-5,	92,	15.209245,	2.5999999},
{	-19.672594,	-9,	119,	15.63654,	-2.5999999},
{	-19.162729,	-6,	89,	15.574372,	2.5999999},
{	-15.596496,	-1,	109,	15.14954,	-2.5999999},
{	-15.086954,	2,	88,	15.090119,	2.5999999},
{	-18.653046,	-2,	117,	15.51261,	-2.5999999},
{	-18.143503,	0.69999999,	88,	15.451235,	2.5999999},
{	-14.577271,	5.5,	112,	15.030966,	-2.5999999},
{	-14.067405,	8.3999996,	87,	14.972065,	2.5999999},
{	-17.634062,	5,	119,	15.390228,	-6.1999998},
{	-17.124681,	7.5,	97,	15.329572,	2.5999999},
{	-10.489829,	-7.5,	119,	14.565539,	-2.5999999},
{	-9.9770317,	-4.6999998,	95,	14.508112,	2.5999999},
{	-13.557318,	-8.5,	126,	14.913401,	-2.5999999},
{	-13.046968,	-6,	92,	14.854958,	2.5999999},
{	-9.4636793,	-1,	112,	14.450804,	-2.5999999},
{	-8.949728,	1.5,	93,	14.3936,	2.5999999},
{	-12.536313,	-2,	121,	14.796721,	-2.5999999},
{	-12.025314,	0.40000001,	96,	14.738676,	2.5999999},
};

#endif






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


    
//    unsigned short rot;
  

    double x, y, z;

    /*
    int circle_col = 0;
    int circle_row = 0;
    int circle_col_other = 0; 
    int circle_row_other = 0;
   */


    for ( c = 0 ; c < CIRCLELENGTH; c++ )
    {

		fseeko(fp , BLOCK_OFFSET, SEEK_CUR);
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
            //rot = ( ( unsigned short ) ( *pshort ) / 100.0 );


            //circle_col =  c * 6 + i / 2;
            rotational = ( ( double ) ( *pshort ) ) / 100.0;

    


            for ( j = 0; j < 32; j++ )
            {
               
                physicalNO  = j + Head;
                //logicalNO = velodyne_physical_to_logical ( physicalNO );

                //circle_col =  c * 6 + i / 2;
                //circle_row =  logicalNO;


                //circle_col_other =  c * 6 + i / 2;  /////????
                //circle_row_other =  logicalNO / 2;

             
                
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
				  && (minDist == -1 || sqr(point.x) + sqr(point.y) + sqr(point.z) > minDist2*1.0)&&(point.y>-180))
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
int ScanIO_velodyne::readScans(int start, int end, string &dir, int maxDist, int minDist,
						  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;
  

  FILE *scan_in = 0;
  FILE *pose_in = 0;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  
  scanFileName = dir + "scan"  + ".bin";
  poseFileName = dir + "scan"  + ".pose";
  
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
  fseeko(scan_in, 24, SEEK_SET);
  fseeko(scan_in, (BLOCK_SIZE+BLOCK_OFFSET)*CIRCLELENGTH*fileCounter, SEEK_CUR);
 #endif
  
#endif
  read_one_packet(scan_in, ptss, maxDist, minDist);
  cout << " with " << ptss.size() << " Points";

  
  cout << " done " << fileCounter<<endl;
  if(pose_in)
  {
      double poseinfo[6];
      fseeko(pose_in, 0, SEEK_SET);
      fseeko(pose_in, sizeof(double)*6*fileCounter, SEEK_CUR);
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
  return new ScanIO_velodyne;
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

