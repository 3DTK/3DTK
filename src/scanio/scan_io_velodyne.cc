/*
 * scan_io_velodyne implementation
 *
 * Copyright (C) Mohammad Faisal, Dorit Borrmann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH
 * @author Dorit Borrmann. Smart Systems Group, Jacobs University Bremen gGmbH, Germany.
 * @author Mohammad Faisal. Smart Systems Group, Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/point.h"
#include "veloslam/velodefs.h"
#include "scanio/scan_io_velodyne.h"
#include "slam6d/globals.icc"
#include <fstream>
#include<strstream>
#include <iostream>
using std::ifstream;
using std::ostrstream;
using std::cerr;
using std::endl;
using std::ends;

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
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#define DATA_PATH_PREFIX "scan"
#define DATA_PATH_SUFFIX ".bin"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".pose"

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

long CountOfLidar = 0;

//Changes the variable to be filled dynamically. Added a 6th variable to enable or disable a specific sensor
double velodyne_calibrated[VELODYNE_NUM_LASERS][6];

/**
 * Method to fill the calibration data using the Velodyne64 calibration data from Wuhan.
 */
int backup()
{
	double velodyne_calibrated1[64][6] = 
	{
		{	-7.1581192,	-4.5,	102,	21.560343,	-2.5999999, 1},
		{	-6.8178215,	-3.4000001,	125,	21.516994,	2.5999999, 1},
		{	0.31782165,	3,	130,	20.617426,	-2.5999999, 1},
		{	0.65811908,	4.5999999,	128,	20.574717,	2.5999999, 1},
		{	-6.4776502,	-0.5,	112,	21.473722,	-2.5999999, 1},
		{	-6.1375928,	1,	125,	21.430525,	2.5999999, 1},
		{	-8.520812,	-1.5,	106,	21.734608,	-2.5999999, 1},
		{	-8.1798887,	0.40000001,	127,	21.690901,	2.5999999, 1},
		{	-5.797637,	4,	111,	21.387396,	-2.5999999, 1},
		{	-5.4577708,	5.5,	126,	21.34433,	2.5999999, 1},
		{	-7.8391404,	3.0999999,	113,	21.647291,	-2.5999999, 1},
		{	-7.4985547,	4.5,	123,	21.603773,	2.5999999, 1},
		{	-3.0802133,	-4.5,	105,	21.044245,	-2.5999999, 1},
		{	-2.7406337,	-3.2,	133,	21.001518,	2.5999999, 1},
		{	-5.1179824,	-5.5,	110,	21.301321,	-2.5999999, 1},
		{	-4.7782598,	-4,	129,	21.258366,	2.5999999, 1},
		{	-2.4010365,	-0.2,	111,	20.958813,	-2.5999999, 1},
		{	-2.0614092,	1,	130,	20.916126,	2.5999999, 1},
		{	-4.4385905,	-1.2,	115,	21.215462,	-2.5999999, 1},
		{	-4.0989642,	0,	133,	21.172602,	2.5999999, 1},
		{	-1.7217404,	3.8,	113,	20.873451,	-2.5999999, 1},
		{	-1.3820176,	5,	130,	20.830786,	2.5999999, 1},
		{	-3.7593663,	3,	117,	21.129782,	-2.5999999, 1},
		{	-3.4197867,	4.5,	129,	21.086998,	2.5999999, 1},
		{	0.998555,	-4.5,	107,	20.531982,	-2.5999999, 1},
		{	1.339141,	-3.2,	131,	20.489222,	2.5999999, 1},
		{	-1.0422293,	-5.4000001,	128,	20.788124,	-2.5999999, 1},
		{	-0.70236301,	-4,	134,	20.745461,	2.5999999, 1},
		{	1.679889,	-0.5,	124,	20.446428,	-2.5999999, 1},
		{	2.0208123,	1,	136,	20.403601,	2.5999999, 1},
		{	-0.36240739,	-1.5,	131,	20.702793,	-2.5999999, 1},
		{	-0.022349782,	0.2,	136,	20.660116,	2.5999999, 1},
		{	-22.737886,	-7.8000002,	101,	16.019152,	-2.5999999, 1},
		{	-22.226072,	-5,	88,	15.954137,	2.5999999, 1},
		{	-11.513928,	4.5,	121,	14.680806,	-2.5999999, 1},
		{	-11.002114,	7.4000001,	88,	14.623099,	2.5999999, 1},
		{	-21.714685,	-1,	94,	15.889649,	-2.5999999, 1},
		{	-21.203688,	2,	88,	15.82566,	2.5999999, 1},
		{	-24.790272,	-2.5,	114,	16.284933,	-2.5999999, 1},
		{	-24.276321,	0.5,	89,	16.217583,	2.5999999, 1},
		{	-20.693031,	6,	98,	15.762167,	-2.5999999, 1},
		{	-20.182682,	9,	92,	15.699132,	2.5999999, 1},
		{	-23.762968,	4.5,	107,	16.15085,	-2.5999999, 1},
		{	-23.250172,	7.5,	80,	16.084715,	2.5999999, 1},
		{	-16.615318,	-7.5,	121,	15.26925,	-2.5999999, 1},
		{	-16.105938,	-5,	92,	15.209245,	2.5999999, 1},
		{	-19.672594,	-9,	119,	15.63654,	-2.5999999, 1},
		{	-19.162729,	-6,	89,	15.574372,	2.5999999, 1},
		{	-15.596496,	-1,	109,	15.14954,	-2.5999999, 1},
		{	-15.086954,	2,	88,	15.090119,	2.5999999, 1},
		{	-18.653046,	-2,	117,	15.51261,	-2.5999999, 1},
		{	-18.143503,	0.69999999,	88,	15.451235,	2.5999999, 1},
		{	-14.577271,	5.5,	112,	15.030966,	-2.5999999, 1},
		{	-14.067405,	8.3999996,	87,	14.972065,	2.5999999, 1},
		{	-17.634062,	5,	119,	15.390228,	-6.1999998, 1},
		{	-17.124681,	7.5,	97,	15.329572,	2.5999999, 1},
		{	-10.489829,	-7.5,	119,	14.565539,	-2.5999999, 1},
		{	-9.9770317,	-4.6999998,	95,	14.508112,	2.5999999, 1},
		{	-13.557318,	-8.5,	126,	14.913401,	-2.5999999, 1},
		{	-13.046968,	-6,	92,	14.854958,	2.5999999, 1},
		{	-9.4636793,	-1,	112,	14.450804,	-2.5999999, 1},
		{	-8.949728,	1.5,	93,	14.3936,	2.5999999, 1},
		{	-12.536313,	-2,	121,	14.796721,	-2.5999999, 1},
		{	-12.025314,	0.40000001,	96,	14.738676,	2.5999999, 1},
	};

	for ( int i = 0; i < VELODYNE_NUM_LASERS; i++ )
	{
	velodyne_calibrated[i][0] = velodyne_calibrated1[i][0];
	velodyne_calibrated[i][1] = velodyne_calibrated1[i][1];
	velodyne_calibrated[i][2] = velodyne_calibrated1[i][2];
	velodyne_calibrated[i][3] = velodyne_calibrated1[i][3];
	velodyne_calibrated[i][4] = velodyne_calibrated1[i][4];
	velodyne_calibrated[i][5] = velodyne_calibrated1[i][5];
	}

	return 0;
}

double rotCorrection[VELODYNE_NUM_LASERS];
double vertCorrection[VELODYNE_NUM_LASERS];
double distCorrection[VELODYNE_NUM_LASERS];
double vertoffsetCorrection[VELODYNE_NUM_LASERS];
double horizdffsetCorrection[VELODYNE_NUM_LASERS];
double enabled[VELODYNE_NUM_LASERS];	//New variable to change enabling and disabling of data.

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

    //vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection  enable
    for ( i = 0; i < VELODYNE_NUM_LASERS; i++ )
    {
        vertCorrection[i] = velodyne_calibrated[i][0] * RADIANS_PER_LSB;
        rotCorrection[i] =  velodyne_calibrated[i][1] * RADIANS_PER_LSB;
        distCorrection[i] = velodyne_calibrated[i][2] * METERS_PER_CM;
        vertoffsetCorrection[i] = velodyne_calibrated[i][3] * METERS_PER_CM;
        horizdffsetCorrection[i] = velodyne_calibrated[i][4] * METERS_PER_CM;
	enabled[i] = velodyne_calibrated[i][5];
    }

    return 0;
}


int calibrate(string filename)
{
	int linecount = 0;
	string line;
	std::ifstream myfile (filename.c_str());
	if (myfile.is_open())
	{	
		cout << "Using Calibration File"<<endl;
		string data[6];
		getline (myfile,line);
		string theDelimiter = ",";
		int j=0;
		do
		{
			getline (myfile,line);
			int i = 0;
			size_t  start = 0, end = 0;
			while ( end != string::npos  && i < 6)
			{
				end = line.find( theDelimiter, start);
				data[i] = line.substr(start, (end == string::npos)?string::npos:end - start);
				start = ((end>(string::npos - theDelimiter.size()))?string::npos:end + theDelimiter.size());
				i++;
			}

			velodyne_calibrated[j][0] = atof(data[0].c_str());
			velodyne_calibrated[j][1] = atof(data[1].c_str());
			velodyne_calibrated[j][2] = atof(data[2].c_str());
			velodyne_calibrated[j][3] = atof(data[3].c_str());
			velodyne_calibrated[j][4] = atof(data[4].c_str());
			velodyne_calibrated[j][5] = atof(data[5].c_str());
			linecount++;
			j++;
		}
		while(!myfile.eof());
		
		myfile.close();

		if (linecount < 60)
		{
			for (int i = 32; i<64; i++)
			{
				velodyne_calibrated[i][0] = 0.0;
				velodyne_calibrated[i][1] = 0.0;
				velodyne_calibrated[i][2] = 0.0;
				velodyne_calibrated[i][3] = 0.0;
				velodyne_calibrated[i][4] = 0.0;
				velodyne_calibrated[i][5] = 0.0;
			}
		}
	}
	else
	{
		cout << "Unable to open calibration file.\nUsing Wuhan hardcored values."<<endl;
		backup();
	}

	return 0;
}


int read_one_packet (
    FILE *fp,
    PointFilter& filter,
    std::vector<double>* xyz,
    std::vector<unsigned char>* rgb,
    std::vector<float>* reflectance,
    std::vector<float>* amplitude,
    std::vector<int>* type,
    std::vector<float>* deviation)
{

	int  c, i, j;
    unsigned char Head;
    BYTE buf[BLOCK_SIZE];
    Point point;
    BYTE *p;
    unsigned short *ps;
    unsigned short *pt;
    unsigned short *pshort;

    double ctheta;
    double theta, phi;

    double sin_ctheta, cos_ctheta;
    double sin_theta, cos_theta;
    double sin_phi, cos_phi;

    unsigned short physicalNO;
    unsigned short logicalNO;

    float rotational;
    float distance;
    float corredistance;
    int intensity;
    int physical;
    int size;

    unsigned short rot;
    double x, y, z;

    int circle_col = 0;
    int circle_row = 0;

	for ( c = 0 ; c < CIRCLELENGTH; c++ )
	{
#ifdef _MSC_VER 
		fseek(fp , BLOCK_OFFSET, SEEK_CUR);
#else
#ifdef __CYGWIN__
		fseek(fp , BLOCK_OFFSET, SEEK_CUR);
#else		
		fseeko(fp , BLOCK_OFFSET, SEEK_CUR);
#endif
#endif		
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
			rot = ( ( unsigned short ) ( *pshort ) / 100.0 );

			circle_col =  c * 6 + i / 2;
			rotational = ( ( float ) ( *pshort ) ) / 100.0;

			for ( j = 0; j < 32; j++ )
			{
				physicalNO  = j + Head;
				logicalNO = velodyne_physical_to_logical ( physicalNO );

				circle_col =  c * 6 + i / 2;
				circle_row =  logicalNO;

				pt = ( unsigned short * ) ( p + 4 + j * 3 );
				distance = absf ( ( *pt ) * 0.002 );

				if( distance < 120   &&	   distance > 2.2 )
				{
					BYTE *inty = ( BYTE * ) ( p + 4 + j * 3 + 2 );
					intensity = *inty;

					ctheta = 2 * M_PI - rotational * RADIANS_PER_LSB;
					if ( ctheta == 2*M_PI )
						ctheta = 0;

					sin_ctheta = sin ( ctheta );
					cos_ctheta = cos ( ctheta );

					//vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
					//                corredistance = ( distance + distCorrection[physicalNO] ) * ( 1.0 + vertoffsetCorrection[physicalNO] );
					corredistance = ( distance + distCorrection[physicalNO] ) ;
					theta     = mod2pi_ref ( M_PI, ctheta + rotCorrection[physicalNO] ); /////////????////////
					phi    = vertCorrection[physicalNO];  ////////?????/////////////////

					sin_theta = sin ( theta );
					cos_theta = cos ( theta );
					sin_phi = sin ( phi );
					cos_phi = cos ( phi );

					/////////////////////?a??¡Á?¡À¨º/////////////////////
					x = corredistance * cos_theta * cos_phi;
					y = corredistance * sin_theta * cos_phi;
					z = corredistance * sin_phi +vertoffsetCorrection[physicalNO]*cos_phi;

					x -= horizdffsetCorrection[physicalNO] * cos_ctheta;
					y -= horizdffsetCorrection[physicalNO] * sin_ctheta;

					point.rad  = sqrt(  x*x 	+  y*y );
					point.tan_theta  = y/x;

					point.type = POINT_TYPE_GROUND;
					point.x=x*100;
					point.y=z*100;
					point.z=-y*100;

				        double p[3];
				        p[0] = point.x;
				        p[1] = point.y;
				        p[2] = point.z;

					//Enable Check. Could be moved earlier to remove all the unnecessary calculations.
					//Here to simplify the implementation
			                if(filter.check(p) && (enabled[physicalNO]==1))
   					{
						 for(int ii = 0; ii < 3; ++ii) xyz->push_back(p[ii]);
					}
				}
			}
            p = p + 100;
            ps = ( unsigned short * ) p;
        }


    }

    return 0;
}

std::list<std::string> ScanIO_velodyne::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
	//Calling the calibration method with the file name "calibration.txt" in the same folder as the scan.bin
	string califilename;
	string dir(dir_path);
	califilename = dir + "calibration" + ".txt";
	calibrate(califilename);
   std::list<std::string> identifiers;
//   fileCounter = start;

   velodyne_calib_precompute();
   for(unsigned int i = start; i <= end; ++i)
   {
	   std::string identifier(to_string(i,3));

	   path data(dir_path);
	   data /= path(std::string(DATA_PATH_PREFIX) + DATA_PATH_SUFFIX);

	   path pose(dir_path);
	   pose /= path(std::string(POSE_PATH_PREFIX) + identifier + POSE_PATH_SUFFIX);

	   if(!exists(data))
		   break;
	   identifiers.push_back(identifier);
   }

  return identifiers;
}

void ScanIO_velodyne::readPose(const char* dir_path, const char* identifier, double* pose)
{
  unsigned int i;
  // on pose information for veloslam
  for(i = 0; i < 6; ++i)  pose[i] = 0.0;
  for(i = 3; i < 6; ++i)  pose[i] = 0.0;
   return;
}

bool ScanIO_velodyne::supports(IODataType type)
{
  return !!(type & (DATA_XYZ));
}


void ScanIO_velodyne::readScan(
    const char* dir_path,
    const char* identifier,
    PointFilter& filter,
    std::vector<double>* xyz,
    std::vector<unsigned char>* rgb,
    std::vector<float>* reflectance,
    std::vector<float>* temperature,
    std::vector<float>* amplitude,
    std::vector<int>* type,
    std::vector<float>* deviation)
{
    unsigned int i;
    FILE *scan_in;

    path data_path(dir_path);
    data_path /= path(std::string(DATA_PATH_PREFIX) +  DATA_PATH_SUFFIX);
    if(!exists(data_path))
        throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");

	char filename[256];
	sprintf(filename, "%s%s%s",dir_path ,DATA_PATH_PREFIX,  DATA_PATH_SUFFIX );

#ifdef _MSC_VER 
    scan_in = fopen(filename,"rb");
#else
#ifdef __CYGWIN__
    scan_in = fopen(filename,"rb");
#else    
    scan_in = fopen64(filename,"rb");
#endif
#endif

    if(scan_in == NULL)
    {
      cerr<<data_path <<endl;
      cerr << "ERROR: Missing file " << data_path <<" "<<strerror(errno)<< endl;
      exit(1);
      return;
    }

    cout << "Processing Scan " << data_path;
    cout.flush();

    xyz->reserve(12*32*CIRCLELENGTH);

	fileCounter=  atoi(identifier);

#ifdef _MSC_VER 
    fseek(scan_in, 24, SEEK_SET);
    fseek(scan_in, (BLOCK_SIZE+BLOCK_OFFSET)*CIRCLELENGTH*fileCounter, SEEK_CUR);
#else
#ifdef __CYGWIN__     
    fseek(scan_in, 24, SEEK_SET);
    fseek(scan_in, (BLOCK_SIZE+BLOCK_OFFSET)*CIRCLELENGTH*fileCounter, SEEK_CUR);
#else
    fseeko(scan_in, 24, SEEK_SET);
    fseeko(scan_in, (BLOCK_SIZE+BLOCK_OFFSET)*CIRCLELENGTH*fileCounter, SEEK_CUR);
#endif    
#endif

    read_one_packet(
                scan_in,
                filter,
                xyz,
                rgb,
                reflectance,
                amplitude,
                type,
                deviation);

    cout << " with " << xyz->size() << " Points";
    cout << " done " << fileCounter<<endl;

    fclose(scan_in);
    // process next frames
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

