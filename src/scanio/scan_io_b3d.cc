/*
 * scan_io_b3d implementation
 *
 *
 *
 *
 */


/**
 * @file
 * @brief Implementation of reading 3D scans in B3D format
 * @author Peter Janotta, Measurement in Motion GmbH, Theilheim, Germany.
 */

#include "scanio/scan_io_b3d.h"
#include "scanio/helper.h"
#include "b3dpsreader.h"
#include "slam6d/point.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
#include <string>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"

#define DATA_PATH_PREFIX ""
#define DATA_PATH_SUFFIX ".b3d"
#define POSE_PATH_PREFIX ""
#define POSE_PATH_SUFFIX ".bps"

B3DPSReader* ScanIO_b3d::inB3DPS = NULL;

inline bool ScanIO_b3d::readEOL(std::ifstream &f) {
    uint16_t buf;
    f.read((char*)&buf, 2);
    return (buf == 0xFFFF);
}

std::list<std::string> ScanIO_b3d::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
	std::list<std::string> identifiers;
  std::string dir_str(dir_path);
  if (dir_str.back() == '/' || dir_str.back() == '\\') dir_str.pop_back();
  int filename_begin = dir_str.find_last_of("/\\");
  if (filename_begin == std::string::npos) filename_begin = 0;
  std::string data_filename = dir_str.substr(0, dir_str.find_first_of('.', filename_begin)) + DATA_PATH_SUFFIX;
  std::string pose_filename = dir_str + POSE_PATH_SUFFIX;
  if (inB3DPS == NULL) inB3DPS = new B3DPSReader(data_filename, pose_filename);
	return inB3DPS->readDirectoryHelper(start, end);
}


void ScanIO_b3d::readPose(const char* dir_path, const char* identifier, double* pose)
{
  readPose(dir_path, identifier, pose, NULL);
}



void ScanIO_b3d::readPose(const char* dir_path, const char* identifier, double* pose, double* timestamp)
{
	//if (inB3DPS == NULL) inB3DPS = new B3DPSReader(std::string(dir_path) + DATA_PATH_SUFFIX, std::string(dir_path) + POSE_PATH_SUFFIX);
	double* poseRef = inB3DPS->getPose(strtoull(identifier, NULL, 10));
	
  double P[16];
  double PS[16];
  double Sinv[16];
  double transMatOrg[16];
  double S[16] =
  { 0, 0, 100, 0,
    -100, 0, 0, 0,
    0, 100, 0, 0,
    0, 0, 0, 1 };
  M4inv(S, Sinv);
  *timestamp = poseRef[0];
  QuatToMatrix4(poseRef + 4, poseRef + 1, P);
  MMult(P, Sinv, PS);
  MMult(S, PS, transMatOrg);
  Matrix4ToEuler(transMatOrg, pose + 3, pose);
}

time_t ScanIO_b3d::lastModified(const char* dir_path, const char* identifier)
{
  const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
  return 0;//lastModifiedHelper(dir_path, identifier, suffixes);
}

bool ScanIO_b3d::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_REFLECTANCE));
}

void ScanIO_b3d::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
	if (xyz == 0 || reflectance == 0)
		return;
	//if (inB3DPS == NULL) inB3DPS = new B3DPSReader(std::string(dir_path) + DATA_PATH_SUFFIX, std::string(dir_path) + POSE_PATH_SUFFIX);
	uint64_t linenr = strtoull(identifier, nullptr, 10);
	inB3DPS->getScan(linenr, xyz, reflectance, filter);
}

ScanIO_b3d::~ScanIO_b3d() {
	if (inB3DPS != NULL) {
		delete inB3DPS;
		inB3DPS = NULL;
	}
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
  return new ScanIO_b3d;
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
