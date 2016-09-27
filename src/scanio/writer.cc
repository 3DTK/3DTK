/*
 * writer implementation
 *
 * Copyright (C) by the 3DTK contributors
 * Copyright (C) Dorit Borrmann, Razvan-George Mihalyi, Remus Dumitru 
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Helper functions for writing scans.
 *
 * @author Dorit Borrmann. Jacobs University Bremen gGmbH, Germany.
 * @author Razvan-George Mihalyi. Jacobs University Bremen gGmbH, Germany.
 * @author Remus Dumitru. Jacobs University Bremen gGmbH, Germany.
 */
#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#define WANT_STREAM ///< define the WANT stream :)
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <errno.h>

#include "scanio/writer.h"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"

#include "slam6d/globals.icc"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <windows.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <strings.h>
#include <dlfcn.h>
#endif

void createdirectory(string dir)
{
  int success = mkdir(dir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);

  if (success == 0 || errno == EEXIST) {
    cout << "Writing to " << dir << endl;
  } else {
    cerr << "Creating directory " << dir << " failed" << endl;
    exit(1);
  }
}


/*
 * given a vector of 3d points, write them out as uos files
 */
void write_uos(vector<cv::Vec4f> &points, string &dir, string id)
{
  ofstream outfile((dir + "/scan" + id + ".3d").c_str());

  outfile << "# header is ignored" << endl;

  for (vector<cv::Vec4f>::iterator it=points.begin(); it < points.end(); it++) {
    outfile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << endl;
  }
  outfile.close();
}

/*
 * given a vector of 3d points, write them out as uosr files
 */
void write_uosr(vector<cv::Vec4f> &points, string &dir, string id)
{
  ofstream outfile((dir + "/scan" + id + ".3d").c_str());
  
  outfile.precision(20);
  outfile << "# header is ignored" << endl;

  for (vector<cv::Vec4f>::iterator it=points.begin(); it < points.end(); it++) {
    if((*it)[0]!=0 && (*it)[1]!=0 && (*it)[2]!=0)
      outfile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << endl;
  }

  outfile.close();
}

/*
 * given a vector of 3d points, write them out as uos_rgb files
 */
void write_uos_rgb(vector<cv::Vec4f> &points, vector<cv::Vec3b> &color, string &dir, string id)
{
  ofstream outfile((dir + "/scan" + id + ".3d").c_str());
  
  outfile.precision(20);
  outfile << "# header is ignored" << endl;

  vector<cv::Vec3b>::iterator cit=color.begin(); 
  for (vector<cv::Vec4f>::iterator it=points.begin();  it < points.end(); it++) {
    if((*it)[0]!=0 && (*it)[1]!=0 && (*it)[2]!=0)
      outfile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " 
      << (int)(*cit)[0] << " " << (int)(*cit)[1] << " " << (int)(*cit)[2] << endl;
      cit++;
  }

  outfile.close();
}

void write_uos(DataXYZ &xyz, FILE *file, bool hexfloat)
{
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a\n",
					xyz[j][0], xyz[j][1], xyz[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			fprintf(file, "%.016e %.016e %.016e\n",
					xyz[j][0], xyz[j][1], xyz[j][2]);
		}
	}
}

void write_uosr(DataXYZ &xyz, DataReflectance &xyz_reflectance, FILE *file, bool hexfloat)
{
	if(xyz.size() != xyz_reflectance.size()) {
		throw std::runtime_error("xyz and reflectance vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %.013a\n",
					xyz[j][0], xyz[j][1], xyz[j][2], xyz_reflectance[j]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			fprintf(file, "%.016e %.016e %.016e %.016e\n",
					xyz[j][0], xyz[j][1], xyz[j][2], xyz_reflectance[j]);
		}
	} 
}

void write_uos_rgb(DataXYZ &xyz, DataRGB &rgb, FILE *file, bool hexfloat)
{
	if(xyz.size() != rgb.size()) {
		throw std::runtime_error("xyz and rgb vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %d %d %d\n",
					xyz[j][0], xyz[j][1], xyz[j][2],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			fprintf(file, "%.016e %.016e %.016e %d %d %d\n",
					xyz[j][0], xyz[j][1], xyz[j][2],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		}
	}
}

void write_xyz(DataXYZ &xyz, FILE *file, double scaleFac, bool hexfloat)
{
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			fprintf(file, "%.016e %.016e %.016e\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1]);
		}
	}
}

void write_xyzr(DataXYZ &xyz, DataReflectance &xyz_reflectance, FILE *file, double scaleFac, bool hexfloat)
{
	if(xyz.size() != xyz_reflectance.size()) {
		throw std::runtime_error("xyz and reflectance vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %.013a\n",
					scaleFac*xyz[j][2],
					-scaleFac*xyz[j][0],
					scaleFac*xyz[j][1],
					xyz_reflectance[j]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			fprintf(file, "%.016e %.016e %.016e %.016e\n",
					scaleFac*xyz[j][2],
					-scaleFac*xyz[j][0],
					scaleFac*xyz[j][1],
					xyz_reflectance[j]);
		}
	}
}

void write_xyz_rgb(DataXYZ &xyz, DataRGB &rgb, FILE *file, double scaleFac, bool hexfloat)
{
	if(xyz.size() != rgb.size()) {
		throw std::runtime_error("xyz and rgb vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %d %d %d\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			fprintf(file, "%.016e %.016e %.016e %d %d %d\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		}
	}
}

// write .pose files
// .frames files can later be generated from them using ./bin/pose2frames
void writeposefile(string &dir, const double* rPos, const double* rPosTheta, string id)
{
  ofstream posefile((dir + "/scan" + id + ".pose").c_str());
  posefile << rPos[0] << " " << rPos[1] << " " << rPos[2] << endl;
  posefile << deg(rPosTheta[0]) << " "
           << deg(rPosTheta[1]) << " "
           << deg(rPosTheta[2]) << endl;
  posefile.close();
}

void writeTrajectoryXYZ(ofstream &posesout, const double * transMat, bool mat, double scaleFac)
{
  if(mat)
  {
    posesout << transMat[10] << " "
     << -transMat[ 2] << " "
     << transMat[ 6] << " "
     << scaleFac*transMat[14] << " "
     << -transMat[ 8] << " "
     << transMat[ 0] << " "
     << -transMat[ 4] << " "
     << scaleFac*-transMat[12] << " "
     << transMat[ 9] << " "
     << -transMat[ 1] << " "
     << transMat[ 5] << " "
     << scaleFac*transMat[13] << " "
     << transMat[11] << " "
     << -transMat[ 3] << " "
     << -transMat[ 7] << " "
     << transMat[15] << " ";
  } else {
    posesout << scaleFac*transMat[14] << " "
    << scaleFac*-transMat[12] << " "
    << scaleFac*transMat[13] << " ";
  }
  posesout << endl;
}

void writeTrajectoryUOS(ofstream &posesout, const double * transMat, bool mat)
{
  if(mat) 
  {
    posesout << transMat[ 0] << " "
     << transMat[ 1] << " "
     << transMat[ 2] << " "
     << transMat[ 3] << " "
     << transMat[ 4] << " "
     << transMat[ 5] << " "
     << transMat[ 6] << " "
     << transMat[ 7] << " "
     << transMat[ 8] << " "
     << transMat[ 9] << " "
     << transMat[10] << " "
     << transMat[11] << " ";
  }
  posesout << transMat[12] << " "
   << transMat[13] << " "
   << transMat[14] << " ";
  if(mat) { 
    posesout << transMat[15] << " ";
  }
  posesout << endl;
}
