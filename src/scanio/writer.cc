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
#include <iostream>
#include <fstream>
#include <errno.h>

#include "scanio/writer.h"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"

#include "slam6d/globals.icc"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#else
#include <dlfcn.h>
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include "XGetopt.h"
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <strings.h>
#include <getopt.h>
#endif


#include "rply.h"

void createdirectory(std::string dir)
{
  int success = mkdir(dir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);

  if (success == 0 || errno == EEXIST) {
    std::cout << "Writing to " << dir << std::endl;
  } else {
    std::cerr << "Creating directory " << dir << " failed" << std::endl;
    exit(1);
  }
}


/*
 * given a vector of 3d points, write them out as uos files
 */
void write_uos(std::vector<cv::Vec4f> &points, std::string &dir, std::string id, bool high_precision, volatile bool *abort_flag)
{
  std::ofstream outfile((dir + "/scan" + id + ".3d").c_str());

  outfile << "# header is ignored" << std::endl;
  if(high_precision) {
    outfile.precision(20);
  } else {
    outfile.precision(10);
  }

  for (std::vector<cv::Vec4f>::iterator it=points.begin(); it < points.end(); it++) {
    if (abort_flag != nullptr && *abort_flag) break;
    outfile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;
  }
  outfile.close();
}

/*
 * given a vector of 3d points, write them out as uosr files
 */
void write_uosr(std::vector<cv::Vec4f> &points, std::string &dir, std::string id, bool high_precision, volatile bool *abort_flag)
{
  std::ofstream outfile((dir + "/scan" + id + ".3d").c_str());
  if(high_precision) {
    outfile.precision(20);
  } else {
    outfile.precision(10);
  }
  outfile << "# header is ignored" << std::endl;

  for (std::vector<cv::Vec4f>::iterator it=points.begin(); it < points.end(); it++) {
    if (abort_flag != nullptr && *abort_flag) break;
    if((*it)[0]!=0 && (*it)[1]!=0 && (*it)[2]!=0)
      outfile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << std::endl;
  }

  outfile.close();
}

/*
 * given a vector of 3d points, write them out as uos_rgb files
 */
void write_uos_rgb(std::vector<cv::Vec4f> &points, std::vector<cv::Vec3b> &color, std::string &dir, std::string id, bool high_precision, volatile bool *abort_flag)
{
  std::ofstream outfile((dir + "/scan" + id + ".3d").c_str());

  if(high_precision) {
    outfile.precision(20);
  } else {
    outfile.precision(10);
  }
  outfile << "# header is ignored" << std::endl;

  std::vector<cv::Vec3b>::iterator cit=color.begin();
  for (std::vector<cv::Vec4f>::iterator it=points.begin();  it < points.end(); it++) {
    if (abort_flag != nullptr && *abort_flag) break;
    if((*it)[0]!=0 && (*it)[1]!=0 && (*it)[2]!=0)
      outfile << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " "
      << (int)(*cit)[0] << " " << (int)(*cit)[1] << " " << (int)(*cit)[2] << std::endl;
    cit++;
  }

  outfile.close();
}

void write_uos(DataXYZ &xyz, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
  if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
        fprintf(file, "%.016e %.016e %.016e\n",
					  scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2]);
      } else {
        fprintf(file, "%lf %lf %lf\n",
					  scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2]);
      }
		}
	}
}

void write_uosr(DataXYZ &xyz, DataReflectance &xyz_reflectance, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
	if(xyz.size() != xyz_reflectance.size()) {
		throw std::runtime_error("xyz and reflectance vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %.013a\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2], xyz_reflectance[j]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
			  fprintf(file, "%.016e %.016e %.016e %.016e\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2], xyz_reflectance[j]);
      } else {
        fprintf(file, "%lf %lf %lf %lf\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2], xyz_reflectance[j]);
      }
		}
	}
}

void write_uos_rgb(DataXYZ &xyz, DataRGB &rgb, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
	if(xyz.size() != rgb.size()) {
		throw std::runtime_error("xyz and rgb vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %d %d %d\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
			  fprintf(file, "%.016e %.016e %.016e %d %d %d\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
      } else {
        fprintf(file, "%lf %lf %lf %d %d %d\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
      }
		}
	}
}

void write_uos_normal(DataXYZ &xyz, DataNormal &normals, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
  if(xyz.size() != normals.size()) {
		throw std::runtime_error("xyz and normal vector are of different length");
	}
  if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %.013a %.013a %.013a\n",
					scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2],
          normals[j][0], normals[j][1], normals[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
        fprintf(file, "%.016e %.016e %.016e %.016e %.016e %.016e\n",
					  scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2],
          normals[j][0], normals[j][1], normals[j][2]);
      } else {
        fprintf(file, "%lf %lf %lf %lf %lf %lf\n",
					  scaleFac*xyz[j][0], scaleFac*xyz[j][1], scaleFac*xyz[j][2],
            normals[j][0], normals[j][1], normals[j][2]);
      }
		}
	}
}

void write_xyz(DataXYZ &xyz, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
			  fprintf(file, "%.016e %.016e %.016e\n",
					  scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1]);
      } else {
        fprintf(file, "%lf %lf %lf\n",
					  scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1]);
      }
		}
	}
}

void write_xyzr(DataXYZ &xyz, DataReflectance &xyz_reflectance, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
	if(xyz.size() != xyz_reflectance.size()) {
		throw std::runtime_error("xyz and reflectance vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
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
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
			  fprintf(file, "%.016e %.016e %.016e %.016e\n",
					  scaleFac*xyz[j][2],
					  -scaleFac*xyz[j][0],
					  scaleFac*xyz[j][1],
					  xyz_reflectance[j]);
                        } else {
                          fprintf(file, "%lf %lf %lf %lf\n",
					  scaleFac*xyz[j][2],
					  -scaleFac*xyz[j][0],
					  scaleFac*xyz[j][1],
					  xyz_reflectance[j]);
      }
		}
	}
}

void write_xyz_rgb(DataXYZ &xyz, DataRGB &rgb, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
	if(xyz.size() != rgb.size()) {
		throw std::runtime_error("xyz and rgb vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %d %d %d\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					(int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
        fprintf(file, "%.016e %.016e %.016e %d %d %d\n",
					  scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					  (int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
		  } else {
        fprintf(file, "%lf %lf %lf %d %d %d\n",
					  scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					  (int)rgb[j][0], (int)rgb[j][1], (int)rgb[j][2]);
      }
    }
	}
}

void write_xyz_normal(DataXYZ &xyz, DataNormal &normals, FILE *file, double scaleFac, bool hexfloat, bool high_precision, volatile bool *abort_flag)
{
	if(xyz.size() != normals.size()) {
		throw std::runtime_error("xyz and normal vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// we print the mantissa with 13 hexadecimal digits because the
			// mantissa for double precision is 52 bits long which is 6.5
			// bytes and thus 13 hexadecimal digits
			fprintf(file, "%.013a %.013a %.013a %.013a %.013a %.013a\n",
					scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					normals[j][2], -normals[j][0], normals[j][1]);
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if (abort_flag != nullptr && *abort_flag) break;
			// 17 significant digits are required to encode a double
			// precision IEEE754 floating point number. Proof: the
			// number of significant digits of the epsilon between 1.0
			// and the next representable value has 16 significant digits.
			// Adding that epsilon to 1.0 leads to a number with 17
			// significant digits.
			// We use %e because it's the only format that allows to set
			// the overall significant digits (and not just the digits
			// after the radix character).
			if(high_precision) {
        fprintf(file, "%.016e %.016e %.016e %.016e %.016e %.016e\n",
					  scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					  normals[j][2], -normals[j][0], normals[j][1]);
		  } else {
        fprintf(file, "%lf %lf %lf %lf %lf %lf\n",
					  scaleFac*xyz[j][2], -scaleFac*xyz[j][0], scaleFac*xyz[j][1],
					  normals[j][2], -normals[j][0], normals[j][1]);
      }
    }
	}
}

void write_ply_rgb(std::vector<cv::Vec4f> &points, std::vector<cv::Vec3b> &color, std::string &dir, std::string id)
{
	p_ply ply = ply_create((dir + "/scan" + id + ".ply").c_str(), PLY_LITTLE_ENDIAN, NULL, 0, NULL);
	if (!ply) {
		throw std::runtime_error("ply_open failed");
	}

	if (!ply_add_element(ply, "vertex", points.size())) {
		throw std::runtime_error("ply_add_element failed");
	}

	if (!ply_add_property(ply, "x", PLY_FLOAT, (e_ply_type)0, (e_ply_type)0)) {
		throw std::runtime_error("ply_add_property failed");
	}
	if (!ply_add_property(ply, "y", PLY_FLOAT, (e_ply_type)0, (e_ply_type)0)) {
		throw std::runtime_error("ply_add_property failed");
	}
	if (!ply_add_property(ply, "z", PLY_FLOAT, (e_ply_type)0, (e_ply_type)0)) {
		throw std::runtime_error("ply_add_property failed");
	}
	if (!ply_add_property(ply, "red", PLY_UCHAR, (e_ply_type)0, (e_ply_type)0)) {
		throw std::runtime_error("ply_add_property failed");
	}
	if (!ply_add_property(ply, "green", PLY_UCHAR, (e_ply_type)0, (e_ply_type)0)) {
		throw std::runtime_error("ply_add_property failed");
	}
	if (!ply_add_property(ply, "blue", PLY_UCHAR, (e_ply_type)0, (e_ply_type)0)) {
		throw std::runtime_error("ply_add_property failed");
	}

	if (!ply_write_header(ply)) {
		throw std::runtime_error("ply_write_header failed");
	}

	std::vector<cv::Vec3b>::iterator cit=color.begin();
	for (std::vector<cv::Vec4f>::iterator it=points.begin(); it < points.end(); it++) {
		ply_write(ply, (*it)[0]);
		ply_write(ply, (*it)[1]);
		ply_write(ply, (*it)[2]);
		ply_write(ply, (*cit)[0]);
		ply_write(ply, (*cit)[1]);
		ply_write(ply, (*cit)[2]);
		cit++;
	}

	if (!ply_close(ply)) {
		throw std::runtime_error("ply_close failed");
	}
}

// write .pose files
// .frames files can later be generated from them using ./bin/pose2frames
void writeposefile(std::string &dir, const double* rPos, const double* rPosTheta, std::string id)
{
  std::ofstream posefile((dir + "/scan" + id + ".pose").c_str());
  posefile << rPos[0] << " " << rPos[1] << " " << rPos[2] << std::endl;
  posefile << deg(rPosTheta[0]) << " "
           << deg(rPosTheta[1]) << " "
           << deg(rPosTheta[2]) << std::endl;
  posefile.close();
}

void writeTrajectoryXYZ(std::ofstream &posesout, const double * transMat, bool mat, double scaleFac)
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
  posesout << std::endl;
}

void writeTrajectoryUOS(std::ofstream &posesout, const double * transMat, bool mat, double scaleFac)
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
  posesout << scaleFac*transMat[12] << " "
   << scaleFac*transMat[13] << " "
   << scaleFac*transMat[14] << " ";
  if(mat) {
    posesout << transMat[15] << " ";
  }
  posesout << std::endl;
}
