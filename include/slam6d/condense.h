#ifndef _CONDENSE_H
#define _CONDENSE_H

#include <sys/types.h>
#include <sys/stat.h>

#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
using std::string;

#include <vector>
#include <map>
#include "slam6d/point.h"
#include "slam6d/scan.h"
#include "scanio/writer.h"
#include "scanio/framesreader.h"
#include "slam6d/basicScan.h"
#include "slam6d/globals.icc"

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
namespace po = boost::program_options;

// checks if the given path exists
int existsDir(const char* path)
{
    struct stat info;
    if (stat( path, &info ) != 0) return 0;
    else if ( info.st_mode & S_IFDIR ) return 1;
    else return 0;
}

inline void transform(double *point, const double *alignxf)
{
  double x_neu, y_neu, z_neu;
  x_neu = point[0] * alignxf[0] + point[1] * alignxf[4] + point[2] * alignxf[8];
  y_neu = point[0] * alignxf[1] + point[1] * alignxf[5] + point[2] * alignxf[9];
  z_neu = point[0] * alignxf[2] + point[1] * alignxf[6] + point[2] * alignxf[10];
  point[0] = x_neu + alignxf[12];
  point[1] = y_neu + alignxf[13];
  point[2] = z_neu + alignxf[14];
}

// write pose file
void writeUOSPose(std::ofstream &posefile, const double* rPos, const double* rPosTheta, double scale=1)
{
  posefile << scale*rPos[0] << " " << scale*rPos[1] << " " << scale*rPos[2] << " ";
  posefile << deg(rPosTheta[0]) << " "
           << deg(rPosTheta[1]) << " "
           << deg(rPosTheta[2]) << std::endl;
}

void writeXYZPose(std::ofstream &posefile, const double* rPos, const double* rPosTheta, double scale=1)
{
    posefile << scale*rPos[2] << " " << -scale*rPos[0] << " " << scale*rPos[1] << " ";
  posefile << deg(-rPosTheta[2]) << " "
           << deg(rPosTheta[0]) << " "
           << deg(-rPosTheta[1]) << std::endl;
}

void writeUOSPoints(std::ofstream &scanfile, DataXYZ &xyz, double scaleFac = 1.0)
{
    for (uint j = 0; j < xyz.size(); j++)
        scanfile << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << endl;
}

void writeUOSPointsReflectance(std::ofstream &scanfile, DataXYZ &xyz, DataReflectance &r, double scaleFac = 1.0)
{
    for (uint j = 0; j < xyz.size(); j++)
        scanfile << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " " << r[j] << endl;
}

void writeUOSC(std::ofstream &scanfile, DataXYZ &xyz ,DataType &xyz_type, double scaleFac = 1.0 , bool hexfloat = false, bool high_precision = false)
{
    if(xyz.size() != xyz_type.size()) {
		throw std::runtime_error("xyz and reflectance vector are of different length");
	}
    if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			scanfile << setprecision(13) << scaleFac*xyz[j][0] << " "
					<< scaleFac*xyz[j][1] << " "
					<< scaleFac*xyz[j][2] << " "
					<< xyz_type[j] << endl;
		}
    } else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if(high_precision) {
			  scanfile << setprecision(17) << scaleFac*xyz[j][0] << " "
					<< scaleFac*xyz[j][1] << " "
					<< scaleFac*xyz[j][2] << " "
					<< xyz_type[j] << endl;
            } else {
                scanfile << setprecision(0) << scaleFac*xyz[j][0] << " "
					<< scaleFac*xyz[j][1] << " "
					<< scaleFac*xyz[j][2] << " "
					<< xyz_type[j] << endl;
            }
		}
	}
}

void writeUOSNormal(std::ofstream &scanfile, DataXYZ &xyz ,DataNormal &normals, double scaleFac = 1.0 , bool hexfloat = false, bool high_precision = false)
{
    if(xyz.size() != normals.size()) {
		throw std::runtime_error("xyz and normal vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
            scanfile << setprecision(13)
                << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " "
				<<	normals[j][0] << " " << normals[j][1] << " " << normals[j][2] << endl;
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if(high_precision) {
                scanfile << setprecision(17)
                        << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " "
                        <<	normals[j][0] << " " << normals[j][1] << " " << normals[j][2] << endl;
		  } else {
                scanfile << setprecision(0)
                        << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " "
                        <<	normals[j][0] << " " << normals[j][1] << " " << normals[j][2] << endl;
            }
        }
	}
}

void writeUOSRGB(std::ofstream &scanfile, DataXYZ &xyz ,DataRGB &rgb, double scaleFac = 1.0 , bool hexfloat = false, bool high_precision = false)
{
    if(xyz.size() != rgb.size()) {
		throw std::runtime_error("xyz and rgb vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
            scanfile << setprecision(13)
                << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " "
				<<	(int)rgb[j][0] << " " << (int)rgb[j][1] << " " << (int)rgb[j][2] << endl;
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if(high_precision) {
                scanfile << setprecision(17)
                        << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " "
                        <<	(int)rgb[j][0] << " " << (int)rgb[j][1] << " " << (int)rgb[j][2] << endl;
		    } else {
                scanfile << setprecision(0)
                        << scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << scaleFac*xyz[j][2] << " "
                        <<	(int)rgb[j][0] << " " << (int)rgb[j][1] << " " << (int)rgb[j][2] << endl;
            }
        }
    }
}

void writeXYZPoints(std::ofstream &scanfile, DataXYZ &xyz, double scaleFac = 1.0)
{
    for (uint j = 0; j < xyz.size(); j++)
        scanfile << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << endl;
}

void writeXYZPointsReflectance(std::ofstream &scanfile, DataXYZ &xyz, DataReflectance &r, double scaleFac = 1.0)
{
    for (uint j = 0; j < xyz.size(); j++)
        scanfile << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " " << r[j] << endl;
}

void writeXYZNormal(std::ofstream &scanfile, DataXYZ &xyz ,DataNormal &normals, double scaleFac = 1.0 , bool hexfloat = false, bool high_precision = false)
{
    if(xyz.size() != normals.size()) {
		throw std::runtime_error("xyz and normal vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
            scanfile << setprecision(13)
                << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " "
				<<	normals[j][2] << " " << -normals[j][0] << " " << normals[j][1] << endl;
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if(high_precision) {
                scanfile << setprecision(17)
                        << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " "
                        <<	normals[j][2] << " " << -normals[j][0] << " " << normals[j][1] << endl;
		  } else {
                scanfile << setprecision(0)
                        << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " "
                        <<	normals[j][2] << " " << -normals[j][0] << " " << normals[j][1] << endl;
            }
        }
	}
}

void writeXYZC(std::ofstream &scanfile, DataXYZ &xyz ,DataType &xyz_type, double scaleFac = 1.0 , bool hexfloat = false, bool high_precision = false)
{
    if(xyz.size() != xyz_type.size()) {
		throw std::runtime_error("xyz and reflectance vector are of different length");
	}
    if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			scanfile << setprecision(13) << scaleFac*xyz[j][2] << " "
					<< -scaleFac*xyz[j][0] << " "
					<< scaleFac*xyz[j][1] << " "
					<< xyz_type[j] << endl;
		}
    } else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if(high_precision) {
			  scanfile << setprecision(17) << scaleFac*xyz[j][2] << " "
					<< -scaleFac*xyz[j][0] << " "
					<< scaleFac*xyz[j][1] << " "
					<< xyz_type[j] << endl;
            } else {
                scanfile << setprecision(0) << scaleFac*xyz[j][2] << " "
					<< -scaleFac*xyz[j][0] << " "
					<< scaleFac*xyz[j][1] << " "
					<< xyz_type[j] << endl;
            }
		}
	}
}

void writeXYZRGB(std::ofstream &scanfile, DataXYZ &xyz ,DataRGB &rgb, double scaleFac = 1.0 , bool hexfloat = false, bool high_precision = false)
{
    if(xyz.size() != rgb.size()) {
		throw std::runtime_error("xyz and rgb vector are of different length");
	}
	if (hexfloat) {
		for(unsigned int j = 0; j < xyz.size(); j++) {
            scanfile << setprecision(13)
                << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " "
				<<	(int)rgb[j][0] << " " << (int)rgb[j][1] << " " << (int)rgb[j][2] << endl;
		}
	} else {
		for(unsigned int j = 0; j < xyz.size(); j++) {
			if(high_precision) {
                scanfile << setprecision(17)
                        << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " "
                        <<	(int)rgb[j][0] << " " << (int)rgb[j][1] << " " << (int)rgb[j][2] << endl;
		    } else {
                scanfile << setprecision(0)
                        << scaleFac*xyz[j][2] << " " << -scaleFac*xyz[j][0] << " " << scaleFac*xyz[j][1] << " "
                        <<	(int)rgb[j][0] << " " << (int)rgb[j][1] << " " << (int)rgb[j][2] << endl;
            }
        }
    }
}

void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}


int parse_options(int argc, char **argv, std::string &dir, double &red, int &rand,
            int &start, int &end, int &maxDist, int &minDist, bool &use_frames,
            bool &use_xyz, bool &use_reflectance, bool &use_type, bool &use_color, int &octree, IOType &type, std::string& customFilter, double &scaleFac,
	    bool &hexfloat, bool &high_precision, int &frame, bool &use_normals, int &split, bool& global)
{
po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  po::options_description input("Input options");
  input.add_options()
    ("format,f", po::value<IOType>(&type)->default_value(UOS, "uos"),
     "using shared library <arg> for input. (chose F from {uos, uos_map, "
     "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
     "riegl_txt, riegl_rgb, riegl_bin, zahn, ply, las})")
    ("start,s", po::value<int>(&start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) "
     "[ATTENTION: counting naturally starts with 0]")
    ("end,e", po::value<int>(&end)->default_value(-1),
     "end after scan <arg>")
    ("split,S", po::value<int>(&split)->default_value(end),
    "Iterativley put <arg> scans together. If not used, condense will put all scans in one file.")
    ("customFilter,u", po::value<string>(&customFilter),
    "Apply a custom filter. Filter mode and data are specified as a semicolon-seperated string:"
    "{filterMode};{nrOfParams}[;param1][;param2][...]\n"
    "Multiple filters can be specified in a file (syntax in file is same as direct specification\n"
    "FILE;{fileName}\n"
    "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("reduce,r", po::value<double>(&red)->default_value(-1.0),
    "turns on octree based point reduction (voxel size=<NR>)")
    ("octree,O", po::value<int>(&octree)->default_value(1),
    "use randomized octree based point reduction (pts per voxel=<NR>)")
    ("scale,y", po::value<double>(&scaleFac)->default_value(0.01),
    "scale factor for point cloud in m (be aware of the different units for uos (cm) and xyz (m), (default: 0.01 means that input and output remain the same)")
    ("min,M", po::value<int>(&minDist)->default_value(-1),
    "neglegt all data points with a distance smaller than NR 'units'")
    ("max,m", po::value<int>(&maxDist)->default_value(-1),
    "neglegt all data points with a distance larger than NR 'units'")
    ("color,c", po::bool_switch(&use_color)->default_value(false),
     "export in color as RGB")
    ("reflectance,R", po::bool_switch(&use_reflectance)->default_value(false),
     "export reflectance values")
    ("type,T", po::bool_switch(&use_type)->default_value(false),
     "export type/class values")
    ("normals,N", po::bool_switch(&use_normals)->default_value(false),
     "export point normals")
    ("use_frames,p", po::bool_switch(&use_frames)->default_value(false),
    "Use .frames files instead of .pose files.")
    ("xyz,x", po::bool_switch(&use_xyz)->default_value(false),
     "export in xyz format (right handed coordinate system in m)")
    ("hexfloat,0", po::bool_switch(&hexfloat)->default_value(false),
     "export points with hexadecimal digits")
    ("highprecision,H", po::bool_switch(&high_precision)->default_value(false),
     "export points with full double precision")
    ("frame,n", po::value<int>(&frame)->default_value(-1),
     "uses frame NR for export")
    ("global,g", po::bool_switch(&global)->default_value(false),
     "Use global reference frame for export");

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<std::string>(&dir), "input dir");

  // all options
  po::options_description all;
  all.add(generic).add(input).add(hidden);

  // options visible with --help
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(input);

  // positional argument
  po::positional_options_description pd;
  pd.add("input-dir", 1);

  // process options
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vm);

  // display help
  if (vm.count("help")) {
    std::cout << cmdline_options;
    std::cout << std::endl
         << "Example usage:" << std::endl
         << "\tbin/condense -S 10 /your/directory" << std::endl;
    exit(0);
  }
  po::notify(vm);

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif

  return 0;
}

void writeMetaScan( Scan* source,
                    std::ofstream &ptsout,
                    int red,
                    bool use_reflectance,
                    bool use_xyz,
                    bool use_type,
                    bool use_color,
                    bool use_normals,
                    bool high_precision,
                    unsigned int types,
                    double scaleFac = 1.0)
{
    /*
    * "xyz reduced show" -> transformed points, globaly
    * "xyz reduced" -> local frame points
    * Here, points are required in local reference, since
    * the corresponding transformation is to be exported into the .pose file.
    */
    std::string red_string = red > 0 ? " reduced" : "";
    DataXYZ xyz  = source->get("xyz" + red_string);

    if(use_reflectance) {
        DataReflectance xyz_reflectance =
        (((DataReflectance)source->get("reflectance" + red_string)).size() == 0) ?
        source->create("reflectance" + red_string, sizeof(float)*xyz.size()) :
        source->get("reflectance" + red_string);

        if (!(types & PointType::USE_REFLECTANCE)) {
            for(unsigned int i = 0; i < xyz.size(); i++) xyz_reflectance[i] = 255;
        }
        if(use_xyz) {
            writeXYZPointsReflectance(ptsout, xyz, xyz_reflectance, scaleFac);
        } else {
            writeUOSPointsReflectance(ptsout, xyz, xyz_reflectance, scaleFac*100.0);
        }

    } else if(use_type) {
        DataType xyz_type =
        (((DataType)source->get("type" + red_string)).size() == 0) ?
        source->create("type" + red_string, sizeof(int)*xyz.size()) :
        source->get("type" + red_string);

        if (!(types & PointType::USE_TYPE)) {
            for(unsigned int i = 0; i < xyz.size(); i++) xyz_type[i] = 0;
        }
        if(use_xyz) {
            writeXYZC(ptsout, xyz, xyz_type, scaleFac, hexfloat, high_precision);
        } else {
            writeUOSC(ptsout, xyz, xyz_type, scaleFac*100.0, hexfloat, high_precision);
        }

    } else if(use_color) {
        std::string data_string = red > 0 ? "color reduced" : "rgb";
        DataRGB xyz_color =
            (((DataRGB)source->get(data_string)).size() == 0) ?
            source->create(data_string, sizeof(unsigned char)*3*xyz.size()) :
            source->get(data_string);
        if (!(types & PointType::USE_COLOR)) {
            for(unsigned int i = 0; i < xyz.size(); i++) {
                xyz_color[i][0] = 0;
                xyz_color[i][1] = 0;
                xyz_color[i][2] = 0;
            }
        }
        if(use_xyz) {
            writeXYZRGB(ptsout, xyz, xyz_color, scaleFac, hexfloat, high_precision);
        } else {
            writeUOSRGB(ptsout, xyz, xyz_color, scaleFac*100.0, hexfloat, high_precision);
        }

    } else if(use_normals) {
        std::string data_string = red > 0 ? "normal reduced" : "normal";
        DataNormal normals =
            (((DataNormal)source->get(data_string)).size() == 0) ?
            source->create(data_string, sizeof(double)*3*xyz.size()) :
            source->get(data_string);
        if(use_xyz) {
            writeXYZNormal(ptsout, xyz, normals, scaleFac, hexfloat, high_precision);
        } else {
            writeUOSNormal(ptsout, xyz, normals, scaleFac*100.0, hexfloat, high_precision);
        }

    } else {
        if(use_xyz) {
            writeXYZPoints(ptsout, xyz, 0.01);
        } else {
            writeUOSPoints(ptsout, xyz);
        }
    }
}

Scan* createMetaScan(vector<Scan*> splitscans,
                     IOType iotype,
                     int ref,
                     string red_string = "",
                     bool use_reflectance = false,
                     bool use_type = false,
                     bool use_color = false,
                     bool use_normals = false,
                     bool global = false)
{

    // Collect all fields of all sub-sequent scan objects
    vector<double*> pts;
    vector<float> refls;
    vector<int> typevec;
    vector<unsigned char*> rgbs;
    vector<double*> normals;

    // transform points into coordinate system of ref
    double tinv[16];
    double rPos[3], rPosTheta[3];
    const double* tmp = splitscans[ref]->get_transMat();
    Matrix4ToEuler(tmp, rPosTheta, rPos);
    M4inv(tmp, tinv);


    // Collecting the data...
    for (uint iter = 0; iter < splitscans.size(); iter++) {
        Scan *sscan = splitscans[iter];
        const double* transMat = sscan->get_transMat();
        int nrpts = sscan->size<DataXYZ>("xyz" + red_string);
        DataXYZ xyz(sscan->get("xyz" + red_string)); // most important

        // Getting all fields
        DataReflectance refl(sscan->get("reflectance" + red_string));
        DataType type(sscan->get("type" + red_string));
        DataRGB rgb(sscan->get("rgb" + red_string));

        // Leave out fields that have no data. i.e. less points than nrpts
        for (int j = 0; j < nrpts; j++) {
            if (refl.size() == nrpts)
                refls.push_back(refl[j]);
            if (type.size() == nrpts)
                typevec.push_back(type[j]);
            if (rgb.size() == nrpts)
                rgbs.push_back(rgb[j]);
            if (iotype == UOS_NORMAL)
            {
                DataNormal normal(sscan->get("normal" + red_string));
                double *n = new double[3];
                n[0] = normal[j][0];
                n[1] = normal[j][1];
                n[2] = normal[j][2];
                normals.push_back(n);
            }
            double *p = new double[3];
            p[0] = xyz[j][0];
            p[1] = xyz[j][1];
            p[2] = xyz[j][2];
            transform(p, transMat);
            if (!global) transform(p, tinv);
            pts.push_back(p);
        }
    }
    if (global)
    {
        for (int i=0;i<3;++i) {
            rPos[i] = 0; rPosTheta[i] = 0;
        }
    }
    Scan *s = new BasicScan(rPos, rPosTheta, pts);
    if (use_reflectance)
    {
        float* data = reinterpret_cast<float*>(s->create("reflectance" + red_string,
                        sizeof(float) * refls.size()).get_raw_pointer());
        for(size_t i2 = 0; i2 < refls.size(); ++i2)
            data[i2] = refls[i2];
    }
    else if (use_type)
    {
        int* data = reinterpret_cast<int*>(s->create("type" + red_string,
                        sizeof(int) * typevec.size()).get_raw_pointer());
        for(size_t i2 = 0; i2 < typevec.size(); ++i2)
            data[i2] = typevec[i2];
    }
    else if (use_color)
    {
        unsigned char** data = reinterpret_cast<unsigned char**>(s->create("rgb" + red_string,
                        sizeof(unsigned char*) * rgbs.size()).get_raw_pointer());
        for(size_t i2 = 0; i2 < rgbs.size(); ++i2)
            data[i2] = rgbs[i2];
    }
    else if (use_normals)
    {
        double** data = reinterpret_cast<double**>(s->create("normal" + red_string,
                        sizeof(double*) * normals.size()).get_raw_pointer());
        for(size_t i2 = 0; i2 < normals.size(); ++i2)
            data[i2] = normals[i2];
    }
    // Cleanup
    for (int o = 0; o < pts.size(); o++)
            delete[] pts[o];
        pts.clear();

    return s;
}

#endif //_CONDENSE_H