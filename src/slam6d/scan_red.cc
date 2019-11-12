/*
 * scan_red implxementation
 *
 * Copyright (C) by the 3DTK contributors
 * Copyright (C) Dorit Borrmann, Razvan-George Mihalyi, Remus Dumitru
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Main program for reducing 3D scans.
 *
 * Program to reduce scans for use with slam6d
 * Usage: bin/scan_red -r <NR> 'dir',
 * Use -r for octree based reduction  (voxel size=<NR>)
 * and 'dir' the directory of a set of scans
 * Reduced scans will be written to 'dir/reduced'
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

#include "slam6d/metaScan.h"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"
#include "slam6d/Boctree.h"
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/scan_cv.h"

#include "scanserver/clientInterface.h"
#include "scanio/writer.h"

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

using namespace fbr;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

enum reduction_method {OCTREE, RANGE, INTERPOLATE, NO_REDUCTION};

/* Function used to check that 'opt1' and 'opt2' are not specified
   at the same time. */
void conflicting_options(const po::variables_map & vm,
                         const char *opt1, const char *opt2)
{
  if (vm.count(opt1) && !vm[opt1].defaulted()
      && vm.count(opt2) && !vm[opt2].defaulted())
    throw std::logic_error(std::string("Conflicting options '")
                           + opt1 + "' and '" + opt2 + "'.");
}

/* Function used to check that if 'for_what' is specified, then
   'required_option' is specified too. */
void option_dependency(const po::variables_map & vm,
                       const char *for_what, const char *required_option)
{
  if (vm.count(for_what) && !vm[for_what].defaulted())
    if (vm.count(required_option) == 0
        || vm[required_option].defaulted())
      throw std::logic_error(std::string("Option '") + for_what +
                             "' requires option '" +
                             required_option + "'.");
}

/*
 * validates panorama method specification
 */
namespace fbr {
  void validate(boost::any& v, const std::vector<std::string>& values,
                projection_method*, int) {
    if (values.size() == 0)
      throw std::runtime_error("Invalid model specification");
    std::string arg = values.at(0);
    if(strcasecmp(arg.c_str(), "EQUIRECTANGULAR") == 0) v = EQUIRECTANGULAR;
    else if(strcasecmp(arg.c_str(), "CYLINDRICAL") == 0) v = CYLINDRICAL;
    else if(strcasecmp(arg.c_str(), "MERCATOR") == 0) v = MERCATOR;


    else if(strcasecmp(arg.c_str(), "RECTILINEAR") == 0) v = RECTILINEAR;
    else if(strcasecmp(arg.c_str(), "PANNINI") == 0) v = PANNINI;
    else if(strcasecmp(arg.c_str(), "STEREOGRAPHIC") == 0) v = STEREOGRAPHIC;
    else if(strcasecmp(arg.c_str(), "EQUALAREACYLINDRICAL") == 0) v = EQUALAREACYLINDRICAL;


    else if(strcasecmp(arg.c_str(), "CONIC") == 0) v = CONIC;
    else throw std::runtime_error(std::string("projection method ")
                                  + arg
                                  + std::string(" is unknown"));
  }
}

/*
 * validates scanner type specification
 */
namespace fbr {
  void validate(boost::any& v, const std::vector<std::string>& values,
                scanner_type*, int) {
    if (values.size() == 0)
      throw std::runtime_error("Invalid scanner type");
    std::string arg = values.at(0);
    if(strcasecmp(arg.c_str(), "NONE") == 0) v = NO_REDUCTION;
    else if(strcasecmp(arg.c_str(), "RIEGL") == 0) v = RIEGL;
    else if(strcasecmp(arg.c_str(), "FARO") == 0) v = FARO;
    else throw std::runtime_error(std::string("scanner type ")
                                  + arg
                                  + std::string(" is unknown"));
  }
}

/*
 * validates input type specification
 */
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

void reduction_option_dependency(const po::variables_map & vm,
                                 reduction_method stype,
                                 const char *option)
{
  if (vm.count("reduction") && vm["reduction"].as<reduction_method>() == stype)
    {
      if (!vm.count(option)) {
        throw std::logic_error(std::string("this reduction option needs ")
                               + option
                               + " to be set");
    }
  }
}

void reduction_option_conflict(const po::variables_map & vm, reduction_method stype, const char *option)
{
  if (vm.count("reduction") && vm["reduction"].as<reduction_method>() == stype) {
    if (vm.count(option)) {
      throw std::logic_error(
          std::string("this reduction option is incompatible with ") +
          option);
    }
  }
}


/*
 * validates reduction method specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
              reduction_method*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  if(strcasecmp(arg.c_str(), "OCTREE") == 0) v = OCTREE;
  else if(strcasecmp(arg.c_str(), "RANGE") == 0) v = RANGE;
  else if(strcasecmp(arg.c_str(), "INTERPOLATE") == 0) v = INTERPOLATE;
  else if(strcasecmp(arg.c_str(), "NONE") == 0) v = NO_REDUCTION;
  else throw std::runtime_error(std::string("reduction method ")
                                + arg
                                + std::string(" is unknown"));
}

void parse_options(int argc, char **argv, int &start, int &end,
                   bool &scanserver, int &width, int &height,
                   fbr::projection_method &ptype, std::string &dir, IOType &iotype,
                   int &maxDist, int &minDist, std::string &customFilter, reduction_method &rtype, IOType &out_format, double &scale,
                   double &voxel, int &octree, bool &use_reflectance,
		   int &MIN_ANGLE, int &MAX_ANGLE, int &nImages, double &pParam,
		   fbr::scanner_type &sType, bool &loadOct, bool &use_color)
{
  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  po::options_description input("Input options");
  input.add_options()
    ("start,s", po::value<int>(&start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) "
     "[ATTENTION: counting naturally starts with 0]")
    ("end,e", po::value<int>(&end)->default_value(-1),
     "end after scan <arg>")
    ("format,f", po::value<IOType>(&iotype)->default_value(UOS),
     "using shared library <arg> for input. (chose F from {uos, uos_map, "
     "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
     "riegl_txt, riegl_rgb, riegl_bin, zahn, ply, las})")
    ("max,M", po::value<int>(&maxDist)->default_value(-1),
     "neglegt all data points with a distance larger than <arg> 'units")
    ("min,m", po::value<int>(&minDist)->default_value(-1),
     "neglegt all data points with a distance smaller than <arg> 'units")
    ("scanserver,S", po::bool_switch(&scanserver),
     "Use the scanserver as an input method and handling of scan data")
    ("loadOct,l", po::bool_switch(&loadOct)->default_value(false),
     "Use Octree to load data if it is available")
    ("customFilter,u", po::value<std::string>(&customFilter),
    "Apply a custom filter. Filter mode and data are specified as a "
    "semicolon-seperated string:\n"
    "{filterMode};{nrOfParams}[;param1][;param2][...]\n"
    "Multiple filters can be specified in a file (syntax in file is same as"
    "direct specification\n"
    "FILE;{fileName}\n"
    "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("sType,t", po::value<fbr::scanner_type>(&sType),
     "Choose scanner type");


  po::options_description reduction("Reduction options");
  reduction.add_options()
    ("reduction,r", po::value<reduction_method>(&rtype)->required(),
     "choose reduction method (OCTREE, RANGE, INTERPOLATE, NONE)")
    ("scale,y", po::value<double>(&scale),
     "scaling factor")
    ("voxel,v", po::value<double>(&voxel),
     "voxel size")
    ("projection,P", po::value<fbr::projection_method>(&ptype),
     "projection method or panorama image. Following Methods can be used: EQUIRECTANGULAR|CONIC|CYLINDRICAL|MERCATOR|RECTILINEAR|PANNINI|STEREOGRAPHIC|EQUALAREACYLINDRICAL      *Not all Projections may work with RANGE-reduction")

    ("octree,O", po::value<int>(&octree),
     "0     -> center\n1     -> random\nN > 1 -> random N\n-1    -> average")
    ("width,w", po::value<int>(&width),
     "width of panorama")
    ("height,h", po::value<int>(&height),
     "height of panorama")
    ("minVangle,a", po::value<int>(&MIN_ANGLE)->default_value(-40),
     "min vertical angle of view")
    ("maxVangle,A", po::value<int>(&MAX_ANGLE)->default_value(60),
     "max vertical angle of view")
    ("nImages,n", po::value<int>(&nImages),
     "number of horizontal images for some projections [pannini, rectilinear, stereographic]")
    ("pParam,p", po::value<double>(&pParam),
     "projection parameter for some projections [pannini, stereographic]");

  po::options_description output("Output options");
  output.add_options()
    ("reflectance,R", po::bool_switch(&use_reflectance),
     "Use reflectance when reducing points and save scan files in UOSR format")
    ("outformat", po::value<IOType>(&out_format)->default_value(UOS),
     "Output format. Chose from uos, xyz or ply" )
    ("color,C", po::bool_switch(&use_color)->default_value(false),
     "Use color when reducing points and save scan files with color");

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<std::string>(&dir), "input dir");

  // all options
  po::options_description all;
  all.add(generic).add(input).add(reduction).add(output).add(hidden);

  // options visible with --help
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(input).add(reduction).add(output);

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
         << "\t./bin/scan_red -s 0 -e 0 -f uos --reduction OCTREE --voxel 10 --octree 0 dat" << std::endl
         << "\t./bin/scan_red -s 0 -e 0 -f uos --reduction RANGE --scale 0.5 --projection EQUIRECTANGULAR --width 3600 --height 1000 dat" << std::endl
         << "\t./bin/scan_red -s 0 -e 0 -f uos --reduction INTERPOLATE --scale 0.2 --projection EQUIRECTANGULAR --width 3600 --height 1000 dat" << std::endl;
    exit(0);
  }

  po::notify(vm);

  reduction_option_dependency(vm, OCTREE, "voxel");
  reduction_option_dependency(vm, OCTREE, "octree");
  reduction_option_conflict(vm, OCTREE, "scale");
  reduction_option_conflict(vm, OCTREE, "projection");
  reduction_option_conflict(vm, OCTREE, "width");
  reduction_option_conflict(vm, OCTREE, "height");

  reduction_option_conflict(vm, RANGE, "voxel");
  reduction_option_conflict(vm, RANGE, "octree");
  reduction_option_dependency(vm, RANGE, "scale");
  reduction_option_dependency(vm, RANGE, "projection");
  reduction_option_dependency(vm, RANGE, "width");
  reduction_option_dependency(vm, RANGE, "height");

  reduction_option_conflict(vm, INTERPOLATE, "voxel");
  reduction_option_conflict(vm, INTERPOLATE, "octree");
  reduction_option_dependency(vm, INTERPOLATE, "scale");
  reduction_option_dependency(vm, INTERPOLATE, "projection");
  reduction_option_dependency(vm, INTERPOLATE, "width");
  reduction_option_dependency(vm, INTERPOLATE, "height");

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
}

void scan2mat(Scan *source, cv::Mat &mat)
{
  DataXYZ xyz = source->get("xyz");
  DataReflectance xyz_reflectance
    = (((DataReflectance)source->get("reflectance")).size() == 0) ?
      source->create("reflectance", sizeof(float)*xyz.size())
    : source->get("reflectance");
  if (((DataReflectance)source->get("reflectance")).size() == 0) {
    for(unsigned int i = 0; i < xyz.size(); i++)
      xyz_reflectance[i] = 255;
  }
  unsigned int nPoints = xyz.size();
  mat.create(nPoints,1,CV_32FC(4));
  mat = cv::Scalar::all(0);
  cv::MatIterator_<cv::Vec4f> it = mat.begin<cv::Vec4f>();
  for(unsigned int i = 0; i < nPoints; i++){
    float reflectance = xyz_reflectance[i];
    //normalize the reflectance
    reflectance += 32;
    reflectance /= 64;
    reflectance -= 0.2;
    reflectance /= 0.3;
    if (reflectance < 0) reflectance = 0;
    if (reflectance > 1) reflectance = 1;

    (*it)[0] = xyz[i][0];
    (*it)[1] = xyz[i][1];
    (*it)[2] = xyz[i][2];
    (*it)[3] = reflectance;
    ++it;
  }

}

void reduce_octree(Scan *scan, std::vector<cv::Vec4f> &reduced_points, std::vector<cv::Vec3b> &color,
                   int octree, double red, bool use_reflectance, bool use_color)
{
  if (use_reflectance) {
    unsigned int types = PointType::USE_REFLECTANCE;
    PointType pointtype(types);
    scan->setReductionParameter(red, octree, pointtype);
    scan->calcReducedPoints();

    DataXYZ xyz_reduced(scan->get("xyz reduced"));
    DataReflectance reflectance_reduced(scan->get("reflectance reduced"));

    if (xyz_reduced.size() != reflectance_reduced.size()) {
      std::cerr << "xyz_reduced size different than reflectance_reduced size"
           << std::endl;
      return;
    }

    for(unsigned int j = 0; j < xyz_reduced.size(); j++) {
      reduced_points.push_back(cv::Vec4f(xyz_reduced[j][0],
                                         xyz_reduced[j][1],
                                         xyz_reduced[j][2],
                                         reflectance_reduced[j]));
    }
  } else if (use_color) {
    unsigned int types = PointType::USE_COLOR;
    PointType pointtype(types);
    scan->setReductionParameter(red, octree, pointtype);
    scan->calcReducedPoints();

    DataXYZ xyz_reduced(scan->get("xyz reduced"));
    DataRGB color_reduced(scan->get("color reduced"));

    std::cout  << xyz_reduced.size() << " " << color_reduced.size() << std::endl;

    if (xyz_reduced.size() != color_reduced.size()) {
      std::cerr << "xyz_reduced size different than color_reduced size"
           << std::endl;
      return;
    }

    for(unsigned int j = 0; j < xyz_reduced.size(); j++) {
      reduced_points.push_back(cv::Vec4f(xyz_reduced[j][0],
                                         xyz_reduced[j][1],
                                         xyz_reduced[j][2],
                                         0.0));

      color.push_back(cv::Vec3b(color_reduced[j][0],
                                color_reduced[j][1],
                                color_reduced[j][2]));
    }
  } else {
    scan->setReductionParameter(red, octree);
    scan->calcReducedPoints();

    DataXYZ xyz_reduced(scan->get("xyz reduced"));
    for(unsigned int j = 0; j < xyz_reduced.size(); j++) {
      reduced_points.push_back(cv::Vec4f(xyz_reduced[j][0],
                                         xyz_reduced[j][1],
                                         xyz_reduced[j][2],
                                         0.0));
    }
  }
}

//void reduce_range(Scan *scan, vector<cv::Vec4f> &reduced_points, int width,
void reduce_range(cv::Mat mat, std::vector<cv::Vec4f> &reduced_points, int width,
                  int height, fbr::projection_method ptype, double scale,
                  bool use_reflectance,
		  int MIN_ANGLE, int MAX_ANGLE, int nImages, double pParam,
		  fbr::panorama_map_method mMethod, float zMin, float zMax,
		  bool imageOptimization)
{
  //panorama image(width, height, ptype);
  panorama image(width, height, ptype, nImages, pParam, mMethod, zMin, zMax, MIN_ANGLE, MAX_ANGLE, imageOptimization);
  //cv::Mat mat;
  //scan2mat(scan, mat);
  image.createPanorama(mat);
  image.getDescription();

  cv::Mat range_image_resized;
  cv::Mat reflectance_image_resized;

  resize(image.getRangeImage(), range_image_resized, cv::Size(),
         scale, scale, cv::INTER_NEAREST);
	 //scale, scale, cv::INTER_LINEAR);

  if (use_reflectance) {
    resize(image.getReflectanceImage(), reflectance_image_resized,
           cv::Size(), scale, scale, cv::INTER_NEAREST);
	   //cv::Size(), scale, scale, cv::INTER_LINEAR);
  } else {
    reflectance_image_resized.create(range_image_resized.size(), CV_8U);
    reflectance_image_resized = cv::Scalar::all(0);
  }

  image.recoverPointCloud(range_image_resized,
                          reflectance_image_resized,
                          reduced_points);
}

//void reduce_interpolation(Scan *scan,
void reduce_interpolation(cv::Mat mat,
                          std::vector<cv::Vec4f> &reduced_points,
                          int width,
                          int height,
                          fbr::projection_method ptype,
                          double scale,
                          bool use_reflectance,
			  int MIN_ANGLE, int MAX_ANGLE, int nImages, double pParam,
			  fbr::panorama_map_method mMethod, float zMin, float zMax,
			  bool imageOptimization)

{
  //panorama image(width, height, ptype);
  panorama image(width, height, ptype, nImages, pParam, mMethod, zMin, zMax, MIN_ANGLE, MAX_ANGLE, imageOptimization);
  //cv::Mat mat;
  //scan2mat(scan, mat);
  image.createPanorama(mat);
  image.getDescription();

  cv::Mat range_image_resized;
  cv::Mat reflectance_image_resized;
  resize(image.getMap(), range_image_resized, cv::Size(),
	 scale, scale, cv::INTER_NEAREST);
	 //scale, scale, cv::INTER_LINEAR);

  if (use_reflectance) {
    resize(image.getReflectanceImage(), reflectance_image_resized,
	   cv::Size(), scale, scale, cv::INTER_NEAREST);
	   //cv::Size(), scale, scale, cv::INTER_LINEAR);
  }

  for(int i = 0; i < range_image_resized.rows; i++) {
    for(int j = 0; j < range_image_resized.cols; j++) {
      cv::Vec3f vec = range_image_resized.at<cv::Vec3f>(i, j);
      if (use_reflectance) {
        reduced_points.push_back(cv::Vec4f(vec[0], vec[1], vec[2],
                    reflectance_image_resized.at<uchar>(i, j)/255.0));
      } else {
	reduced_points.push_back(cv::Vec4f(vec[0], vec[1], vec[2], 0.0));
      }
    }
  }
}


/**
 * Main program for reducing scans.
 * Usage: bin/scan_red -r <NR> 'dir',
 * Use -r for octree based reduction  (voxel size=<NR>)
 * and 'dir' the directory of a set of scans
 * Reduced scans will be written to 'dir/reduced'
 *
 */
int main(int argc, char **argv)
{
  int start, end;
  bool scanserver;
  int width, height;
  int maxDist, minDist;
  fbr::projection_method ptype;
  std::string dir;
  IOType iotype;
  reduction_method rtype;
  IOType out_format;
  double scale, voxel;
  int octree;
  bool use_reflectance;
  std::string customFilter;
  bool rangeFilterActive = false;
  bool customFilterActive = false;

  //scan
  fbr::scanner_type sType;
  bool loadOct, saveOct = false, use_color;
  //panorama
  int MIN_ANGLE, MAX_ANGLE;
  int nImages = 1;
  double pParam = 0;
  fbr::panorama_map_method mMethod = FARTHEST;
  float zMin = 0, zMax = 0;
  bool imageOptimization = false;


  parse_options(argc, argv, start, end, scanserver, width, height, ptype,
                dir, iotype, maxDist, minDist, customFilter, rtype, out_format, scale, voxel, octree,
                use_reflectance, MIN_ANGLE, MAX_ANGLE, nImages, pParam,
		sType, loadOct, use_color);

  rangeFilterActive = minDist > 0 || maxDist > 0;
  // custom filter set? quick check, needs to contain at least one ';'
  // (proper chsecking will be done case specific in pointfilter.cc)
  size_t pos = customFilter.find_first_of(";");
  if (pos != std::string::npos){
      customFilterActive = true;

      // check if customFilter is specified in file
      if (customFilter.find("FILE;") == 0){
          std::string selection_file_name = customFilter.substr(5, customFilter.length());
          std::ifstream selectionfile;
          // open the input file
          selectionfile.open(selection_file_name, std::ios::in);

          if (!selectionfile.good()){
              std::cerr << "Error loading custom filter file " << selection_file_name << "!" << std::endl;
              std::cerr << "Data will NOT be filtered!" << std::endl;
              customFilterActive = false;
          }
          else {
              std::string line;
              std::string custFilt;
              while (std::getline(selectionfile, line)){
                  if (line.find("#") == 0) continue;
                  custFilt = custFilt.append(line);
                  custFilt = custFilt.append("/");
              }
              if (custFilt.length() > 0) {
                  // last '/'
                  customFilter = custFilt.substr(0, custFilt.length() - 1);
              }
          }
          selectionfile.close();
      }
  }
  else {
      // give a warning if custom filter has been inproperly specified
      if (customFilter.length() > 0){
          std::cerr << "Custom filter: specifying string has not been set properly, data will NOT be filtered." << std::endl;
      }
  }

  for (int iter = start; iter <= end; iter++) {

    std::vector<cv::Vec4f> reduced_points;
    std::vector<cv::Vec3b> color;

    std::string reddir = dir + "reduced";
    createdirectory(reddir);

    if(rtype == NO_REDUCTION) {
      Scan::openDirectory(scanserver, dir, iotype, iter, iter);
      if(Scan::allScans.size() == 0) {
        std::cerr << "No scans found. Did you use the correct format?" << std::endl;
        exit(-1);
      }

      Scan* scan = *Scan::allScans.begin();

      if(rangeFilterActive) scan->setRangeFilter(maxDist, minDist);
      if(customFilterActive) scan->setCustomFilter(customFilter);

      if (use_reflectance) {
        DataXYZ xyz(scan->get("xyz"));
        DataReflectance reflectance(scan->get("reflectance"));

        for(unsigned int j = 0; j < xyz.size(); j++) {
          reduced_points.push_back(cv::Vec4f(xyz[j][0],
                                             xyz[j][1],
                                             xyz[j][2],
                                             reflectance[j]));
        }
        write_uosr(reduced_points,
            reddir,
            scan->getIdentifier());
      } else if(use_color) {
        DataXYZ xyz(scan->get("xyz"));
        DataRGB _color(scan->get("rgb"));

        for(unsigned int j = 0; j < xyz.size(); j++) {
          reduced_points.push_back(cv::Vec4f(xyz[j][0],
                                             xyz[j][1],
                                             xyz[j][2],
                                             0.0));

          color.push_back(cv::Vec3b(_color[j][0],
                                    _color[j][1],
                                    _color[j][2]));
        }
		switch(out_format) {
			case UOS:
				write_uos_rgb(reduced_points,
						color,
						reddir,
						scan->getIdentifier());
				break;
			case XYZ:
				/*
				write_xyz_rgb(reduced_points,
						color,
						reddir,
						scan->getIdentifier());
						*/
				break;
			case PLY:
				write_ply_rgb(reduced_points,
						color,
						reddir,
						scan->getIdentifier());
				break;
			default:
				throw std::runtime_error("unknown output format");
		}
      } else {
        DataXYZ xyz(scan->get("xyz"));
        for(unsigned int j = 0; j < xyz.size(); j++) {
          reduced_points.push_back(cv::Vec4f(xyz[j][0],
                                             xyz[j][1],
                                             xyz[j][2],
                                             0.0));
        }
        write_uos(reduced_points,
            reddir,
            scan->getIdentifier());
      }


      writeposefile(reddir,
          scan->get_rPos(),
          scan->get_rPosTheta(),
          scan->getIdentifier());
    } else if(rtype == OCTREE)
    {
      Scan::openDirectory(scanserver, dir, iotype, iter, iter);
      if(Scan::allScans.size() == 0) {
        std::cerr << "No scans found. Did you use the correct format?" << std::endl;
        exit(-1);
      }

      Scan* scan = *Scan::allScans.begin();

      if(rangeFilterActive) scan->setRangeFilter(maxDist, minDist);
      if(customFilterActive) scan->setCustomFilter(customFilter);

      reduce_octree(scan,
          reduced_points,
          color,
          octree,
          voxel,
          use_reflectance,
          use_color);

      if (use_reflectance)
        write_uosr(reduced_points,
            reddir,
            scan->getIdentifier());
      else if(use_color)
		switch(out_format) {
			case UOS:
				write_uos_rgb(reduced_points,
						color,
						reddir,
						scan->getIdentifier());
				break;
			case XYZ:
				/*
				write_xyz_rgb(reduced_points,
						color,
						reddir,
						scan->getIdentifier());
						*/
				break;
			case PLY:
				write_ply_rgb(reduced_points,
						color,
						reddir,
						scan->getIdentifier());
				break;
			default:
				throw std::runtime_error("unknown output format");
		}
      else
        write_uos(reduced_points,
            reddir,
            scan->getIdentifier());


      writeposefile(reddir,
          scan->get_rPos(),
          scan->get_rPosTheta(),
          scan->getIdentifier());

    }
    else
    {
      scan_cv sMat(dir, iter, iotype, scanserver, sType, loadOct, saveOct, use_reflectance, use_color);

      if(rtype == RANGE)
      {
        sMat.convertScanToMat();
        reduce_range(sMat.getMatScan(),
            reduced_points,
            width,
            height,
            ptype,
            scale,
            use_reflectance,
            MIN_ANGLE,
            MAX_ANGLE,
            nImages,
            pParam,
            mMethod,
            zMin,
            zMax,
            imageOptimization);


      }
      else if(rtype == INTERPOLATE)
      {
        sMat.convertScanToMat();
        reduce_interpolation(sMat.getMatScan(),
            reduced_points,
            width,
            height,
            ptype,
            scale,
            use_reflectance,
            MIN_ANGLE,
            MAX_ANGLE,
            nImages,
            pParam,
            mMethod,
            zMin,
            zMax,
            imageOptimization);

      }
      else
      {
        std::cerr << "unknown method" << std::endl;
      }

      if (use_reflectance)
        write_uosr(reduced_points,
            reddir,
            to_string(iter,3));
      /* TODO
      else if (use_color)
        write_uos_rgb(reduced_points,
            reddir,
            to_string(iter,3));
            */
      else
        write_uos(reduced_points,
            reddir,
            to_string(iter,3));

    }
    Scan::closeDirectory();
  }
}
