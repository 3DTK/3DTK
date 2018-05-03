/*
 * exportPoints implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

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
#include "slam6d/globals.icc"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <boost/program_options.hpp>
namespace po = boost::program_options;

/**
 * Explains the usage of this program's command line parameters
 */
void usage(char* prog)
{
#ifndef _MSC_VER
  const std::string bold("\033[1m");
  const std::string normal("\033[m");
#else
  const std::string bold("");
  const std::string normal("");
#endif
  std::cout << std::endl
	  << bold << "USAGE " << normal << std::endl
	  << "   " << prog << " [options] directory" << std::endl << std::endl;
  std::cout << bold << "OPTIONS" << normal << std::endl
	  << std::endl
	  << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << std::endl
	  << "         end after scan NR" << std::endl
	  << std::endl
	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << std::endl
	  << "         using shared library F for input" << std::endl
	  << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply})" << std::endl
	  << std::endl
	  << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << std::endl
	  << "         neglegt all data points with a distance larger than NR 'units'" << std::endl
	  << std::endl
	  << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << std::endl
	  << "         neglegt all data points with a distance smaller than NR 'units'" << std::endl
	  << std::endl
	  << bold << "  -O" << normal << " NR (optional), " << bold << "--octree=" << normal << "NR (optional)" << std::endl
	  << "         use randomized octree based point reduction (pts per voxel=<NR>)" << std::endl
	  << "         requires -r or --reduce" << std::endl
	  << std::endl
	  << bold << "  -p, --trustpose" << normal << std::endl
	  << "         Trust the pose file, do not use the information stored in the .frames." << std::endl
	  << "         (just for testing purposes.)" << std::endl
	  << std::endl
	  << std::endl
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << std::endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << std::endl
	  << std::endl
    /*
	 << bold << "  -R" << normal << " NR, " << bold << "--random=" << normal << "NR" << std::endl
      << "         turns on randomized reduction, using about every <NR>-th point only" << std::endl
      << std::endl
    */
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << std::endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << std::endl
	  << "         [ATTENTION: counting naturally starts with 0]" << std::endl
	  << std::endl
	  << bold << "  -u" << normal << " STR, " << bold << "--customFilter=" << normal << "STR" << std::endl
	  << "         apply custom filter, filter mode and data are specified as semicolon-seperated string:" << std::endl
	  << "         STR: '{filterMode};{nrOfParams}[;param1][;param2][...]'" << std::endl
	  << "         see filter implementation in pointfilter.cc for more detail." << std::endl
	  << std::endl
	  << bold << "  -c, --color" << std::endl << normal
	  << "         export in color as RGB" << std::endl
	  << std::endl
	  << bold << "  -R, --reflectance, --reflectivity" << std::endl << normal
	  << "         export in reflectance" << std::endl
	  << std::endl
	  << bold << "  -x, --xyz" << std::endl << normal
	  << "         export in xyz format (right handed coordinate system in m)" << std::endl
	  << std::endl
	  << bold << "  -y" << normal << " NR, " << bold << "--scale=" << normal << "NR" << std::endl
	  << "         scale factor for export in XYZ format (default value is 0.01, so output will be in [m])" << std::endl
	  << bold << "  -H, --highprecision" << std::endl << normal
	  << "         export points with full double precision" << std::endl
	  << std::endl
	  << bold << "  --hexfloat" << std::endl << normal
	  << "         export points with hexadecimal digits" << std::endl
	  << std::endl
    << bold << "  -n" << normal << " NR, " << bold << "--frame=" << normal << "NR" << std::endl
    << "         uses frame NR for export" << std::endl    
	  << std::endl
	  << std::endl << std::endl;
  
  std::cout << bold << "EXAMPLES " << normal << std::endl
      << "   " << prog << " -s 2 -e 3 dat" << std::endl << std::endl;
  exit(1);
}


/** A function that parses the command-line arguments and sets the respective flags.
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir the directory
 * @param red using point reduction?
 * @param mdm maximal distance match
 * @param mdml maximal distance match for SLAM
 * @param mni maximal number of iterations
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param maxDist - maximal distance of points being loaded
 * @param minDist - minimal distance of points being loaded
 * @param quiet switches on/off the quiet mode
 * @param veryQuiet switches on/off the 'very quiet' mode
 * @param use_pose - i.e., use the original pose information instead of using
 * the final transformation from the frames 
 * @param meta match against all scans (= meta scan), or against the last scan only???
 * @param anim selects the rotation representation for the matching algorithm
 * @param mni_lum sets the maximal number of iterations for SLAM
 * @param net specifies the file that includes the net structure for SLAM
 * @param cldist specifies the maximal distance for closed loops
 * @param epsilonICP stop ICP iteration if difference is smaller than this value
 * @param epsilonSLAM stop SLAM iteration if average difference is smaller than this value
 * @param algo specfies the used algorithm for rotation computation
 * @param lum6DAlgo specifies the used algorithm for global SLAM correction
 * @param loopsize defines the minimal loop size
 * @return 0, if the parsing was successful. 1 otherwise
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




int parseArgs(int argc, char **argv, std::string &dir, double &red, int &rand,
            int &start, int &end, int &maxDist, int &minDist, bool &use_pose,
            bool &use_xyz, bool &use_reflectance, bool &use_color, int &octree, IOType &type, std::string& customFilter, double &scaleFac,
	    bool &hexfloat, bool &high_precision, int &frame)
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
    ("customFilter,u", po::value<string>(&customFilter)->default_value(" "),
    "Apply a custom filter. Filter mode and data are specified as a "
    "semicolon-seperated string:"
    "{filterMode};{nrOfParams}[;param1][;param2][...]"
    "Multiple filters can be specified in a file (syntax in file is same as"
    "direct specification"
    "FILE;{fileName}"
    "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("reduce,r", po::value<double>(&red)->default_value(-1.0),
    "turns on octree based point reduction (voxel size=<NR>)")
    ("octree,O", po::value<int>(&octree)->default_value(0),
    "use randomized octree based point reduction (pts per voxel=<NR>)")
    ("scale,y", po::value<double>(&scaleFac)->default_value(0.01),
    "use ICP with scale'")
    ("color,c", po::bool_switch(&use_color)->default_value(false),
     "export in color as RGB")
    ("reflectance,R", po::bool_switch(&use_reflectance)->default_value(false),
     "end after scan <arg>")
    ("trustpose,p", po::bool_switch(&use_pose)->default_value(true),
    "Trust the pose file, do not extrapolate the last transformation."
    "(just for testing purposes, or gps input.)")
    ("xyz,x", po::bool_switch(&use_xyz)->default_value(false),
     "export in xyz format (right handed coordinate system in m)")
    ("hexfloat,0", po::bool_switch(&hexfloat)->default_value(false),
     "export points with hexadecimal digits")
    ("highprecision,H", po::bool_switch(&high_precision)->default_value(true),
     "export points with full double precision")
    ("frame,n", po::value<int>(&frame)->default_value(-1),
     "uses frame NR for export");

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
         << "\t./bin/pose2frames -s 0 -e 1 /Your/directory" << std::endl;
    exit(0);
  }
  po::notify(vm);

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  
  //parseFormatFile(dir, type, start, end);

  return 0;
}

void readFrames(std::string dir, int start, int end, int frame, bool use_pose=false)
{
  std::ifstream frame_in;
  int  fileCounter = start;
  std::string frameFileName;
  if((int)(start + Scan::allScans.size() - 1) > end) end = start + Scan::allScans.size() - 1;
  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    
    frameFileName = dir + "scan" + to_string(fileCounter++,3) + ".frames";
    if(!use_pose) {

      frame_in.open(frameFileName.c_str());

      // read 3D scan
      if (!frame_in.good()) break; // no more files in the directory

      std::cout << "Reading Frames for 3D Scan " << frameFileName << "..." << std::endl;

      double transMat[16];
      int algoTypeInt;

      int frameCounter = 0;
      while (frame_in.good()) {
        if (frame != -1 && frameCounter > frame) break;
        frameCounter++;
        try {
          frame_in >> transMat >> algoTypeInt;
        }
        catch (const std::exception &e) {   
          break;
        }
      }

      // calculate RELATIVE transformation
      const double * transMatOrig = Scan::allScans[fileCounter - start - 1]->get_transMatOrg();
      double tinv[16];
      M4inv(transMatOrig, tinv);

      double tfin[16];
      MMult(transMat, tinv, tfin);
      //Scan::allScans[fileCounter - start - 1]->transformMatrix(tfin);
      //Scan::allScans[fileCounter - start - 1]->transformMatrix(tinv);
      // save final pose in scan
      Scan::allScans[fileCounter - start - 1]->transformMatrix(tfin);
      
      Scan::allScans[fileCounter - start - 1]->transformAll(transMat);
    
    } else {
      const double * transMatOrig = Scan::allScans[fileCounter - start - 1]->get_transMatOrg();
      Scan::allScans[fileCounter - start - 1]->transformAll(transMatOrig);
    }
    frame_in.close();
    frame_in.clear();
  }
}

/**
 * program for point export
 * Usage: bin/exportPoints 'dir',
 * with 'dir' the directory of a set of scans
 * ...
 */
int main(int argc, char **argv)
{
  if (argc <= 1) {
    usage(argv[0]);
  }

  // parsing the command line parameters
  // init, default values if not specified
  std::string dir;
  double red   = -1.0;
  int    rand  = -1;
  int    start = 0,   end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  bool   uP         = false;  // should we use the pose information instead of the frames?? 
  bool   use_xyz = false;
  bool   use_color = false;
  bool   use_reflectance = false;
  int octree       = 0;  // employ randomized octree reduction?
  IOType iotype    = UOS;
  bool rangeFilterActive = false;
  bool customFilterActive = false;
  std::string customFilter;
  double scaleFac = 0.01;
  bool hexfloat = false;
  bool high_precision = false;
  int frame = -1;

  parseArgs(argc, argv, dir, red, rand, start, end,
      maxDist, minDist, uP, use_xyz, use_reflectance, use_color, octree, iotype, customFilter, scaleFac,
      hexfloat, high_precision, frame);


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

  // Get Scans
  Scan::openDirectory(false, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    std::cerr << "No scans found. Did you use the correct format?" << std::endl;
    exit(-1);
  }

  unsigned int types = PointType::USE_NONE;
  if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
  if(supportsColor(iotype)) types |= PointType::USE_COLOR;

  // if specified, filter scans
  for (size_t i = 0; i < Scan::allScans.size(); i++)  {
     if(rangeFilterActive) Scan::allScans[i]->setRangeFilter(maxDist, minDist);
     if(customFilterActive) Scan::allScans[i]->setCustomFilter(customFilter);
  }
  
//
  int end_reduction = (int)Scan::allScans.size();
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int iterator = 0; iterator < end_reduction; iterator++) {
    if (red > 0) {
      PointType pointtype(types);
      std::cout << "Reducing Scan No. " << iterator << std::endl;
      Scan::allScans[iterator]->setReductionParameter(red, octree, pointtype);
      Scan::allScans[iterator]->calcReducedPoints();
    } else {
      std::cout << "Copying Scan No. " << iterator << std::endl;
    }
    // reduction filter for current scan!
  }

  readFrames(dir, start, end, frame, uP);
  
 std::cout << "Export all 3D Points to file \"points.pts\"" << std::endl;
 std::cout << "Export all 6DoF poses to file \"positions.txt\"" << std::endl;
 std::cout << "Export all 6DoF matrices to file \"poses.txt\"" << std::endl;
 FILE *redptsout = fopen("points.pts", "w");
 std::ofstream posesout("positions.txt");
 std::ofstream matricesout("poses.txt");
  
  for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
    Scan *source = Scan::allScans[i];
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
        write_xyzr(xyz, xyz_reflectance, redptsout, scaleFac, hexfloat, high_precision);
      } else {
        write_uosr(xyz, xyz_reflectance, redptsout, hexfloat, high_precision);
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
        write_xyz_rgb(xyz, xyz_color, redptsout, scaleFac, hexfloat, high_precision);
      } else {
        write_uos_rgb(xyz, xyz_color, redptsout, hexfloat, high_precision);
      }

    } else {
      if(use_xyz) {
        write_xyz(xyz, redptsout, scaleFac, hexfloat, high_precision);
      } else {
        write_uos(xyz, redptsout, hexfloat, high_precision);
      }
    
    }
    if(use_xyz) {
      writeTrajectoryXYZ(posesout, source->get_transMat(), false, scaleFac);
      writeTrajectoryXYZ(matricesout, source->get_transMat(), true, scaleFac);
    } else {
      writeTrajectoryUOS(posesout, source->get_transMat(), false);
      writeTrajectoryUOS(matricesout, source->get_transMat(), true);
    }

  }
   
  fclose(redptsout);
  posesout.close();
  posesout.clear();
  matricesout.close();
  matricesout.clear();
}
