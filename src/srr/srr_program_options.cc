#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "srr/srr_program_options.h"
#include "slam6d/globals.icc"

namespace po = boost::program_options;

inline void validate(boost::any& v, const std::vector<std::string>& values,
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

void parseArgs(int argc, char **argv, SRRSettings &settings) {

  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "produce help message")
    ("config", po::value<std::string>(), "config file for arguments")
    ("segments", po::value<std::string>(&settings.segment_filename)->default_value(""), "config file for segments");

  po::options_description general("General options");
  general.add_options()
    ("start,s", po::value<int>(&settings.start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) ")
    ("end,e", po::value<int>(&settings.end)->default_value(-1),
     "end after scan <arg>")
    ("format,f", po::value<IOType>(&settings.format)->default_value(UOS),
     "using shared library <arg> for input. "
     "(chose F from {uos, uos_map, uos_rgb, uos_frames, "
     "uos_map_frames, old, rts, rts_map, ifp, "
     "riegl_txt, riegl_rgb, riegl_bin, zahn, ply, las})")
    ("min,M", po::value<double>(&settings.filterMinDist)->default_value(150),
     "only points greater than this distance from the scanner will be used for correction (default: 150)")
    ("max,m", po::value<double>(&settings.filterMaxDist)->default_value(std::numeric_limits<double>::max()),
     "only points smaller than this distance from the scanner will be used for correction")
    ("customFilter,u", po::value<std::string>(&settings.customFilter)->default_value(""),
     "Apply a custom filter. Filter mode and data are specified as a "
     "semicolon-seperated string:\n"
     "{filterMode};{nrOfParams}[;param1][;param2][...]\n"
     "Multiple filters can be specified in a file (syntax in file is same as"
     "direct specification)\n"
     "FILE;{fileName}\n"
     "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("continue", po::bool_switch(&settings.continue_processing)->default_value(false),
     "continue using last frames entry as starting pose")
    ("anim,A", po::value<int>(&settings.anim)->default_value(-1),
     "");

  po::options_description reduction("Reduction options");
  reduction.add_options()
    ("voxel,v", po::value<double>(&settings.voxelsize)->default_value(-1.0),
     "turns on octree based point reduction (voxel size=<NR>)")
    ("octree,O", po::value<int>(&settings.octree)->default_value(1),
     "use randomized octree based point reduction (pts per voxel=<NR>)");

  po::options_description prereg("Preregistration options");
  prereg.add_options()
    ("prereg,p", po::bool_switch(&settings.prereg)->default_value(false),
     "use pre-registration")
    ("preRegInterval,P", po::value<int>(&settings.preRegInterval)->default_value(25),
     "pre-registration interval")
    ("preRegIter,J", po::value<int>(&settings.preRegIter)->default_value(1),
     "pre-registration iterations")
    ("iter,i", po::value<int>(&settings.mni)->default_value(10),
     "icp iterations for pre-registration")
    ("dist,d", po::value<double>(&settings.mdm)->default_value(20),
     "sets the maximal point-to-point distance for matching to <NR> 'units'"
     "(unit of scan data, e.g. cm)")
    ("epsICP", po::value<double>(&settings.epsICP)->default_value(1.0e-12),
     "in pre-registration stop ICP iteration if difference is smaller than NR ")
    ("metascan", po::bool_switch(&settings.meta)->default_value(false),
     "match latest group of linescans against a metascan with all previous linescans");

  po::options_description ctslam("Continuous Time SLAM options");
  ctslam.add_options()
    ("distSLAM,D", po::value<double>(&settings.mdml)->default_value(-1),
     "sets the maximal point-to-point distance for SLAM to <NR> 'units'"
     "defaults to 'dist' if not set")
    ("jter,j", po::value<int>(&settings.mnj)->default_value(10),
     "time continuous slam iterations")
    ("SLAMiter,I", po::value<int>(&settings.slamiterations)->default_value(15),
     "slam iterations")
    ("interval,N", po::value<int>(&settings.interval)->default_value(100),
     "interval of reference scans")
    ("size,S", po::value<int>(&settings.size)->default_value(100),
     "NR of scans before and after reference scans to group together")
    ("overlap", po::value<double>(&settings.overlap)->default_value(-1),
     "relative overlap between scan groups.")
    ("odomWeight", po::value<double>(&settings.odomWeightFactor)->default_value(0.05),
     "weight factor for odometry (used as odomWeight times number of linescans)")
    ("gapOdomWeight", po::value<double>(&settings.gapOdomWeightFactor)->default_value(0.01),
     "weight factor for odometry at segment boundaries")
    ("clpairs,C", po::value<int>(&settings.clPairs)->default_value(250),
     "specifies the minimal number of points for an overlap to add an edge in the gaph");

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<std::string>(&settings.dir), "input dir");

  po::options_description visible;
  visible.add(general).add(reduction).add(prereg).add(ctslam);

  po::options_description all;
  all.add(generic).add(visible).add(hidden);

  po::positional_options_description pd;
  pd.add("input-dir", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vm);

  if(vm.count("config")) {
    std::string configfilename = vm["config"].as<std::string>();
    std::ifstream ifs{configfilename.c_str()};
    if(ifs) {
      std::cout << "read configuration from " << configfilename << std::endl;
      po::store(po::parse_config_file(ifs, visible), vm);
    }
  }

  po::notify(vm);

  if (vm.count("help")) {
    std::cout << "Usage: " << argv[0] << " [options] <input-dir>" << std::endl;
    std::cout << generic << visible << std::endl;
    exit(0);
  }

  if (vm.count("input-dir") != 1) {
    std::cerr << "Error: Please specify a directory. See --help for options." << std::endl;
    exit(1);
  }

  if (settings.start < 0) {
		throw std::logic_error("Cannot start at a negative scan number.");
	}
	if (settings.end < -1) {
		throw std::logic_error("Cannot end at a negative scan number.");
	}
	if (0 <= settings.end && settings.end <= settings.start) {
		throw std::logic_error("<end> (" + to_string(settings.end)
        + ") cannot be smaller or equal than <start> (" + to_string(settings.start)
        + ").");
	}

  if (settings.voxelsize == 0 || settings.voxelsize < -1) {
    throw std::logic_error("Voxelsize cannot be zero.");
  }

  if(settings.mdm <= 0) {
    throw std::logic_error("icp search distance cannot be less equal than 0");
  }

  if (settings.mdml <= 0) {
    settings.mdml = settings.mdm;
  }

  if(vm.count("overlap") && settings.overlap != -1 && vm.count("size")) {
    // scan group generation with with +/- interval/2 around representative linescans
    // plus an overlap specified relative to interval size.
    std::cout << "use relative overlap instead of size in scan group generation" << std::endl;
  }

  if(settings.size < settings.interval/2 && settings.overlap < 0) {
    // scan group generation with +/-size around representative linescans;
    throw std::logic_error("size mus be > interval/2, to avoid gaps between scan groups");
  }

  if(settings.overlap != -1) {
    if(settings.overlap < 0.0) {
      throw std::logic_error("relative overlap between scan groups must be >= 0");
    }
  }

  const char separator =
#ifdef _MSC_VER
  '\\';
#else
  '/';
#endif

  if (vm.count("input-dir")) {
    if (settings.dir.back() != separator) {
      settings.dir += separator;
    }
  }

}
