#include "mesh/parse_options.h"

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#endif

using namespace std;
namespace po = boost::program_options;

// Parse commandline options
void parse_options(
  int argc, char **argv, int &start, int &end,
  bool &scanserver, int &max_dist, int &min_dist,
  string &dir, string &odir, IOType &iotype,
  bool &in_color, bool &reflectance, double &min_refl, double &max_refl,
  bool &no_normal, bool &join,
  double &red, int &rand, bool &use_pose,
  int &octree, bool &rangeFilterActive, bool &customFilterActive,
  string &customFilter, double &scaleFac, bool &autoRed,
  int &k1, int &k2, normal_method &ntype, int &width, int &height,
  bool &outward, int &depth, float &samplesPerNode, float &trimVal
)
{
  po::options_description cmd_options("Poisson Surface Reconstruction <options> \n"
                                      "Options list:");
  cmd_options.add_options()
      // IO parameters
      ("help,?", "Display this help message")
      ("start,s",
       po::value<int>(&start)->default_value(0),
       "Start at scan number <arg>")
      ("end,e",
       po::value<int>(&end)->default_value(-1),
       "Stop at scan number <arg>")
      ("scanserver,S",
       po::bool_switch(&scanserver)->default_value(false),
       "Use the scanserver as an input method")
      ("format,f",
       po::value<IOType>(&iotype)->default_value(UOS),
       "using shared library <arg> for input. (chose format from "
       "[uos|uosr|uos_map|uos_rgb|uos_frames|uos_map_frames|old|rts|rts_map"
       "|ifp|riegl_txt|riegl_rgb|riegl_bin|zahn|ply])")
      ("max,M",
       po::value<int>(&max_dist)->default_value(-1),
       "neglegt all data points with a distance larger than <arg> 'units")
      ("min,m",
       po::value<int>(&min_dist)->default_value(-1),
       "neglegt all data points with a distance smaller than <arg> 'units")
      ("incolor,C",
       po::bool_switch(&in_color)->default_value(false),
       "pointset contains color info")
      ("reflectance,R",
       po::bool_switch(&reflectance)->default_value(false),
       "transform reflectance information to gray scale")
      ("minR",
       po::value<double>(&min_refl)->default_value(-15.0),
       "lower threshold for reflectance grayscale")
      ("maxR",
       po::value<double>(&max_refl)->default_value(-15.0),
       "upper threshold for reflectance grayscale")
      ("nonormal,N",
       po::bool_switch(&no_normal)->default_value(false),
       "export mesh with normal data")
      // reduction and filter parameters
      ("autored,a", po::bool_switch(&autoRed)->default_value(false),
       "automatically reduce scans if necessary")
      ("reduce,r", po::value<double>(&red)->default_value(-1.0),
      "turns on octree based point reduction (voxel size=<NR>)")
      ("octree,O", po::value<int>(&octree)->default_value(1),
      "use randomized octree based point reduction (pts per voxel=<NR>)")
      ("customFilter,u", po::value<string>(&customFilter),
      "Apply a custom filter. Filter mode and data are specified as a "
      "semicolon-seperated string:"
      "{filterMode};{nrOfParams}[;param1][;param2][...]"
      "Multiple filters can be specified in a file (syntax in file is same as"
      "direct specification"
      "FILE;{fileName}"
      "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
      // Join scans parameters
      ("join,j", po::bool_switch(&join)->default_value(false),
      "whether to join scans together for surface reconstruction")
      ("scale,y", po::value<double>(&scaleFac)->default_value(1.0),
      "scale factor for point cloud in m (be aware of the different units for uos (cm) and xyz (m), default: 0.01 means that input and output remain the same)'")
      ("trustpose,t", po::bool_switch(&use_pose)->default_value(false),
      "Trust the pose file, do not extrapolate the last transformation."
      "(just for testing purposes, or gps input.)")
      // Normal parameters
      ("normal,g",
       po::value<normal_method>(&ntype)->default_value(AKNN),
       "normal calculation method "
       "(KNN, ADAPTIVE_KNN, AKNN, ADAPTIVE_AKNN"
#ifdef WITH_OPENCV
       ", PANORAMA, PANORAMA_FAST"
#endif
       ")")
      ("K1,k",
       po::value<int>(&k1)->default_value(20),
       "<arg> value of K value used in the nearest neighbor search of ANN or"
       "kmin for k-adaptation")
      ("K2,K",
       po::value<int>(&k2)->default_value(20),
       "<arg> value of Kmax for k-adaptation")
#ifdef WITH_OPENCV
      ("width,w",
       po::value<int>(&width)->default_value(3600),
       "width of panorama image")
      ("height,h",
       po::value<int>(&height)->default_value(1000),
       "height of panorama image")
#endif
      // Poisson parameters
      ("outward,I",
       po::bool_switch(&outward)->default_value(false),
       "normal direction outward? default false")
      ("depth,d",
      po::value<int>(&depth)->default_value(8),
      "value of poisson octree depth")
      ("samples,P",
      po::value<float>(&samplesPerNode)->default_value(1.0f),
      "value of poisson samples per node")
      ("trim,T",
      po::value<float>(&trimVal)->default_value(0.0f),
      "value of trimming threshold value");

  // input scan dir and output obj dir are hidden and mandatory
  po::options_description hidden("Hidden options");
  hidden.add_options()
      ("input-dir", po::value<string>(&dir), "input dir")
      ("output-dir", po::value<string>(&odir), "output dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1).add("output-dir", 1);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl << endl;
    cout << "Sample command for reconstructing surface:" << endl;
    cout << "- e.g. Join scans and trust pose with octree depth 10 and trimming threshold 7.0:" << endl;
    cout << "  bin/recon dat dat/test/test -j true -t true -d 10 -T 7.0 -i true" << endl;
    cout << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";
}

// validate normmal_method type (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values,
  normal_method*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  if (strcasecmp(arg.c_str(), "KNN") == 0)
    v = KNN;
  else if (strcasecmp(arg.c_str(), "ADAPTIVE_KNN") == 0)
    v = ADAPTIVE_KNN;
  else if (strcasecmp(arg.c_str(), "AKNN") == 0)
    v = AKNN;
  else if (strcasecmp(arg.c_str(), "ADAPTIVE_AKNN") == 0)
    v = ADAPTIVE_AKNN;
#ifdef WITH_OPENCV
  else if (strcasecmp(arg.c_str(), "PANORAMA") == 0)
    v = PANORAMA;
  else if (strcasecmp(arg.c_str(), "PANORAMA_FAST") == 0)
    v = PANORAMA_FAST;
#endif
  else throw std::runtime_error(std::string("normal calculation method ")
                                + arg + std::string(" is unknown"));
}

// validate IO types (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values,
  IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}
