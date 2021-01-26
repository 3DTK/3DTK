#include "detectCylinder/detectCylinder.h"

namespace po = boost::program_options;
using namespace std;

/// =============================================
/// Main
/// =============================================
int main(int argc, char** argv)
{
  int start, end;
  bool scanserver;
  string dir;
  IOType iotype;

  parse_options(argc, argv, start, end, scanserver, dir, iotype);

  // Prepare and read scans
  if (scanserver) {
    try {
      ClientInterface::create();
    } catch(std::runtime_error& e) {
      cerr << "ClientInterface could not be created: " << e.what() << endl;
      cerr << "Start the scanserver first." << endl;
      exit(-1);
    }
  }

  /// Read the scans
  Scan::openDirectory(scanserver, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }

  /// Make directory for saving the scan segments
  string deCyldir = dir + "detectCylinder";
  int success = mkdir(deCyldir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
  if(success == 0) {
    cout << "Writing detected Cylinder to " << deCyldir << endl;
  } else if(errno == EEXIST) {
    cout << "WARN: Directory " << deCyldir << " exists already. Contents will be overwriten" << endl;
  } else {
    cerr << "Creating directory " << deCyldir << " failed" << endl;
    exit(1);
  }

  //Create Cylinder Detector
  CylinderDetector cd;

  //Put Scans in (all Data needed)
  for(std::vector<Scan*>::iterator it_scan = Scan::allScans.begin(); it_scan != Scan::allScans.end(); ++it_scan){
    //read scan Points and get normals
    DataXYZ xyz((*it_scan)->get("xyz"));
    DataNormal norm((*it_scan)->get("normal"));
    cd.addScanPoints(&xyz, &norm);
  }

  //Detect Cylinder
  std::vector<Cylinder*> v_cyl = cd.detectCylinder();

  //Write Cylinder in File
  string fileName = deCyldir + "/cylinder" + ".2d";
  ofstream cylinderOut(fileName.c_str());
  //define structure cylinder output
  cylinderOut << "#" << "CylinderIndex;Radius;Axis_x Axis_y lAxis_z; StartAxis_x StartAxis_y StartAxis_z;EndAxis_x EndAxis_y EndAxis_z;OrigPointOnAxis_x OrigPointOnAxis_y OrigPointOnAxis_z"<< std::endl;
  std::cout << "#" << "CylinderIndex(#lateralSurfacePoints, #InlierPoints);Radius;Axis_x Axis_y lAxis_z; StartAxis_x StartAxis_y StartAxis_z;EndAxis_x EndAxis_y EndAxis_z;OrigPointOnAxis_x OrigPointOnAxis_y OrigPointOnAxis_z"<< std::endl;

  //print cylinder paras
  int cylinderCounter = 0;
  for(std::vector<Cylinder*>::iterator it = v_cyl.begin(); it != v_cyl.end(); it++){
    //Get cylinder
    Cylinder cyl = (*(*it));

    //Give all Cylinder new unique, sorted cylinderIndex
    cyl.cylinderNumber = cylinderCounter;
    cylinderCounter++;

    //console outPut
    std::cout << cyl.cylinderNumber <<"("<< cyl.cylinderPoints->size() <<", " << cyl.inlierPoints<<");";
    std::cout << cyl.radius << ";";
    std::cout << cyl.axis[0] << " "<<cyl.axis[1] << " " << cyl.axis[2]<< ";";
    std::cout << cyl.cylinderAxisStart[0] << " " <<  cyl.cylinderAxisStart[1] << " " <<  cyl.cylinderAxisStart[2]<< ";";
    std::cout << cyl.cylinderAxisEnd[0] << " " <<  cyl.cylinderAxisEnd[1] << " " <<  cyl.cylinderAxisEnd[2]<< ";";
    std::cout << cyl.pAxis[0]  <<  cyl.pAxis[1] << cyl.pAxis[2]<< '\n';

    //write in File
    cylinderOut << cyl.cylinderNumber << ";";
    cylinderOut << cyl.radius << ";";
    cylinderOut << cyl.axis[0] << " "<<cyl.axis[1] << " " << cyl.axis[2]<< ";";
    cylinderOut << cyl.cylinderAxisStart[0] << " " <<  cyl.cylinderAxisStart[1] << " " <<  cyl.cylinderAxisStart[2]<< ";";
    cylinderOut << cyl.cylinderAxisEnd[0] << " " <<  cyl.cylinderAxisEnd[1] << " " <<  cyl.cylinderAxisEnd[2]<< ";";
    cylinderOut << cyl.pAxis[0]<< " " << cyl.pAxis[1]<< " " <<cyl.pAxis[2]<< '\n';

  }
  cylinderOut.clear();
  cylinderOut.close();

  //clean up
  if (scanserver)
  ClientInterface::destroy();
  else
  Scan::closeDirectory();

  std::cout << "Successfull closed" << '\n';
  return 0;
}



/**
 * void validate - validate IO types
 *
 * @param  {type} boost::any& v                          io_type
 * @param  {type} const std::vector<std::string>& values Model specification
 * @param  {type} IOType*                                Using shared library for input
 * @param  {type} int                                    arg
 */
void validate(boost::any& v, const std::vector<std::string>& values, IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}



/**
 * void parse_options - Parse + set up command line options
 *
 * @param  {type} int argc
 * @param  {type} char **argv
 * @param  {type} int &start       scan number start
 * @param  {type} int &end         scan number end
 * @param  {type} bool &scanserver Use scanserver as input method
 * @param  {type} string &dir      Input directory
 * @param  {type} IOType &iotype   Using shared library for input
 */
void parse_options(int argc, char **argv, int &start, int &end, bool &scanserver, string &dir, IOType &iotype){
  po::options_description cmd_options("Usage: detectCylinder <options> where options are (default values in brackets)");
  cmd_options.add_options()
    ("help,?", "Display this help message")
    ("start,s",
     po::value<int>(&start)->default_value(0),
     "Start at scan number <arg>")
    ("end,e",
     po::value<int>(&end)->default_value(-1),
     "Stop at scan number <arg>")
    ("scanserver,S",
     po::value<bool>(&scanserver)->default_value(false),
     "Use the scanserver as an input method")
    ("format,f",
     po::value<IOType>(&iotype)->default_value(UOS),
     "using shared library <arg> for input. "
     "(chose format from [uos|uosr|uos_map|"
     "uos_rgb|uos_frames|uos_map_frames|old|rts|rts_map|ifp|"
     "riegl_txt|riegl_rgb|riegl_bin|zahn|ply])")
    ;

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<string>(&dir), "input dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";

}
