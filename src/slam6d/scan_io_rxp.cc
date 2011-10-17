/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH
 * @author Jan Elseberg. Smart Systems Group, Jacobs University Bremen gGmbH, Germany. 
 */

#include "slam6d/scan_io_rxp.h"
#include "riegl/scanlib.hpp"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

#include <algorithm>
using std::swap;

#ifdef _MSC_VER
#include <windows.h>
#endif

using namespace scanlib;
using namespace std;
using namespace std::tr1;

/**
 * Reads specified scans from given directory in
 * the file format Riegl Laser Measurement GmbH 
 * uses. It will be compiled as shared lib.
 *
 * Scan poses will NOT be initialized after a call
 * to this function. Initial pose estimation works 
 * only with the -p switch, i.e., trusting the initial
 * estimations by Riegl.
 *
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_rxp::readScans(int start, int end, string &dir, int maxDist, int minDist,
						  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;

  ifstream pose_in;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  
  string rxp = ".rxp";
  if (dir.rfind(rxp) == dir.length() - rxp.length() - 1) { // dir is a .rxp file
    string shortdir = dir;
    shortdir.erase(shortdir.length() -1); // erase last character
    if (!dec) {
      rc = basic_rconnection::create(shortdir);
      rc->open();
      
      // decoder splits the binary file into readable chunks
      dec = new decoder_rxpmarker(rc);
      // importer interprets the chunks
      imp = new importer(&ptss, maxDist, minDist, start);
    } 

    buffer  buf;

    // skip the first scans
    if (imp->getCurrentScan() < start ) {
      for ( dec->get(buf); !dec->eoi(); dec->get(buf) ) {
        imp->dispatch(buf.begin(), buf.end());
        if (imp->getCurrentScan() >= start) break;
      }
    }
    if (dec->eoi()) return -1;
    int cscan = imp->getCurrentScan();
    // iterate over chunks, until the next scan is reached
    for ( dec->get(buf); !dec->eoi() ; dec->get(buf) ) {
      imp->dispatch(buf.begin(), buf.end());
      if (imp->getCurrentScan() != cscan) break;
    }
    
    for (unsigned int i = 0; i < 6; euler[i++] = 0.0);
    //done
  } else {

    poseFileName = dir + "scan" + to_string(fileCounter,3) + ".pose";
    scanFileName = "file://" + dir + "scan" + to_string(fileCounter,3) + ".rxp";

    pose_in.open(poseFileName.c_str());
    // read 3D scan

    if (!pose_in.good()) return -1; // no more files in the directory
    if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
    cout << "Processing Scan " << scanFileName;
    cout.flush();

    for (unsigned int i = 0; i < 6; pose_in >> euler[i++]);

    cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
      << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;

    // convert angles from deg to rad
    for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
    pose_in.close();
    pose_in.clear();


    // open scanfile
    rc = basic_rconnection::create(scanFileName);
    rc->open();

    // decoder splits the binary file into readable chunks
    dec = new decoder_rxpmarker(rc);
    // importer interprets the chunks
    imp = new importer(&ptss, maxDist, minDist);

    // iterate over chunks
    buffer  buf;
    for ( dec->get(buf); !dec->eoi(); dec->get(buf) ) {
      imp->dispatch(buf.begin(), buf.end());
    }

    //done
    rc->close();

  }


  fileCounter++;
  
  return fileCounter-1;
}
    
void importer::on_echo_transformed(echo_type echo)
{
  if (currentscan < start) return;
  // targets is a member std::vector that contains all
  // echoes seen so far, i.e. the current echo is always
  // indexed by target_count-1.
  target& t(targets[target_count - 1]);

  // target.reflectance
  // target.amplitude
  // target.deviation
  // target.time
  // target.vertex  point coordinates
  //

  Point p;
  
  p.x = t.vertex[1]*-100.0;
  p.y = t.vertex[2]*100.0;
  p.z = t.vertex[0]*100.0;

  p.reflectance = t.reflectance;
  p.amplitude   = t.amplitude;
  p.deviation   = t.deviation;


  if ( pointcloud::first == echo ) {
    p.type = 0;
  } else if ( pointcloud::interior == echo ) {
    p.type = 1;              
  } else if ( pointcloud::last == echo ) {
    p.type = 10;
  } else if ( pointcloud::single == echo ){
    p.type = 9;
  }
  if(maxD == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxD*maxD) {
    if(minD == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) > minD*minD) {
       {
         if((sqr(p.x) + sqr(p.y) + sqr(p.z)) > 0.1)o->push_back(p);
       }
    }
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
  return new ScanIO_rxp;
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
