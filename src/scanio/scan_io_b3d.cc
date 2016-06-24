/*
 * scan_io_uos implementation
 *
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Thomas Escher. Institute of Computer Science, University of Osnabrueck, Germany.
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

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"



#define DATA_PATH_PREFIX "scan"
#define DATA_PATH_SUFFIX ".b3d"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".bps"

inline bool ScanIO_b3d::readEOL(std::ifstream &f) {
    uint16_t buf;
    f.read((char*)&buf, 2);
    return buf;
}



std::list<std::string> ScanIO_b3d::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
    const char* data_path_suffixes[2] = {DATA_PATH_SUFFIX, NULL};
    return readDirectoryHelper(dir_path, start, end, data_path_suffixes);
}

void ScanIO_b3d::readPose(const char* dir_path, const char* identifier, double* pose)
{
}

time_t ScanIO_b3d::lastModified(const char* dir_path, const char* identifier)
{
  const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
  return lastModifiedHelper(dir_path, identifier, suffixes);
}

bool ScanIO_b3d::supports(IODataType type)
{
  return !!(type & (DATA_XYZ));
}

void ScanIO_b3d::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
/*
  double tmat[16] = {0.54795052483708983626, 0.83650940154068242904, -0.0014978200183333721142, 0, 0.83651057876831369509, -0.54795065951447730068, 0.00035545231819502260155, 0, 0.00052339226090998515126, 0.0014477125747364113281, 0.99999881509371923993, 0, 4365199.7430608589202, 5527044.3391020325944, 275.19461287117206894, 1};
  cout << tmat << endl;*/


    // error handling
    path data_path(dir_path);
    data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
    if(!exists(data_path))
        throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");

    path pose_path(dir_path);
    pose_path /= path(std::string("scan") + identifier + POSE_PATH_SUFFIX);
    if (!exists(pose_path)) {
        throw std::runtime_error(std::
                string("There is no pose file for [") +
                identifier + "] in [" + dir_path + "]");
    }

    if(xyz != 0) {
//      const string &d = data_path;
//      const string &p = pose_path;
      B3DPSReader *reader = new B3DPSReader(data_path.string(), pose_path.string());
      vector<double *> pts;
      reader->toGlobal(pts, false); 
   
      for (unsigned int i = 0; i < pts.size(); i++) {
        double *p = pts[i]; 
       /* 
        Point P(p[0], p[1], p[2]);
        P.transform(tmat);
    //    printf("%lf %lf %lf %lf %lf\n", P.x, P.y, P.z, reader->getReflectance(p), reader->getTime(p) );
        printf("%lf %lf %lf\n", P.x, P.y, P.z );
        */
        
//       if (p[1] < 70.0) continue;
       
        for (unsigned int j=0; j < 3; j++)
          xyz->push_back(p[j]);

        //fprintf(stderr, "%lf %lf %lf %lf %lf\n", p[0], p[1], p[2], reader->getReflectance(p), reader->getTime(p));
//        printf("%lf %lf %lf\n", p[0], p[1], p[2]); 
          
        if(reflectance) reflectance->push_back( reader->getReflectance(p) );
        if(deviation) deviation->push_back( reader->getTime(p) ); 
      }

//      cout << "Read " << xyz->size() << " points from line " << minpose << " to line " << maxpose << endl;
      delete reader;
    }

/*
    if(xyz != 0) {
      double calibration[16];
    // faro Upwards Calibration
      double rPos[3] = {0.0, 0.0, 0.00};
      double rPosTheta[3] = {0.0, 0.0, 0.0};
      EulerToMatrix4(rPos, rPosTheta, calibration);
      cout << "CALIBRATION: " << calibration << endl;
      
      // scaling matrix for slam6d coordinate system
      double S[16] =
      { 0, 0, 100, 0,
        -100, 0, 0, 0,
        0, 100, 0, 0,
        0, 0, 0, 1 };
      double Sinv[16];
      M4inv(S, Sinv);

      double SC[16];
      MMult(S, calibration, SC);
  

      ifstream scan_in, pose_in;
      scan_in.open(data_path, std::ios::in | std::ios::binary);
      pose_in.open(pose_path, std::ios::in | std::ios::binary);


      // read 3D scan
      if (!pose_in.good() && !scan_in.good()) return; // no more files in the directory
      if (!pose_in.good()) { cerr << "ERROR: Missing file " << data_path << endl; exit(1); }
      if (!scan_in.good()) { cerr << "ERROR: Missing file " << data_path << endl; exit(1); }


      // read headers 
      char buffer[1024];

      scan_in.read(buffer, 10);
      if ( buffer[0] != 'M' || buffer[1] != '3' || buffer[2] != 'D') {
        cerr << "ERROR: " << data_path << " is not a MIM B3D file!" << endl;
        scan_in.close();
        return;
      }
      hasreflectance = ( buffer[4] != 0);
      hasintensity   = ( buffer[5] != 0);
      hasamplitude   = ( buffer[6] != 0);
      hastemperature = ( buffer[7] != 0);
      hastype        = ( buffer[8] != 0);
      hasrgb         = ( buffer[9] != 0);

      
      int pointdim = 4;
      if(hasreflectance ) { reflectanceidx = pointdim++; }
      if(hasintensity   ) { amplitudeidx   = pointdim++; }
      if(hasamplitude   ) { intensityidx   = pointdim++; }
      if(hastemperature ) { rgbidx         = pointdim++; }
      if(hastype        ) { temperatureidx = pointdim++; }
      if(hasrgb         ) { typeidx        = pointdim++; }

      
      int nrbytes;
      scan_in.read((char*)&nrbytes, sizeof(int));

      if (!readEOL(scan_in) ) {
        cerr << "ERROR: Read error in :" << data_path << "." << endl;
        scan_in.close();
        return;
      }

      pose_in.read(buffer, 3);
      if ( buffer[0] != 'M' || buffer[1] != 'P' || buffer[2] != 'S') {
        cerr << "ERROR: " << pose_path << " is not a MIM BPS file!" << endl;
        pose_in.close();
        return;
      } 
      if (!readEOL(pose_in) ) {
        cerr << "ERROR: Read error in :" << data_path << "." << endl;
        pose_in.close();
        return;
      }

      // read poses
      std::map<uint64_t, double*> poses;
      uint64_t minpose = UINT64_MAX;
      uint64_t maxpose = 0;

      while (pose_in.good()) {
        uint64_t linenr;
        pose_in.read((char*)&linenr, sizeof(uint64_t));

        if (linenr < minpose) minpose = linenr;
        if (linenr > maxpose) maxpose = linenr;

        double *pose = new double[8];
        pose_in.read((char*)pose, sizeof(double)*8);
        poses[linenr] = pose;

        if (!readEOL(pose_in) ) {
          pose_in.close();
          break;     
        }
      }

      // read points
      while (scan_in.good()) {
        uint64_t linenr;
        scan_in.read((char*)&linenr, sizeof(uint64_t));
        if (!scan_in.good()) break;

        uint32_t nrpoints;
        scan_in.read((char*)&nrpoints, sizeof(uint32_t));

        uint32_t nrbytes;
        scan_in.read((char*)&nrbytes, sizeof(uint32_t));

        double points[nrpoints * pointdim];
        scan_in.read((char*)points, nrbytes-2);

        if (!readEOL(scan_in) ) {
          scan_in.close();
          break;     
        }

        if (linenr > maxpose) break; // no more pose information for this and all following points
        if(true) {
          double ax = atan2(points[0], points[1]);

          // set pose
          if (poses.find(linenr) == poses.end() ) continue;
          double *pose = poses[linenr];
          double tm[16], transMat[16];
          QuatToMatrix4(pose + 4, pose + 1, tm); 
      
          double P[16];
          QuatToMatrix4(pose + 4, pose + 1, P); 
          double PS[16];

          MMult(P, Sinv ,PS);
          MMult(S, PS, transMat);
//          cout << linenr << "  TM " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4] << " " << pose[5] << " " << pose[6] << " " << pose[7] << endl;
//          cout << points[3] << endl;
//            cout <<  linenr << " " <<transMat << endl;
          // TODO transform points here, for motion compensation
          for (unsigned int i = 0; i < nrpoints; i++) {
            double p[3];
            p[0] = points[i*pointdim+0];
            p[1] = points[i*pointdim+1];
            p[2] = points[i*pointdim+2];
            if (sqrt( p[0] * p[0] + p[1] * p[1] + p[2] * p[2] ) < 1.5 || p[2] < -10.0 ) continue; // TODO Hack for trimble data 
//            if (deg(ax) > 95.0 && deg(ax) < 145.0 && sqrt( p[0] * p[0] + p[1] * p[1] + p[2] * p[2] ) < 1.5) continue; // TODO Hack to filter out person pushing cart
//            if (deg(ax) > -100.0 && deg(ax) < -25.0 && sqrt( p[0] * p[0] + p[1] * p[1] + p[2] * p[2] ) < 0.80 ) continue; // TODO Hack to filter out carriage
//            cerr << p[0] << " " << p[1] << " " << p[2] << " " << linenr << endl;
            transform3(SC, p);
            transform3(transMat, p);
            for (unsigned int j=0; j < 3; j++)
              xyz->push_back(p[j]);
              
            if (type) type->push_back(linenr);  // TODO actual type if any
            if (hasreflectance && reflectance) reflectance->push_back( points[i * pointdim + reflectanceidx] ); // TODO use other info as well

          }
        }
      }

      cout << "Read " << xyz->size() << " points from line " << minpose << " to line " << maxpose << endl;

      // done
      scan_in.close();
      scan_in.clear();
      pose_in.close();
      pose_in.clear();
    }
    */
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
