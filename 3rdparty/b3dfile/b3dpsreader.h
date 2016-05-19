#ifndef __B3DPSREADER_H__
#define __B3DPSREADER_H__

#include "slam6d/globals.icc"
#include <cstdlib>
#include <iomanip>
#include <map>
using std::map;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;
using std::string;
#include <iostream>
#include <fstream>
#include <queue>
#include <utility>
using std::queue;
using std::cout;
using std::cerr;
using std::endl;
using std::pair;
using std::ifstream;

#include "b3dpsfile.h"

class B3DPSReader : B3DPSFile {
    std::ifstream iffile3d;
    std::ifstream iffileps;

    std::string filename3d;
    std::string filenameps;

    map<uint64_t, double*> poses;
    map<uint64_t, pair<uint32_t, double *> > orig_points;
    
    bool mydeserialize(double* &points, uint32_t &nrpoints, uint64_t &linenr);

    static inline bool readEOL(ifstream &f) {
      uint16_t buf;
      f.read((char*)&buf, 2);
      return buf; 
    }

    uint64_t minpose;
    uint64_t maxpose;

    
public:
    B3DPSReader(std::string _filename3d, std::string _filenameps); 
    ~B3DPSReader();
    static void readTrajectory(string filename, map<uint64_t, double*> &poses, uint64_t &minpose, uint64_t &maxpose) ;
    bool deserializeHeader();
    bool deserialize(vector<double*> &points, long &_linenr);
    void deserializeFree(vector<double*> &points) {
      if (points.empty()) return;
      delete[] points[0];
      points.clear();
    }

    double *getPose(uint64_t linenr) {
      return poses[linenr];
    }

    void setBPS(string filenamebps) {
      minpose = UINT64_MAX;
      maxpose = 0;
      readTrajectory(filenamebps, poses, minpose, maxpose);
    }
    
    template <typename T>
    inline T getReflectance(T * &points) { return points[idxreflectance]; }
    template <typename T>
    inline T getAmplitude(T * &points) { return points[idxamplitude]; }
    template <typename T>
    inline T getIntensity(T * &points) { return points[idxintensity]; }
    template <typename T>
    inline T getTemperature(T * &points) { return points[idxtemperature]; }
    template <typename T>
    inline T getTime(T * &points) { return points[3]; }

    template <typename T>
    void toLocal(vector<T *> &localpoints, double min, double max) {
        // read points
        do {
            double *points = 0;
            uint32_t nrpoints;
            uint64_t linenr;
        
            bool result = mydeserialize(points, nrpoints, linenr); // 
            if (!result) {
                delete[] points;
                break; 
            }
            
//            if (linenr < 60000) continue;
//            if (linenr > 65000) break;
            
            for (unsigned int i = 0; i < nrpoints; i++) {
                T *p = new T[pointdim];
                p[0] = points[i*pointdim+0];
                p[1] = points[i*pointdim+1];
                p[2] = points[i*pointdim+2];
                p[3] = points[i*pointdim+3]; // timestamp
                if (hasreflectance) p[idxreflectance] = points[i*pointdim + idxreflectance]; 

                if (p[3] > max ) {
                  return; // TODO fix memory leak here
                }
                if (p[3] < min) {
                    delete[] p;
                    break;
                }
                localpoints.push_back(p);
            }

            delete []points;
            
        } while(true);

        
    }

    template <typename T>
    void toGlobal(vector<T *> &globalpoints, bool useOriginalPoints = false) {
      int lc = 0;
      int fl = -1;
//      minpose = 81100;
//      maxpose = 85300; 
//      minpose = 120000;
//      maxpose = 180000;
//      maxpose = 400000;

        // read points from file if we are not supposed to use orig. points, 
        // or if we should, but they are empty
        bool readFromFile = !useOriginalPoints || orig_points.empty();
        auto opit = orig_points.begin(); // in case we need to get original points
      double calinv[16];
      
      double rPos[3] = {0.3, 0.0, -0.1};
      double rPosTheta[3] = {rad(0.0), rad(0.0), -rad(30.0)};
      EulerToMatrix4(rPos, rPosTheta, calinv);
      M4inv(calinv, calibration);
        
        cout << calibration << endl;
  //double tmat[16] = {0.11227781266720060493, 0.99367579916241322024, -0.0014487724487998857904, 0, -0.99367672626474168318, 0.11227842180544289619, 0.00034594315239551941266, 0, 0.00050642122252784029304, 0.0014007697235678586833, 0.99999889069024816557, 0, 4365199.7652809200808, 5527044.4005631273612, 275.19399283129524747, 1};
//        double tmat[16]; M4identity(tmat);
//  cout << tmat << endl;
        
        // scaling matrix for slam6d coordinate system
        double S[16] =
        { 0, 0, 100, 0,
        -100, 0, 0, 0,
        0, 100, 0, 0,
        0, 0, 0, 1 };       
        
//        double S[16]; M4identity(S);
        double Sinv[16];
        M4inv(S, Sinv);
        
        double SC[16];
        MMult(S, calibration, SC);
        
        // read points
        do {
            double *points = 0;
            uint32_t nrpoints;
            uint64_t linenr;
        
            if (readFromFile) {
                bool result = mydeserialize(points, nrpoints, linenr); // 
                if (!result) {
                    delete[] points;
                    break; 
                }
                if (fl < 0) fl = linenr;
                if (linenr < minpose ) {
                  delete[] points;
                  continue;;
                }
                if (linenr > maxpose) {
                  delete[] points;
                  cout << globalpoints.size() << endl;
                  break;
                }
                lc++;
                if (useOriginalPoints) {
                    //auto op = pair(nrpoints, points);
                    auto op = std::make_pair(nrpoints, points);
                    orig_points[linenr] = op;
                }
                // set pose
            } else { // get points from orig_points
                linenr = opit->first;
                nrpoints = opit->second.first;
                points = opit->second.second;
                
            }

//            if (linenr < 11000) continue;
//            if (linenr < 60000) continue;
//            if (linenr > 65000) break;
            
            if (poses.find(linenr) == poses.end() ) continue;
            double *pose = poses[linenr];
            
            double tm[16], transMat[16];
            QuatToMatrix4(pose + 4, pose + 1, tm); 
            
            double P[16];
            QuatToMatrix4(pose + 4, pose + 1, P); 
            double PS[16];
                    
            MMult(P, Sinv ,PS);
            MMult(S, PS, transMat);

  /*          
            //cout << transMat << endl;
            double PP[3];
            PP[0] = 0.0;
            PP[1] = 0.0;
            PP[2] = 0.0;
            transform3(transMat, PP);
            transform3(tmat, PP);
            cout << std::setprecision(15); 
            cout << PP[0] << " " << PP[1] << " " << PP[2] << endl;
            */
            //cout << linenr << " " << transMat << endl; 
            // TODO transform points here, for motion compensation
//          double ax = atan2(points[0], points[1]);

            for (unsigned int i = 0; i < nrpoints; i++) {
//            for (unsigned int i = 0; i < 1; i++) 
//              if (sqrt(points[i*pointdim+0]*points[i*pointdim+0] + points[i*pointdim+1]*points[i*pointdim+1] + points[i*pointdim+2]*points[i*pointdim+2]) > 30.0) continue;
              if (sqrt(points[i*pointdim+0]*points[i*pointdim+0] + points[i*pointdim+1]*points[i*pointdim+1] + points[i*pointdim+2]*points[i*pointdim+2]) < 3.0) continue;
              if (!(i %10 == 0) ) continue;
                T *p = new T[pointdim];
                p[0] = points[i*pointdim+0];
                p[1] = points[i*pointdim+1];
                p[2] = points[i*pointdim+2];
                //p[3] = points[i*pointdim+3];//timestamp
                p[3] = linenr;
                if (hasreflectance) p[idxreflectance] = points[i*pointdim + idxreflectance]; 
                transform3(SC, p);
                transform3(transMat, p);
            /*
                transform3(tmat, p);
                p[0] -= 4365204.56138761;
                p[1] -= 5527039.80752574;
                p[2] -= 275.135539519947;*/
                globalpoints.push_back(p);
//                cerr << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << linenr << endl;
            }
            if (!useOriginalPoints) { // we read points, and we dont store them, so delete
                delete[] points;
            }
        } while(true);

            cout << "MIN " << fl << "   +   " << lc << endl;
    }
    
        
    template <typename T>
    void getNextGlobal(vector<T *> &globalpoints, double *scaling, double *calibration, int count = 1 ) {
      int lc = 0;
      int fl = -1;

      // scaling matrix for slam6d coordinate system
      double Sinv[16];
      M4inv(scaling, Sinv);

      double SC[16];
      MMult(scaling, calibration, SC);

      // read points
      do {
        double *points = 0;
        uint32_t nrpoints;
        uint64_t linenr;

        bool result = mydeserialize(points, nrpoints, linenr); // 
        if (!result) {
          delete[] points;
          break; 
        }
        if (fl < 0) fl = linenr; // remember first line nr
        // skip lines for which there is no pose info (no global points can be computed)
        if (linenr < minpose ) {
          delete[] points;
          continue;;
        }
        // stop at end of file or when count lines have been read already
        if (linenr > maxpose ) {
          delete[] points;
          cout << globalpoints.size() << endl;
          break;
        }
        // TODO, this case shouldnt be occuring
        if (poses.find(linenr) == poses.end() ) continue;
        double *pose = poses[linenr];

        double tm[16], transMat[16];
        QuatToMatrix4(pose + 4, pose + 1, tm); 

        double P[16];
        QuatToMatrix4(pose + 4, pose + 1, P); 
        double PS[16];

        MMult(P, Sinv ,PS);
        MMult(scaling, PS, transMat);

        for (unsigned int i = 0; i < nrpoints; i++) {
          T *p = new T[pointdim];
          p[0] = points[i*pointdim+0];
          p[1] = points[i*pointdim+1];
          p[2] = points[i*pointdim+2];
          p[3] = points[i*pointdim+3];//timestamp
          if (hasreflectance) p[idxreflectance] = points[i*pointdim + idxreflectance]; 
          transform3(SC, p);
          transform3(transMat, p);
          globalpoints.push_back(p);
        }
        delete[] points;
        lc++;
        if (lc >= count) break;
      } while(true);
    }

    template <typename T>
    inline void deleteGlobalPoints(vector<T *> &globalpoints) {
      for (unsigned int i = 0; i < globalpoints.size(); i++) {
        delete[] globalpoints[i];
      }
      globalpoints.clear();
    }

};




#endif
