#include "scanio/framesreader.h"
#include "slam6d/scan.h"
#include "slam6d/globals.icc"

void readFramesAndTransform(std::string dir, int start, int end, int frame, bool use_pose, bool reduced)
{
  std::ifstream frame_in;
  int  fileCounter = start;
  std::string frameFileName;
  if((int)(start + Scan::allScans.size() - 1) > end) end = start + Scan::allScans.size() - 1;
  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read

    frameFileName = dir + "scan" + to_string(fileCounter++,3) + ".frames";

    const double * transMatOrig = Scan::allScans[fileCounter - start - 1]->get_transMatOrg();
    double tinv[16];
    M4inv(transMatOrig, tinv);

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
      double tfin[16];
      MMult(transMat, tinv, tfin);
      // save final pose in scan

      if(reduced) {
        Scan::allScans[fileCounter - start - 1]->transformMatrix(tinv);
        Scan::allScans[fileCounter - start - 1]->transform(transMat,Scan::INVALID);
      } else {
        Scan::allScans[fileCounter - start - 1]->transformMatrix(tfin);
        Scan::allScans[fileCounter - start - 1]->transformAll(transMat);
      }
    } else {
      if(reduced) {
        Scan::allScans[fileCounter - start - 1]->transformMatrix(tinv);
        Scan::allScans[fileCounter - start - 1]->transform(transMatOrig,Scan::INVALID);
      } else {
        Scan::allScans[fileCounter - start - 1]->transformAll(transMatOrig);
      }
    }
    frame_in.close();
    frame_in.clear();
  }
}


