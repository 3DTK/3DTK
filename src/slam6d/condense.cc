/**
 * @file
 * @author Fabian Arzberger, JMU, Germany.
 * @author Jochen Sprickerhof, JMU, Germany.
 *
 * condense implementation
 *
 * @brief
 * This programm is a modified version of exportPoints.
 * The mod supports splitting an archive into multiple scan files, instead of just exporting everything into one file.
 * Use param -S for that. If that param is not used, the behavior will mimic the one of exportPoints.
 * If the param is used, put S scans together, thus creating metascans of size S.
 * Use the atomize (atomize.cc) program to re-apply any optimizations back onto the indivudial frames.
 * Further, the program will not export "points.txt, poses.txt, and positions.txt",
 * but export directly into scanXXX.3d and scanXXX.pose files.
 *
 * Copyright (C) Fabian Arzberger, Jochen Sprickerhof
 * Modifications made by Fabian Arzberger, JMU
 * Released under the GPL version 3.
 *
 */

#include "slam6d/condense.h"

int main(int argc, char **argv)
{
  // parsing the command line parameters
  // init, default values if not specified
  std::string dir;
  double red   = -1.0;
  int    rand  = -1;
  int    start = 0,   end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  bool   trustpose   = false;
  bool   use_xyz = false;
  bool   use_color = false;
  bool   use_reflectance = false;
  bool   use_type = false;
  bool   use_normals = false;
  int octree       = 0;  // employ randomized octree reduction?
  IOType iotype    = UOS;
  bool rangeFilterActive = false;
  bool customFilterActive = false;
  std::string customFilter;
  double scaleFac = 0.01;
  bool hexfloat = false;
  bool high_precision = false;
  int frame = -1;
  int split = -1;
  bool global = false;
  bool rm_scatter = false;
  bool skip_empty = false;

  try {
    parse_options(argc, argv, dir, red, rand, start, end,
      maxDist, minDist, trustpose, use_xyz, use_reflectance, use_type, use_color, octree, iotype, customFilter, scaleFac,
      hexfloat, high_precision, frame, use_normals, split, global, rm_scatter, skip_empty);
  } catch (std::exception& e) {
    std::cerr << "Error while parsing settings: " << e.what() << std::endl;
    exit(1);
  }

  if(!supportsNormals(iotype) && use_normals) {
    std::cerr << "WARNING File format does not support normals. Normals are not exported" << std::endl;
    use_normals = false;
  }

  std::string red_string = red > 0 ? " reduced" : "";
  rangeFilterActive = minDist > 0 || maxDist > 0;
  if (rangeFilterActive) {
    cout << "MinDist: " << minDist << ", MaxDist: " << maxDist << endl;
  }
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
  if (!trustpose){
    Scan::continueProcessing();
    std::cout << "ATTENTION: using .frames files!!!" << std::endl;
  } else {
    std::cout << "ATTENTION: using .pose files!!!" << std::endl;
  }
  Scan::openDirectory(false, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    std::cerr << "No scans found. Did you use the correct format?" << std::endl;
    exit(-1);
  }

  unsigned int types = PointType::USE_NONE;
  if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
  if(supportsColor(iotype)) types |= PointType::USE_COLOR;
  if(supportsType(iotype)) types |= PointType::USE_TYPE;

  // if specified, filter scans
  for (size_t i = 0; i < Scan::allScans.size(); i++)  {
     if(rangeFilterActive) Scan::allScans[i]->setRangeFilter(maxDist, minDist);
     if(customFilterActive) Scan::allScans[i]->setCustomFilter(customFilter);
  }

  // Apply reduction
  cout << "Applying reduction... " << endl;
  if (rm_scatter && red == -1
   || octree != 1 && red == -1 )
   {
    cout << "Conflicting reduction options found. " << endl;
    cout << "Use -r <arg> to set voxelsize. Use -O <arg> to specify nr. of pts. in a voxel." << endl;
    cout << "Use -d to delete the voxels with fewer pts. than specified with -O." << endl;
   }
  int end_reduction = (int)Scan::allScans.size();
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int iterator = 0; iterator < end_reduction; iterator++) {
    if (red > 0) {
      PointType pointtype(types);
      std::cout << "Reducing Scan No. " << iterator << std::endl;
      Scan::allScans[iterator]->setReductionParameter(red, octree, pointtype);
      Scan::allScans[iterator]->calcReducedPoints(rm_scatter);
    }
    // reduction filter for current scan!
  }

  // creating subdirectory for condensed scans
  std::string save_dir = dir + "cond/";
  if ( !existsDir( save_dir.c_str() ) )
  {
   boost::filesystem::create_directory(save_dir);
   std::cout << "Creating \"" << save_dir << "\"." << std::endl;
  } else std::cout << save_dir << " exists allready." << std::endl;

  // declare files
  std::ofstream ptsout;
  std::ofstream poseout;
  char* scanfilepath;
  char* posefilepath;

  int k = 0; // count subscans
  int seq = 0; // count subfiles
  vector<Scan*> splitscans;

  for(unsigned int i = 0; i < Scan::allScans.size(); i++)
  {

    Scan *source = Scan::allScans[i];
    // if a scan contains no points do not include it in condensed files
    if( source->size<DataXYZ>("xyz") == 0 && skip_empty)
    {
        std::cout << "scan" << source->getIdentifier() << " has no points. Skipping..." << std::endl;
        continue;
    } else {
        splitscans.push_back(source);
        std::cout << "Exporting scan" << source->getIdentifier() << ".3d" << std::endl;
    }

    // open new file after 'split' scans
    if (0 == k++)
    {
        // initializing filenames
        scanfilepath = new char[1000]; // hardcoded buffer of size 1000
        posefilepath = new char[1000]; // ugly, I know. But it works
        sprintf(scanfilepath, "%sscan%03d.3d", save_dir.c_str(), seq);
        sprintf(posefilepath, "%sscan%03d.pose", save_dir.c_str(), seq);
        cout << "Creating scanfile " << scanfilepath << endl;
        poseout.close();
        poseout.clear();
        // Truncate the files (overwrite mode)
        poseout.open(posefilepath, std::ofstream::out | std::ofstream::trunc);
        ptsout.close();
        ptsout.clear();
        ptsout.open(scanfilepath, std::ofstream::out | std::ofstream::trunc);

        // free filename mem
        delete[] scanfilepath;
        delete[] posefilepath;
    }
    // Now flush everything. If split was not specified, write all in one file
    if (k == split && split != -1)
    {
        // Reference index (middle) for all scans
        int ref = 0;//(int)(0.5 * split);

        Scan *s = createMetaScan(splitscans, iotype, ref, red_string,
            use_reflectance, use_type, use_color, use_normals, global);
        const double* rPos = s->get_rPos();
        const double* rPosTheta = s->get_rPosTheta();

        writeMetaScan(s, ptsout, red, use_reflectance, use_xyz, use_type,
            use_color, use_normals, high_precision, types, scaleFac);

        if(use_xyz) {
          writeXYZPose(poseout, rPos, rPosTheta, 0.01);
        } else {
          writeUOSPose(poseout, rPos, rPosTheta);
        }

        // prepare next iteration:
        k = 0;
        seq++;
        splitscans.clear();
    }

  }

   // Flush the rest.
  if (!splitscans.empty())
  {
    int ref = 0;//(int)(0.5 * splitscans.size());

    Scan *s = createMetaScan(splitscans, iotype, ref, red_string,
        use_reflectance, use_type, use_color, use_normals, global);
    const double* rPos = s->get_rPos();
    const double* rPosTheta = s->get_rPosTheta();

    writeMetaScan(s, ptsout, red, use_reflectance, use_xyz, use_type,
        use_color, use_normals, high_precision, types, scaleFac);
    // TODO: interpolate path over all metascans vector<Scan*>
    // edit: TODO finished. use "atomize" for that.
    if(use_xyz) {
      writeXYZPose(poseout, rPos, rPosTheta, 0.01);
    } else {
      writeUOSPose(poseout, rPos, rPosTheta);
    }
  }

  poseout.close();
  poseout.clear();
  return 0;
}
