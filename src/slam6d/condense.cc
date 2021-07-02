/**
 * @file
 * @author Fabian Arzberger, JMU, Germany.
 *
 * condense implementation
 *
 * @brief
 * This programm is a slightly modified version of exportPoints.
 * The mod supports splitting an archive into multiple scan files, instead of just exporting everything into one file.
 * Use param -S for that. If that param is not used, the behavior will mimic the one of exportPoints.
 *
 * Copyright (C) Jochen Sprickerhof
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
  bool   use_frames   = false;
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

  try {
    parse_options(argc, argv, dir, red, rand, start, end,
      maxDist, minDist, use_frames, use_xyz, use_reflectance, use_type, use_color, octree, iotype, customFilter, scaleFac,
      hexfloat, high_precision, frame, use_normals, split);
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
  if (use_frames) Scan::continueProcessing();
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
  uint seq = 0; // count subfiles
  vector<Scan*> splitscans;

  for(unsigned int i = 0; i < Scan::allScans.size(); i++)
  {

    Scan *source = Scan::allScans[i];

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
        poseout.open(posefilepath, std::ofstream::out);
        ptsout.close();
        ptsout.clear();
        ptsout.open(scanfilepath, std::ofstream::out | std::ofstream::app);

        // free filename mem
        delete[] scanfilepath;
        delete[] posefilepath;
    }
    // Now flush everything. If split was not specified, write all in one file
    if (k == split && split != -1)
    {
        // Reference index (middle) for all scans 
        int ref = (int)(0.5 * split);

        // Collect all fields of all sub-sequent scan objects
        vector<double*> pts;
        vector<float> refls; 
        vector<int> typevec;
        vector<unsigned char*> rgbs;
        vector<double*> normals;

        // transform points into coordinate system of ref 
        double tinv[16];
        double rPos[3], rPosTheta[3];
        const double* tmp = splitscans[ref]->get_transMat();
        Matrix4ToEuler(tmp, rPosTheta, rPos);
        M4inv(tmp, tinv);

        // Collecting the data...
        for (uint iter = 0; iter < splitscans.size(); iter++) {
            Scan *sscan = splitscans[iter];
            const double* transMat = sscan->get_transMat();
            int nrpts = sscan->size<DataXYZ>("xyz" + red_string);
            DataXYZ xyz(sscan->get("xyz" + red_string)); // most important
            
            // Getting all fields
            DataReflectance refl(sscan->get("reflectance" + red_string));
            DataType type(sscan->get("type" + red_string));
            DataRGB rgb(sscan->get("rgb" + red_string));
       
            // Leave out fields that have no data. i.e. less points than nrpts
            for (int j = 0; j < nrpts; j++) {
                if (refl.size() == nrpts) 
                    refls.push_back(refl[j]);
                if (typevec.size() == nrpts)
                    typevec.push_back(type[j]);
                if (rgb.size() == nrpts)
                    rgbs.push_back(rgb[j]);
                if (iotype == UOS_NORMAL)
                {
                    DataNormal normal(sscan->get("normal" + red_string));
                    double *n = new double[3];
                    n[0] = normal[j][0];
                    n[1] = normal[j][1];
                    n[2] = normal[j][2];
                    normals.push_back(n);
                }
                double *p = new double[3];
                p[0] = xyz[j][0];
                p[1] = xyz[j][1];
                p[2] = xyz[j][2];
                transform(p, transMat);
                transform(p, tinv);
                pts.push_back(p);
            }
        }
        Scan *s = new BasicScan(rPos, rPosTheta, pts);
        if (use_reflectance)
        {
            float* data = reinterpret_cast<float*>(s->create("reflectance" + red_string, 
                            sizeof(float) * refls.size()).get_raw_pointer());
            for(size_t i2 = 0; i2 < refls.size(); ++i2)
                data[i2] = refls[i2];
        }
        if (use_type)
        {
            int* data = reinterpret_cast<int*>(s->create("type" + red_string, 
                            sizeof(int) * typevec.size()).get_raw_pointer());
            for(size_t i2 = 0; i2 < typevec.size(); ++i2)
                data[i2] = typevec[i2];
        }
        if (use_color)
        {
            unsigned char** data = reinterpret_cast<unsigned char**>(s->create("rgb" + red_string, 
                            sizeof(unsigned char*) * rgbs.size()).get_raw_pointer());
            for(size_t i2 = 0; i2 < rgbs.size(); ++i2)
                data[i2] = rgbs[i2];
        } 
        if (use_normals)
        {
            double** data = reinterpret_cast<double**>(s->create("normal" + red_string, 
                            sizeof(double*) * normals.size()).get_raw_pointer());
            for(size_t i2 = 0; i2 < normals.size(); ++i2)
                data[i2] = normals[i2];
        }
        
        writeMetaScan(s, ptsout, red, use_reflectance, use_xyz, use_type,
            use_color, use_normals, high_precision, types, scaleFac);
        // TODO: interpolate path.
        if(use_xyz) {
          writeXYZPose(poseout, rPos, rPosTheta, 0.01);
        } else {
          writeUOSPose(poseout, rPos, rPosTheta);
        }

        // prepare next iteration:
        k = 0;
        seq++;
        for (int o = 0; o < pts.size(); o++)
            delete[] pts[o];
        pts.clear();
        splitscans.clear();
    }

    // CONDENSE SCANS:

    // if a scan contains no points do not include it in condensed files
    if( source->size<DataXYZ>("xyz") == 0 )
    {
        std::cout << "scan" << source->getIdentifier() << " has no points. Skipping..." << std::endl;
        continue;
    } else { 
        splitscans.push_back(source);
        std::cout << "Exporting scan" << source->getIdentifier() << ".3d" << std::endl; 
    }
    
  }

  poseout.close();
  poseout.clear();

}
