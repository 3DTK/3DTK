/*
 * scanmanager implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/scanmanager.h"
#include <cstring>
#include "slam6d/globals.icc"
#include <stdexcept>
using std::exception;
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

/**
 * Ctor.
 * Does nothing
 */
scanmanager::scanmanager()
{
}

/**
 * DTor.
 * Clear the memory
 */
scanmanager::~scanmanager()
{
    // Deleting the stored metamatrixes
    for (size_t i = 0; i < metaMatrix.size(); i++) {
	for (size_t j = 0; j < metaMatrix.at(i).size(); j++) {
	    delete[] metaMatrix.at(i).at(j);
	}
    }
       
    // Deleting the stored scans and reset the vector
    for(size_t i=0; i < Scan::allScans.size(); ++i)
    {	
	delete Scan::allScans[i];
    }
    
    Scan::allScans.clear();
}

/*
 * A function that read the .frame files created by slam6D
 * (taken from the originial test-class)
 *
 * @param dir the directory
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param readInitial read a file containing a initial transformation matrix and apply it
 * @param correctYAxis if set, value 14 of the transformationmatrix will be set to 0
 */
void scanmanager::readFrames(string dir, int start, int end,
					    bool readInitial, bool correctYAxis)
{

  // convert to OpenGL coordinate system
  double mirror[16];
  M4identity(mirror);
  mirror[10] = -1.0;
  
  double initialTransform[16];
  if (readInitial) {
    cout << "Initial Transform:" << endl;
    string initialTransformFileName = dir + "initital.frame";
    ifstream initial_in(initialTransformFileName.c_str());
    if (!initial_in.good()) {
      cout << "Error opening " << initialTransformFileName << endl;
      exit(-1);
    }
    initial_in >> initialTransform;
    cout << initialTransform << endl;
    
    // update the mirror to apply the initial frame for all frames
    double tempxf[16];
    MMult(mirror, initialTransform, tempxf);
    memcpy(mirror, tempxf, sizeof(tempxf));
  }

  for(std::vector<Scan*>::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it) {
    const double* transformation;
    Scan::AlgoType algoType;
    std::vector<double*> Matrices;
    
    // iterate over frames (stop if none were created) and pull/convert the frames into local containers
    unsigned int frame_count = (*it)->readFrames();
    if(frame_count == 0) break;
    for(unsigned int i = 0; i < frame_count; ++i) {
      (*it)->getFrame(i, transformation, algoType);
      double* transMatOpenGL = new double[16];
      
      // apply mirror to convert (and initial frame if requested) the frame and save in opengl
      MMult(mirror, transformation, transMatOpenGL);

      Matrices.push_back(transMatOpenGL);
    }
    
    metaMatrix.push_back(Matrices);
    
    //    current_frame = MetaMatrix.back().size() - 1;
  }
  
  if (metaMatrix.size() == 0) {
    cerr << "*****************************************" << endl;
    cerr << "** ERROR: No .frames could be found!   **" << endl;
    cerr << "*****************************************" << endl;
    cerr << " ERROR: Missing or empty directory: " << dir << endl << endl;
    return;
  }
    
  // if set, value 14 of the transformationmatrix will be set to 0
  if(correctYAxis)
    {
	 for(size_t i=0; i < metaMatrix.size(); ++i)
	   {
		for(size_t j=0; j < metaMatrix.at(i).size(); ++j)
		  {
		    metaMatrix.at(i).at(j)[13] = 0;
		  }
	   }
    }
}

/**
 * Reads all scans, frames and the transformationmatrix
 * 
 * @param inputDir the input directory
 * @param outputdir the output directory
 * @param scantype the scantype
 * @param start start with scannumber 'start'
 * @param end end with scannumber 'end'
 * @param readInitial read files, that contain a 'initial transformation matrix'
 * @param max_distance the maximal distance
 * @param min_distance the minimal distance
 * @param correctYAxis if set, value 14 of the transformationmatrix will be set to 0
 */
void scanmanager::startscan(string inputdir, string outputdir,
			    IOType scantype, int start, int end,
			    bool readInitial, int max_distance,
			    int min_distance, bool correctYAxis)
{
  // Read scans

  // load all available scans
  bool scanserver = false;
  Scan::openDirectory(scanserver, inputdir, scantype, start, end);
  
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);

  }
  
  // Read frames
  readFrames(inputdir, start, end, readInitial, correctYAxis);
}

/**
 * Returns the number of scans
 *
 * @return the number of scans
 */
size_t scanmanager::getScanCount() const 
{
    return Scan::allScans.size();
}

/**
 * Returns scan with the number i
 *
 * @param i number of the scan
 * @return scan with number i
 */
Scan& scanmanager::getScan(int i)
{
    return *Scan::allScans.at(i);
}

/**
 * Returns the transformationmatrix with number i.
 *
 * The original readFrames method always read one matrix more 
 * than needed, but it complicated freeing memory and wasnt used in 
 * this programm, so it was commented out.
 *
 * @param i the number of the transformationmatrix
 * @return transformationmatrix with number i
 */
const std::vector <double*>& scanmanager::getMatrix(int i) 
{
    return this->metaMatrix.at(i);
}
