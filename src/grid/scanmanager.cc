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
    double initialTransform[16];
    
    if (readInitial)
    {
	cout << "Initial Transform:" << endl;
	string initialTransformFileName = dir + "initital.frame";
	ifstream initial_in(initialTransformFileName.c_str());
	if (!initial_in.good())
	{
	    cout << "Error opening " << initialTransformFileName << endl;
	    exit(-1);
	}
	initial_in >> initialTransform;
	cout << initialTransform << endl;
    }
    
    ifstream frame_in;
    int  fileCounter = start;
    string frameFileName;
    for (;;)
    {
	if (end > -1 && fileCounter > end) break; // 'nuf read
	frameFileName = dir + "scan" + to_string(fileCounter++,3) + ".frames";
	
	frame_in.open(frameFileName.c_str());
	
	// read 3D scan
	if (!frame_in.good())
	    break; // no more files in the directory
	
	cout << "Reading Frames for 3D Scan " << frameFileName << "...";
	vector <double*> Matrices;
	vector <double*> ColMatrices;
	int frameCounter = 0;
	
	while (frame_in.good())
	{
	    frameCounter++;	 
	    double *transMatOpenGL = new double[16];
      int type;
//	    double *colourMat = new double[4];
	    //double colourMat[4];
	    
	    try
	    {
		double transMat[16];
		frame_in >> transMat >> type;
		
		// convert to OpenGL coordinate system
		double mirror[16];
		M4identity(mirror);
		mirror[10] = -1.0;
		if (readInitial)
		{
		    double tempxf[16];
		    MMult(mirror, initialTransform, tempxf);
		    memcpy(mirror, tempxf, sizeof(tempxf));
		}
		//@@@
		// memcpy(transMatOpenGL, transMat, 16*sizeof(double));
		MMult(mirror, transMat, transMatOpenGL);
	    }
	    catch (const exception &e) {   
		break;
	    }
	    // don't store the very first entry, since it's the identity matrix.
	    if (frameCounter > 1)
	    {
		Matrices.push_back(transMatOpenGL);
		//ColMatrices.push_back(colourMat);
	    }
	}
	//MetaColour.push_back(ColMatrices);
	
	metaMatrix.push_back(Matrices);
	
	/////////////////!!!!!!!!!!!!!!!!!!!!!!!!
	//@@@
	// Commented out, because its not used in this part of the source
	// and prevents the memory from beeing freed correctly
	// (at least without a rewrite of the dtor)
	/*if (fileCounter == start+1) {
	    MetaColour.push_back(ColMatrices);
	    metaMatrix.push_back(Matrices);
	}*/
	
	frame_in.close();
	frame_in.clear();
	cout << metaMatrix.back().size() << " done." << endl;
    }
    if (metaMatrix.size() == 0) 
	cerr << "ERROR: Missing or empty directory: " << dir << endl << endl;
    
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
			    reader_type scantype, int start, int end,
			    bool readInitial, int max_distance,
			    int min_distance, bool correctYAxis)
{
    // Read scans 
    Scan::readScans(scantype, start, end, inputdir,
		    max_distance, min_distance, 0);
    
    // Read frames
    readFrames(inputdir, start, end, readInitial, correctYAxis);

    if(Scan::allScans.size() != (metaMatrix.size()))
    {
	std::cout << "Frames and scans do not match!"
		  << "(" << Scan::allScans.size() << " vs. "
		  << (metaMatrix.size()) << ")" << std::endl;
	exit(1);
    }
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
const vector <double*>& scanmanager::getMatrix(int i) 
{
    return this->metaMatrix.at(i);
}
