/*
 * 2DGridder implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/scanGrid.h"
#include "grid/scanToGrid.h"
#include "slam6d/scan.h"
#include "grid/scanmanager.h"
#include "grid/parcelmanager.h"
#include "grid/gridWriter.h"
#include "grid/viewpointinfo.h"
#include "grid/gridlines.h"
#include "slam6d/globals.icc"

#include "grid/Debug.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <dirent.h>

using std::cerr;
using std::flush;

#ifdef _MSC_VER
  #define  _USE_MATH_DEFINES
  #include <windows.h>
  #include "GL/glut.h"    /* Header File For The glu toolkit */
  #include "XGetopt.h"
#else
  #include <getopt.h>
  #include <strings.h>
#endif

/**
 * 2DGridder
 *
 * Main class to convert a 3D scan to a 2D grid
 *
 * @author Andre Schemschadt, Uwe Hebbelmann, Sebastian Stock
 * @date 20.02.2008
 */

/**
 * Explains the usage of this program's command line parameters
 *
 * @param prog name of the program
 */
void usage(char* prog)
{
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif
  
    cout << endl
	 << "Usage: " << prog << endl
	 << "       [-s NR] [-e NR] [-m NR] [-M NR] [-f F] [-o DIR] [-t] " << endl
	 << "       [-h NR] [-H NR] [-r NR] [-w] [-p NR] [-P Nr] [-y] [-n] " <<endl
	 << "       [-g] [-d] [-l] [-a NR] inputdirectory" << endl << endl;
    
    cout << "  -s NR   start at scan NR (i.e., neglects the first NR scans)" << endl
	 << "          [ATTENTION: counting starts with 0]" << endl
	 << "  -e NR   end after scan NR" << "" << endl
	 << "  -m NR   set the maximal range distance to NR 'units' (unit of scan data, e.g. cm)" << endl
	 << "          (default 2500, unit of scan data, e.g. cm)" << endl
	 << "  -M NR   set the minimal range distance to NR 'units' (unit of scan data, e.g. cm)" << endl
	 << "  -f F    format = F" << endl
	 << "          using shared library F for input" << endl
	 << "          (chose F from {uos, uos_map, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_bin, riegl_txt, zahn, ply})" << endl
	 << "  -o DIR  the output directory (if none is set use the input directory" << endl 
	 << "  -t      read a file containing a initial transformation matrix" << endl
	 << "  -H NR   set the maximal relevant height to NR 'units' (unit of scan data, e.g. cm)" << endl
	 << "  -h NR   set the minimal relevant height to NR 'units' (unit of scan data, e.g. cm)" << endl
	 << "  -r NR   the width of the gridunit (unit of scan data, e.g. cm per unit)" << endl
	 << "  -w      default true, if set false " << endl
	 << "          (if true free points between viewpoint and points will be created)" << endl
	 << "  -p NR   set the parcel width (unit of scan data, e.g. cm)" << endl
	 << "  -P NR   set the parcel height (unit of scan data, e.g. cm)" << endl
	 << "  -y      default false, if set true " <<endl
	 << "          (if true the transformationmatrix of the scans will be corrected (the value for Y)" << endl
	 << "  -n      default true, if set false" << endl
	 << "          (if true, neighbours will be weighted) " << endl
	 << "  -a NR   the spot diameter of the laser per 3000 units of scan data, e.g. cm " << endl
	 << "          (default 15 cm / 3000 cm) " << endl
	 << "  -g      default false, if set true " << endl
	 << "          (if true, the grids will be written) " << endl
	 << "  -d      default true, if set false " << endl
	 << "          (if true, the world will be written) " << endl
	 << "  -l      default false, if set true " << endl
	 << "          (if true, lines will be created and written) " << endl
	 << "  -i      default false, if set true " << endl
	 << "          (if true, a world ppm will be created) " << endl
	 << "  -c      default 50. This is the numbers of scans which " << endl
	 << "          will be process at a time" << endl
	 << "  -R      default false, if set the programm will resume " << endl
	 << endl << endl;

    exit(1);
}

/**
 * Checks if the directory exists
 * @param path the directory to check
 * @return true if exists, else false
 */
bool directoryExists(const string path )
{
    DIR *pDir;
    bool bExists = false;
 
    pDir = opendir(path.c_str());

    if (pDir != NULL){
	bExists = true;
	closedir (pDir);
    }
 
    return bExists;
}

/**
 * A function that parses the command-line arguments and sets the respective flags.
 *
 * @param argc the number of arguments
 * @param argv the arguments
 * @param inputdir parsing result - the input directory
 * @param outputdir parsing result - the output directory
 * @param start parsing result - starting at scan number 'start'
 * @param end parsing result - stopping at scan number 'end'
 * @param maxDist parsing result - maximal distance
 * @param minDist parsing result - minimal distance
 * @param readInitial parsing result -  read a file containing a initial transformation matrix
 * @param type parsing result - file format to be read
 * @param minHeight the minimal relevant height
 * @param maxHeight the maximal relevant height
 * @param resolution the width of the gridunit (centimeters per unit)
 * @param waypoints default true, if set false (if true free points between viewpoint and points will be created)
 * @param parcel_width the width of the parcel
 * @param parcel_height the height of the parcel
 * @param correctY default true, if set false (if true the transformationmatrix of the scans will be corrected (the value for Y)
 * @return 0, if the parsing was successful, 1 otherwise 
 */
int parseArgs(int argc, char **argv,
	      string &inputdir, string &outputdir,
	      int& start, int& end, int& maxDist, int& minDist,
	      bool &readInitial, IOType &type, bool& correctY,
	      double &minHeight, double &maxHeight, long &resolution,
	      bool &waypoints, bool &neighbours,
	      int &parcelWidth, int &parcelHeight,
	      bool &writeWorld, bool &writeLines, bool &writeGrids,
	      int &spotradius, bool &writeWorldppm, int& count,
	      bool &resume)
{
    int  c;
    
    // from unistd.h
    extern char *optarg;
    extern int optind;
    
    cout << endl;
    while ((c = getopt (argc, argv, "o:s:a:e:m:ncwgidlRM:h:H:f:r:p:P:yt")) != -1)
    {
      switch (c)
      {
        case 'o':
          outputdir = optarg;
          if (!directoryExists(outputdir)) {
            cerr << "Error: Directory doesn't exist.\n";
            exit(1);
          }
          break;
        case 's':
          start = atoi(optarg);
          if (start < 0) { 
            cerr << "Error: Cannot start at a negative scan number.\n"; 
            exit(1); 
          }
          break;
        case 'e':
          end = atoi(optarg);
          if (end < 0) { 
            cerr << "Error: Cannot end at a negative scan number.\n"; 
            exit(1); 
          }
          if (end < start) { 
            cerr << "Error: <end> cannot be smaller than <start>.\n"; 
            exit(1); 
          }
          break;
        case 'c':
          count = atoi(optarg);
          if( count < 1){
            cerr << "Error: <count> must be greater than 1.\n";
            exit(1);
          }
          break;
        case 'R':
          resume = true;
          break;
        case 'm':
          maxDist = atoi(optarg);
          break;
        case 'M':           
          minDist = atoi(optarg);
          break;
        case 't':
          readInitial = true;
          break;
        case 'f':
          try {
            type = formatname_to_io_type(optarg);
          } catch (...) { // runtime_error
            cerr << "Format " << optarg << " unknown." << endl;
            abort();
          }
          break;
        case 'H':
          maxHeight = atoi(optarg);
          break;
        case 'h':
          minHeight = atoi(optarg);
          break;
        case 'g':
          writeGrids = true;
          break;	  
        case 'r':
          resolution = atol(optarg);
          if (resolution < 1) { 
            cerr << "Error: <resolution> cannot be smaller than 1.\n"; 
            exit(1); 
          }
          break;	
        case 'w':
          waypoints = false;
          break;
        case 'n':
          neighbours = false;
          break;
        case 'p':
          parcelWidth = atol(optarg);
          if (parcelWidth < 1) { 
            cerr << "Error: <parcel_width> cannot be smaller than 1.\n"; 
            exit(1); 
          }
          break;
        case 'P':
          parcelHeight = atol(optarg);
          if (parcelHeight < 1) { 
            cerr << "Error: <parcel_height> cannot be smaller than 1.\n"; 
            exit(1); 
          }
          break;
        case 'y':
          correctY = true;
          break;
        case 'a':
          spotradius = atoi(optarg);
          if (spotradius < 0) { 
            cerr << "Error: <spotradius> cannot be smaller than 0.\n"; 
            exit(1); 
          }
          break;
        case 'd':
          writeWorld = false;
          break;
        case 'l':
          writeLines = true;
          break;
        case 'i':
          writeWorldppm = true;
          break;
        case '?':
          usage(argv[0]);
          return 1;
        default:
          abort();
      }}
    
    if (optind != argc-1) {
	cerr << "\n*** Input directory missing ***" << endl;
	usage(argv[0]);
    }
    inputdir = argv[optind];
  
    //If no output directory is set, set it to the input directory
    if (outputdir == "") outputdir = inputdir;  

#ifndef _MSC_VER
  if (inputdir[inputdir.length()-1] != '/') inputdir = inputdir + "/";
  if (outputdir[outputdir.length()-1] != '/') outputdir = outputdir + "/";
#else
  if (inputdir[inputdir.length()-1] != '\\') inputdir = inputdir + "\\";
  if (outputdir[outputdir.length()-1] != '\\') outputdir = outputdir + "\\";
#endif

    return 0;
}
  
/**
 * Main function.
 * Reads the scan (scan000.3d, ...) and frames files (scan000.frames, ...) from the data directory.
 * The frames are used for animation of the matching process. Converts the 3D scan data to a 2D grid map
 * and stores it
 * 
 * @param argc count of the command-line arguments
 * @param argv command-line arguments
 */
int main(int argc, char **argv){
    // Start message
    cout << "(c) University of Osnabrueck, 2006 - 2008" << endl << endl
	 << "Restricted Usage" << endl
	 << "Don't use without permission" << endl;


    // Usage
    if(argc <= 1){
	usage(argv[0]);
    }

    ///////////////////////////////////////
    // Defaultvalues
    string scandir = "";
    string outputdir = "./parcels/";

    int start = 0;
    int end = -1;
    int maxDistance = 2500;
    int minDistance = -1;
    bool readInitial = false;
    IOType scantype  = UOS;
    bool correctY = false;

    double maxRelevantHeight = 50;
    double minRelevantHeight = 2;
    long resolution = 10;
    bool createWaypoints = true;
    bool createNeighbours = true;

    int parcelWidth = 1000;
    int parcelHeight = 1000;

    int spotradius = 15;

    bool writeWorld = true;
    bool writeLines = false;
    bool writeGrids = false;
    bool writeWorldppm = false;

    double isSolidPoint = 0.2;
    int count = 50;
    bool resume = false;
    ///////////////////////////////////////


    // parses the command-line arguments and sets the respective flags
    parseArgs(argc, argv,
	      scandir, outputdir, start, end, maxDistance, minDistance,
	      readInitial, scantype, correctY,
	      minRelevantHeight, maxRelevantHeight, resolution,
	      createWaypoints, createNeighbours,
	      parcelWidth, parcelHeight,
	      writeWorld, writeLines, writeGrids, spotradius, writeWorldppm,
	      count, resume);

    // calculate parcel width and height
    parcelWidth /= resolution;
    parcelHeight /= resolution;

    // create parcelmanager
    cout << "Create parcelmanager ..." << endl;
    parcelmanager parcelman(parcelWidth, parcelHeight,
			    outputdir, resolution, resume);

    cout << "Create viewpointlist ... " << endl;
    viewpointinfo viewpoint(outputdir);
    
    for (int i = start; i <= end; i+=count)
    {
	int endloop = i + count - 1 < end ? i + count - 1 : end;

	// get Scans
	cout << "Reading scans " << i << " to " << endloop << " ... ";
	scanmanager scanman;
	scanman.startscan(scandir, outputdir, scantype, i, endloop,
				   readInitial, maxDistance, minDistance,
				   correctY);

	cout << "Done."<< endl;
		
	// create grid from scans
	cout << "Creating grids " << i << " to " << endloop << " ... ";
	scanToGrid stg(resolution,
		       minRelevantHeight,
		       maxRelevantHeight,
		       maxDistance,
		       spotradius,
		       createWaypoints,
		       createNeighbours);

	cout << "Done."<< endl;
	
	// convert scans
	cout << "Converting " << scanman.getScanCount() <<" scans ... ";
	
	vector<scanGrid*> grids;
	for(size_t j = 0; j < scanman.getScanCount(); j++)
	{
	    cout << "." << flush;
	    double* p = scanman.getMatrix(j).back();
	    scanman.getScan(j).transformAll(p);
	    grids.push_back(stg.convert(scanman.getScan(j), p));
	}
	cout << "Done." << endl;

	// start writing
	cout << "Processing from " << i << " to " << endloop << endl;	
	for(size_t j = 0; j < grids.size(); ++j)
	{
	    cout << "Adding scan " << i << " ... " << flush; 
	    parcelman.addGrid(grids[j],
			      grids[j]->getViewpointX(),
			      grids[j]->getViewpointZ());
	   
	    viewpoint.addGrid(grids[j]);
	    cout << "Done."<< endl;
	}

	// print grids for each scan if wished
	if(writeGrids)
	{
	    cout << "Writing grids " << i
		 << " to " << endloop << " ... " << endl;

	    string str;
	    stringstream stream;
	    for(size_t j = 0; j < grids.size(); ++j)
	    {
	      cout << "Writing grid " << i << " ... " << flush;
		stream.clear();
		stream << outputdir << "grid" << j+i << ".ppm ";
		stream >> str;
		
		ppmWriter writer(str);
		writer.write(*grids[j]);
		cout << "Done."<<endl;
	    }
	}	


	cout << "Freeing subdate ... "<< flush;
	for(size_t j = 0; j < grids.size(); ++j)
	    delete grids[j];
	grids.clear();
	cout << "Done."<< endl;
    }
	
    // write the world and the viewpoints
    if(writeWorld)
    {
	// write world map
	cout << "Writing world map ... " << flush;
	parcelman.writeWorld("world.2dm");
	cout << "done." << endl;

	// write viewpoints
	cout << "Writing viewpoints ... " << flush; 
	viewpoint.write("viewpoints.pts");
	cout << "Done." << endl;
    }
        
    // write gridlines
    if(writeLines || writeWorldppm)
    {	
	cout << "Creating world grid ... ";
	grid *g = parcelman.createWorldGrid();
	cout << "Done." << endl;

	if(writeLines) {
	    cout << "Writing Lines ... ";
	    gridlines glines(g, maxDistance, isSolidPoint);
	    glines.writeLin(outputdir + "/lines.plot");
	    cout << "Done." << endl;
	}

	if(writeWorldppm) {
	    cout << "Writing world ppm ... ";
	    ppmWriter writer(outputdir + "world.ppm");
	    writer.write(*g);
	    cout << "Done." << endl;
	}
	
	delete g;
    }

    cout << "Freeing data ... " << endl;
} 
