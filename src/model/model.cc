/**
 * @file model.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 12 Feb 2012
 *
 */

//==============================================================================
//  Defines
//==============================================================================


//==============================================================================
//  Includes
//==============================================================================
#include "model/scene.h"
#include "model/util.h"
#include "model/graphicsAlg.h"

#include "slam6d/io_utils.h"
#include "slam6d/io_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif
#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include <string>
#include <iostream>
using namespace std;

//==============================================================================
//  Global Variables
//==============================================================================
model::Scene* scene = NULL;             // the scene containing all data
bool quiet = false;                     // no output will be shown

//==============================================================================
//  Functions
//==============================================================================
/**
 * Prints the usage of the program to stdout.
 * @param programName the name of the current program
 */
void printUsage(const string& programName) {
#ifndef _MSC_VER
    const string bold("\033[1m");
    const string normal("\033[m");
#else
    const string bold("");
    const string normal("");
#endif

    cout << endl
	    << bold << "USAGE " << normal << endl
	    << "\t" << programName << " [options] directory" << endl << endl;

    cout << bold << "OPTIONS" << normal << endl
	    << bold << "\t -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	    << "\t\t using shared library F for input" << endl
	    << "\t\t (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rxp, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply})" << endl
	    << endl
	    // TODO enable the usage of all algos, use rht for now
	    << bold << "\t -p" << normal << " P, " << bold << "--plane=" << normal << "P" << endl
	    << "\t\t using algorithm P for plane detection" << endl
	    << "\t\t (chose P from {rht, sht, pht, ppht, apht, ran})" << endl
	    << endl
	    << bold << "\t -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	    << "\t\t start at scan NR (i.e., neglects the first NR scans)" << endl
	    << "\t\t [ATTENTION: counting naturally starts with 0]" << endl
	    << endl
	    << bold << "\t  -S, --scanserver" << normal << endl
	    << "\t\t Use the scanserver as an input method and handling of scan data" << endl
	    << endl
	    << bold << "\t -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
	    << "\t\t end at scan NR (i.e., neglects the scans following NR)" << endl
	    << endl;
    
    exit(EXIT_SUCCESS);
}

/**
 * Parses program arguments.
 */
void parseArgs(int argc, char* argv[],
			string& dir, int& start, int& end, IOType &type, PlaneAlgorithm& alg,
			bool& scanserver)
{
    char c;
    int index;

    // from unistd.h:
    extern char *optarg;
    extern int optind;
  
    WriteOnce<IOType> w_type(type);
    WriteOnce<int> w_start(start), w_end(end);

    // define the known flags in an option structure
    static struct option long_options[] = {
            {"start",       required_argument,  0,  's'},
            {"end",         required_argument,  0,  'e'},
            {"format",      required_argument,  0,  'f'},
            {"plane",       required_argument,  0,  'p'},
            {"help",        no_argument,        0,  'h'},
		  {"scanserver",  no_argument,        0,  'S'},
            {"quiet",       no_argument,        0,  'q'}
    };

    // loop over the arguments
    while ((c = getopt_long(argc, argv, "s:e:f:p:hq", long_options, &index)) != -1) {
        switch (c) {
        case 's':
            w_start = atoi(optarg);
		  if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
            break;
        case 'e':
            w_end = atoi(optarg);
		  if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
		  if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
            break;
        case 'f':
		  try {
              w_type = formatname_to_io_type(optarg);
            } catch (...) { // runtime_error
              cerr << "Format " << optarg << " unknown." << endl;
              abort();
            }
            break;
        case 'p':
            if (strcasecmp(optarg, "rht")  == 0) alg = RHT;
            else if (strcasecmp(optarg, "sht")  == 0) alg = SHT;
            else if (strcasecmp(optarg, "pht")  == 0) alg = PHT;
            else if (strcasecmp(optarg, "ppht") == 0) alg = PPHT;
            else if (strcasecmp(optarg, "apht") == 0) alg = APHT;
            else if (strcasecmp(optarg, "ran")  == 0) alg = RANSAC;
            else printUsage(argv[0]);
            break;
        case 'q':
            quiet = true;
            break;
	   case 'S':
		  scanserver = true;
		  break;
        case 'h':
        case '?':
            printUsage(argv[0]);
            break;
        default:
            throw runtime_error("getopt_long detected invalid flag");
            break;
        }
    }

    if (optind != argc-1) {
	 cerr << "\n*** Directory missing ***" << endl;
	 printUsage(argv[0]);
    }
    dir = argv[optind];

#ifndef _MSC_VER
    if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
    if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  
    parseFormatFile(dir, w_type, w_start, w_end);

}

/**
 * Cleanup function that deallocates memory and cleans up other stuff...
 */
void cleanup() {
    if (!quiet) cout << endl << "== Cleaning up..." << endl;

    if (scene != NULL) {
        delete scene;
    }

    if (!quiet) cout << endl << "== Exiting..." << endl;
}

//==============================================================================
// Main
//==============================================================================
int main(int argc, char* argv[]) {
    // seed the RNG
    srand(time(NULL));

    // default values for possible parse parameters
    IOType type        = UOS;
    bool scanserver    = false;

    // beginning and ending scan number
    int start          = 0;
    int end            = 0;

    // path to directory containing the scans
    string dir;

    // minimum and maximum scan distances
    int maxDist        = -1;
    int minDist        = -1;

    int octree         = 1;
    double red         = -1.0;

    // which plane algorithm to perform
    PlaneAlgorithm alg = RHT;

    // parse the command line arguments before registering and signals and atexit callbacks
    parseArgs(argc, argv,
		    dir, start, end, type, alg,
		    scanserver);

    // register cleanup function to deallocate memory at exit
    atexit(cleanup);

    if (!model::fileExists(dir)) {
        throw runtime_error("directory " + dir + " does not exist");
    }

    // Prepare to create the scene.
    model::Point3d pt(+78.4556, +43.0196, -181.107);
//    model::Point3d pt(0.0, 0.0, 0.0);
    model::Rotation3d rot(0.0, 0.0, 0.0);
    model::Pose6d pose(pt, rot);
    vector<model::Pose6d> poses;
    poses.push_back(pose);

    // Allocate memory for scene object.
    scene = new model::Scene(type, start, end, dir, scanserver,
					    maxDist, minDist, alg, octree, red, poses);
    scene->detectWalls();
    scene->applyLabels(scene->walls[0]);
    scene->applyLabels(scene->walls[1]);
    scene->applyLabels(scene->walls[2]);
    scene->applyLabels(scene->walls[3]);
    scene->applyLabels(scene->ceiling);
    scene->applyLabels(scene->floor);

    vector<model::CandidateOpening> openings;
    scene->addFinalOpenings(scene->walls[0], openings);
    scene->correct(scene->walls[0], openings);
    scene->addFinalOpenings(scene->walls[1], openings);
    scene->correct(scene->walls[1], openings);
    scene->addFinalOpenings(scene->walls[2], openings);
    scene->correct(scene->walls[2], openings);
    scene->addFinalOpenings(scene->walls[3], openings);
    scene->correct(scene->walls[3], openings);
    scene->addFinalOpenings(scene->ceiling, openings);
    scene->correct(scene->ceiling, openings);
    scene->addFinalOpenings(scene->floor, openings);
    scene->correct(scene->floor, openings);

    scene->writeCorrectedWalls(dir);
    scene->writeModel(dir);

    return 0;
}
