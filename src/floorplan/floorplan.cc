//==============================================================================
// Includes.
//==============================================================================
// C includes.
#include <stdlib.h>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

// C++ includes.
#include <iostream>
using namespace std;

// User includes.
#include "floorplan/util.h"
#include "slam6d/io_utils.h"
#include "slam6d/io_types.h"

#include "floorplan/scene.h"
#include "floorplan/floorPlan.h"
using namespace floorplan;

//==============================================================================
//  Global Variables
//==============================================================================
bool quiet = false;     // no output will be shown
Scene *scene = NULL;

//==============================================================================
// Helper functions.
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
            << bold << "\t -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
            << "\t\t start at scan NR (i.e., neglects the first NR scans)" << endl
            << "\t\t [ATTENTION: counting naturally starts with 0]" << endl
            << endl
            << bold << "\t -S, --scanserver" << normal << endl
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
        string& dir, int& start, int& end, IOType &type,
        bool& scanserver)
{
    char c;
    int index;

    // From unistd.h.
    extern char *optarg;
    extern int optind;

    WriteOnce<IOType> wType(type);
    WriteOnce<int> wStart(start), wEnd(end);

    // Define the known flags in an option structure.
    static struct option long_options[] = {
            {"start",       required_argument,  0,  's'},
            {"end",         required_argument,  0,  'e'},
            {"format",      required_argument,  0,  'f'},
            {"help",        no_argument,        0,  'h'},
            {"scanserver",  no_argument,        0,  'S'},
            {"quiet",       no_argument,        0,  'q'}
    };

    // Loop over the arguments.
    while ((c = getopt_long(argc, argv, "s:e:f:hq", long_options, &index)) != -1) {
        switch (c) {
        case 's':
            wStart = atoi(optarg);
            if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
            break;
        case 'e':
            wEnd = atoi(optarg);
            if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
            if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
            break;
        case 'f':
            try {
                wType = formatname_to_io_type(optarg);
            } catch (...) { // runtime_error
                cerr << "Format " << optarg << " unknown." << endl;
                abort();
            }
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

    parseFormatFile(dir, wType, wStart, wEnd);

}

/**
 * Cleanup function that deallocates memory and cleans up other stuff...
 */
void cleanup() {
    if (!quiet) cout << endl << "== Cleaning up..." << endl;

    // Clean up stuff here.

    if (!quiet) cout << endl << "== Exiting..." << endl;
}

//==============================================================================
// Main.
//==============================================================================
int main(int argc, char* argv[]) {
    if (!quiet) cout << endl << "== Starting..." << endl;

    // Seed the RNG.
    srand(time(NULL));

    // Default values for possible parse parameters.
    IOType type        = UOS;
    bool scanserver    = false;

    // Beginning and ending scan number.
    int start          = 0;
    int end            = 0;

    // Path to directory containing the scans.
    string dir;

    // Minimum and maximum scan distances.
    int maxDist        = -1;
    int minDist        = -1;

    int octree         = 1;
    double red         = -1.0;


    // Parse the command line arguments before registering and signals and atexit callbacks.
    parseArgs(argc, argv,
            dir, start, end, type,
            scanserver);

    // Register cleanup function to deallocate memory at exit.
    atexit(cleanup);

    if (!floorplan::fileExists(dir) || !floorplan::fileIsDir(dir)) {
        throw runtime_error("directory " + dir + "does not exist");
    }

    // TODO add to parse args
    maxDist = 700;
    scene = new Scene(type, start, end, dir, scanserver, maxDist, minDist, octree, red);

    double resolution = 2.0;
    cv::Mat hMap = scene->computeHorizontalHist(resolution, false, false);
    FloorPlan plan(hMap, resolution);

    return 0;
}
