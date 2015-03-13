/**
 * @file
 * @brief Computes a transformation between two sets of points, and transforms a series of frames accordingly
 *
 * @author Florian Leutert
 */

#ifdef _MSC_VER
#ifdef OPENMP
#ifndef _OPENMP
#define _OPENMP
#endif
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#define WANT_STREAM ///< define the WANT stream :)

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;

#include "slam6d/icp6Dsvd.h"
#include "slam6d/globals.icc"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#if WIN32
#define snprintf sprintf_s
#else
#ifndef MAX_PATH
#define MAX_PATH 255
#endif 
#endif 


/**
 * Explains the usage of this program's command line parameters
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
        << bold << "USAGE " << normal << endl
        << "   " << prog << " [parameters] directory" << endl << endl;
    cout << bold << "PARAMETERS" << normal << endl

        << endl
        << bold << "  -n" << normal << " NR, " << bold << "--pointNr=" << normal << "NR" << endl
        << "         number of points (= lines from input file) to use for computing transformation" << endl
        << endl
        << bold << "  -i" << normal << " STR, " << bold << "--input=" << normal << "STR" << endl
        << "         input file containing (at least 3) point pairs for computing transformation" << endl
        << "         (input file line syntax (mapping between Points p1 and p2): p1x p1y p1z p2x p2y p2z\n)" << endl
        << endl
        << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
        << "         start at frame NR (i.e., neglects the first NR frame files)" << endl
        << "         [ATTENTION: counting naturally starts with 0]" << endl
        << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
        << "         end after frame file NR" << endl
        << endl
        << bold << "  -r" << normal << ", " << bold << "--reverse" << normal << "" << endl
        << "         reverse order of input point pairs, that is, instead of transforming from pts2 to pts1," << endl
        << "         transform from pts1 to pts2" << endl
        << endl
        << bold << "  -x" << normal << ", " << bold << "--xyz" << normal << "" << endl
        << "         frame files are in xyz format (right handed coordinate system in m);" << endl
        << "         with this flag, they will be converted to the 3DTK-coordinate system" << endl
        << "         before applying the transformation, and additionaly, files with converted" << endl
        << "         input coordinates will be created " << endl
        << endl

        << endl << endl;

    cout << bold << "EXAMPLES " << normal << endl
        << "   " << prog << " -n 4 -i ptPairs.txt -s 2 -e 3 -x dat/dirWithFrameFiles" << endl << endl;
    exit(1);
}

int parseArgs(int argc, char **argv, string &dir, int &nrOfPts, string &inputFile,
    int &start, int &end, bool& inputInXYZ, bool& reverse)
{
    int  c;
    // from unistd.h:
    extern char *optarg;
    extern int optind;

    /* options descriptor */
    // 0: no arguments, 1: required argument, 2: optional argument
    static struct option longopts[] = {
        { "pointNr",    required_argument, 0, 'n' },
        { "input",      required_argument, 0, 'i' },
        { "start",      required_argument, 0, 's' },
        { "end",        required_argument, 0, 'e' },
        { "xyz",        required_argument, 0, 'x' },
        { "reverse",    required_argument, 0, 'r' },
        { 0, 0, 0, 0 }                    // needed, cf. getopt.h
    };

    cout << endl;
    while ((c = getopt_long(argc, argv, "n:i:s:e:xr", longopts, NULL)) != -1)
        switch (c)
    {
        case 'n':
            nrOfPts = atoi(optarg);
            break;
        case 's':
            start = atoi(optarg);
            if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
            break;
        case 'e':
            end = atoi(optarg);
            if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
            if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
            break;
        case 'i':
            inputFile = optarg;
            break;
        case 'x':
            inputInXYZ = true;
            break;
        case 'r':
            reverse = true;
            break;
        case '?':
            usage(argv[0]);
            return 1;
        default:
            abort();
    }

    if (optind != argc - 1) {
        cerr << "\n*** Directory missing ***" << endl;
        usage(argv[0]);
    }
    dir = argv[optind];

#ifndef _MSC_VER
    if (dir[dir.length() - 1] != '/') dir = dir + "/";
#else
    if (dir[dir.length() - 1] != '\\') dir = dir + "\\";
#endif

    return 0;
}

void matMulVec(double* mat4, double* v4, double* res) {
    res[0] = mat4[0] * v4[0] + mat4[4] * v4[1] + mat4[8] * v4[2] + mat4[12] * v4[3];
    res[1] = mat4[1] * v4[0] + mat4[5] * v4[1] + mat4[9] * v4[2] + mat4[13] * v4[3];
    res[2] = mat4[2] * v4[0] + mat4[6] * v4[1] + mat4[10] * v4[2] + mat4[14] * v4[3];
    res[3] = 1;
}

void printMat_RowOrderStyle(double *mat){
    cout << mat[0] << "  " << mat[1] << "  " << mat[2] << "  " << mat[3] << endl;
    cout << mat[4] << "  " << mat[5] << "  " << mat[6] << "  " << mat[7] << endl;
    cout << mat[8] << "  " << mat[9] << "  " << mat[10] << "  " << mat[11] << endl;
    cout << mat[12] << "  " << mat[13] << "  " << mat[14] << "  " << mat[15] << endl << endl;
}

void printMat_ColumnOrderStyle(double *mat){
    cout << mat[0] << "  " << mat[4] << "  " << mat[8] << "  " << mat[12] << endl;
    cout << mat[1] << "  " << mat[5] << "  " << mat[9] << "  " << mat[13] << endl;
    cout << mat[2] << "  " << mat[6] << "  " << mat[10] << "  " << mat[14] << endl;
    cout << mat[3] << "  " << mat[7] << "  " << mat[11] << "  " << mat[15] << endl << endl;
}

inline void to3DTKMat(double *inMatrix, double *outMatrix) {
    outMatrix[0] = inMatrix[5];
    outMatrix[1] = -inMatrix[9];
    outMatrix[2] = -inMatrix[1];
    outMatrix[3] = -inMatrix[13];
    outMatrix[4] = -inMatrix[6];
    outMatrix[5] = inMatrix[10];
    outMatrix[6] = inMatrix[2];
    outMatrix[7] = inMatrix[14];
    outMatrix[8] = -inMatrix[4];
    outMatrix[9] = inMatrix[8];
    outMatrix[10] = inMatrix[0];
    outMatrix[11] = inMatrix[12];
    outMatrix[12] = -1 * inMatrix[7];
    outMatrix[13] = 1 * inMatrix[11];
    outMatrix[14] = 1 * inMatrix[3];
    outMatrix[15] = inMatrix[15];
}

void computeTransformation(int nrOfPts, string &inputFile, string &dir, bool reverseOrder, double *resTrans){
    ifstream file1(inputFile);
    if (!file1.is_open()) {
        cout << "Input file" << inputFile << " not found!" << endl;
    }
    vector<PtPair> Pairs;

    double x1, x2, y1, y2, z1, z2, centroid_m[3], centroid_d[3];
    centroid_m[0] = centroid_m[1] = centroid_m[2] = centroid_d[0] = centroid_d[1] = centroid_d[2] = 0;
    double alignxf[16];
    for (int i = 0; i < nrOfPts; i++){
        file1 >> x1 >> y1 >> z1 >> x2 >> y2 >> z2; //  world coords, scanner coords
        Point p1(x1, y1, z1);
        Point p2(x2, y2, z2);

        if (reverseOrder){
            PtPair p(p2, p1);
            Pairs.push_back(p);
            cout << i << ": " << p << endl;
            centroid_m[0] += 1.0 / nrOfPts*p2.x;
            centroid_m[1] += 1.0 / nrOfPts*p2.y;
            centroid_m[2] += 1.0 / nrOfPts*p2.z;
            centroid_d[0] += 1.0 / nrOfPts*p1.x;
            centroid_d[1] += 1.0 / nrOfPts*p1.y;
            centroid_d[2] += 1.0 / nrOfPts*p1.z;
        }
        else {
            PtPair p(p1, p2);
            Pairs.push_back(p);
            cout << i << ": " << p << endl;
            centroid_m[0] += 1.0 / nrOfPts*p1.x;
            centroid_m[1] += 1.0 / nrOfPts*p1.y;
            centroid_m[2] += 1.0 / nrOfPts*p1.z;
            centroid_d[0] += 1.0 / nrOfPts*p2.x;
            centroid_d[1] += 1.0 / nrOfPts*p2.y;
            centroid_d[2] += 1.0 / nrOfPts*p2.z;
        }

    }
    file1.close();

    icp6Dminimizer *my_icp6Dminimizer = 0;
    my_icp6Dminimizer = new icp6D_SVD(false);
    my_icp6Dminimizer->Align(Pairs, alignxf, centroid_m, centroid_d);

    // if Determinant < 0 --> computed matrix is a reflection
    if (M4det(alignxf)< 0) {
        // icp6D_SVD tries to compensate for reflection, if the output still is one, input data is most likely degenerate
        cerr << "Computed a reflection matrix as solution for point transformation - results would be erroneous!" << endl << "Canceling operation..." << endl;
        exit(1);
    }
    printMat_RowOrderStyle(alignxf);
    printMat_ColumnOrderStyle(alignxf);

    M4inv(alignxf, resTrans);

}

void modifyFrames(double* resTrans, string &dir, int start, int end, bool inputInXYZ){
    char inputFileName[MAX_PATH];
    char modFramesFileName[MAX_PATH];
    char modInputFileName[MAX_PATH];

    ifstream input_in;
    ofstream modFrames_out;
    ofstream modFrames_out2;

    for (size_t i = start; i <= end; i++){
        snprintf(inputFileName, 255, "%sscan%.3d.frames", dir.c_str(), i);
        snprintf(modFramesFileName, 255, "%sMODIFIED_scan%.3d.frames", dir.c_str(), i);

        if (inputInXYZ){
            snprintf(modInputFileName, 255, "%sMODIFIED_ORIG_scan%.3d.frames", dir.c_str(), i);
        }
        cout << "Processing Scan " << inputFileName << "..." << endl;
        cout.flush();

        input_in.open(inputFileName);
        if (!input_in.good()){
            cerr << "Error reading input file " << inputFileName << "!" << endl;
            return;
        }
        // store;
        modFrames_out.open(modFramesFileName);
        if (!modFrames_out.good()){
            cerr << "Error writing output file " << modFramesFileName << "!" << endl;
            return;
        }

        if (inputInXYZ){
            modFrames_out2.open(modInputFileName);
            if (!modFrames_out2.good()){
                cerr << "Error writing output file " << modInputFileName << "!" << endl;
                return;
            }
        }

        string line;
        size_t lineCnt = 0;
        while (std::getline(input_in, line)){
            cout << lineCnt++ << "...";
            std::istringstream iss(line);
            double inputMat[16];
            double inputMat3DTK[16];
            for (size_t j = 0; j < 16; j++)
            {
                iss >> inputMat[j];
            }

            if (inputInXYZ){
                to3DTKMat(inputMat, inputMat3DTK);

                for (size_t j = 0; j < 16; j++) {
                    modFrames_out2 << inputMat3DTK[j] << " ";
                }
                modFrames_out2 << "3" << endl;
            }
            else {
                // just copy values
                for (size_t j = 0; j < 16; j++) {
                    inputMat3DTK[j] = inputMat[j];
                }
            }

            // transform
            double modMat[16];
            MMult(inputMat3DTK, resTrans, modMat);

            for (size_t j = 0; j < 16; j++) {
                modFrames_out << modMat[j] << " ";
            }
            modFrames_out << "3" << endl;

        }
        cout << endl;

        input_in.close();
        input_in.clear();

        modFrames_out.close();
        modFrames_out.clear();

        if (inputInXYZ){
            modFrames_out2.close();
            modFrames_out2.clear();
        }
    }

    cout << "Done." << endl;
}

/**
 * Converts a number of frame files. Takes an input file with two sets of point pairs, and computes the transformation between them;
 * then reads a number of (possibly right-handed) transformations from the input frame files and applies the computed transformation to them,
 * before writing the resultant transformation back into the frame file dir (new files will be created, the input frames are safe :) ).
 * Usage: bin/transformFrames [parameters] 'dir',
 * with 'dir' the directory of a set of frame files
 * ...
 */
int main(int argc, char **argv)
{
    if (argc <= 1) {
        usage(argv[0]);
    }

    string dir;
    int nrOfPts = 0;
    string inputFile;
    int    start = 0, end = -1;
    bool inputInXYZ = false;
    bool reverseOrder = false;

    parseArgs(argc, argv, dir, nrOfPts, inputFile, start, end, inputInXYZ, reverseOrder);

    double resTrans[16];
    computeTransformation(nrOfPts, inputFile, dir, reverseOrder, resTrans);

    modifyFrames(resTrans, dir, start, end, inputInXYZ);
}
