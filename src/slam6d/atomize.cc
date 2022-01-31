/**
 * @file
 * @author Fabian Arzberger, JMU, Germany
 *
 * atomize implementation
 *
 * @brief
 * This programm is the opposing program to condense.
 * Once you've condensed S subsequent scans into multiple MetaScans and matched them,
 * each MetaScan has a .frames file. The purpose of this program is to split the
 * .frames files back and apply them onto the corresponding single scans.
 *
 */

#include "slam6d/atomize.h"

int main(int argc, char **argv)
{
    // init, default values if not specified
    std::string cond_dir;
    std::string orig_dir;
    double red   = -1.0;
    int    rand  = -1;
    int    start = 0,   end = -1;
    int    maxDist    = -1;
    int    minDist    = -1;
    int octree       = 0;  // employ randomized octree reduction?
    IOType iotype    = UOS;
    bool rangeFilterActive = false;
    bool customFilterActive = false;
    std::string customFilter;
    int split = -1;
    bool rm_scatter = false;
    bool skip_empty = false;

    // parsing the command line parameters with boost::po, see header file
    try {
        parse_options(argc, argv, cond_dir, orig_dir, red, rand, start, end,
            maxDist, minDist, octree, iotype, customFilter, split, rm_scatter,
            skip_empty);
    } catch (std::exception& e) {
        std::cerr << "Error while parsing settings: " << e.what() << std::endl;
        exit(1);
    }

    // Read and set filter and reduction parameters
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

    // Open Scans from disk
    Scan::openDirectory(false, orig_dir, iotype, start, end);
    if(Scan::allScans.size() == 0) {
        std::cerr << "No scans found. Did you use the correct format?" << std::endl;
        exit(-1);
    }

    // Check which PointTypes the IOType supports
    unsigned int types = PointType::USE_NONE;
    if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
    if(supportsColor(iotype)) types |= PointType::USE_COLOR;
    if(supportsType(iotype)) types |= PointType::USE_TYPE;

    // If specified, filter scans
    for (size_t i = 0; i < Scan::allScans.size(); i++)  {
        if(rangeFilterActive) Scan::allScans[i]->setRangeFilter(maxDist, minDist);
        if(customFilterActive) Scan::allScans[i]->setCustomFilter(customFilter);
    }

    // If specified, apply reduction
    int end_reduction = (int)Scan::allScans.size();
    cout << "Applying reduction... " << endl;
    if (rm_scatter && red == -1
    || octree != 1 && red == -1 )
    {
        cout << "Conflicting reduction options found. " << endl;
        cout << "Use -r <arg> to set voxelsize. Use -O <arg> to specify nr. of pts. in a voxel." << endl;
        cout << "Use -d to delete the voxels with fewer pts. than specified with -O." << endl;
    }
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
    }

    // Iterate in the same fashion as in condense.cc to reconstruct file association

    int k = 0; // count subscans
    uint seq = 0; // count condensed files
    vector<Scan*> splitscans;
    double transMat[16], transMatPose[16]; // transform from condensed frames / pose
    double tPoseInv[16], transMatRel[16]; // relative transformations between frames / pose
    for (uint i = 0; i < Scan::allScans.size(); ++i)
    {
        Scan * source = Scan::allScans[i];
        // In atomize, we need to push every scan, even if its empty
        splitscans.push_back( source );
        // In condense, empty scans got skipped
        if ( source->size<DataXYZ>("xyz") == 0 ) continue;

        // In condense, new files get opened here
        // So in atomize, we have to read the transforms of those
        if (0 == k++) {
            readTransformFromFrames(cond_dir, seq, transMat);
            readTransformFromPose(cond_dir, seq, transMatPose);
            M4inv(transMatPose, tPoseInv);
            MMult(transMat, tPoseInv, transMatRel);
        }

        // In condense, buffered scans get written to disk here
        // So in atomize, we have to split the relative transformations on each original scan
        if (k == split && split != -1) {
            for (int j = 0; j < splitscans.size(); ++j) {
                // Get original transform from pose file
                const double* transMatOrig = splitscans[j]->get_transMatOrg();
                double transMatOut[16];
                MMult(transMatRel, transMatOrig, transMatOut);
                writeFrame(orig_dir, splitscans[j]->getIdentifier(), transMatOut);
            }

            // prepare next iteration:
            k = 0;
            seq++;
            splitscans.clear();
        }
    }

    // flush the rest
    if (!splitscans.empty())
    {
        for (int j = 0; j < splitscans.size(); ++j) {
            // Get original transform from pose file
            const double* transMatOrig = splitscans[j]->get_transMatOrg();
            double transMatOut[16];
            MMult(transMatRel, transMatOrig, transMatOut);
            writeFrame(orig_dir, splitscans[j]->getIdentifier(), transMatOut);
        }
    }

    return 0;
}