#ifndef __VIEWPOINT_H_
#define __VIEWPOINT_H_

#include <vector>
using std::vector;
#include <utility>
using std::pair;

#include "grid/scanGrid.h"
#include <string>
using std::string;

/**
 * The class manages a list of viewpoints from which the 
 * roboter has taken the scans.
 * It provides methods for adding a new viewpoint, saving and loading the
 * list.
 *
 * @author Sebastian Stock, Uwe Hebbelmann, Andre Schemschat
 * @date 22.02.08
 */
class viewpointinfo
{
 private:
    /** Typedef for a viewpoint */
    typedef pair<long, long> viewpoint;
    
    /** Internal vector for storing the viewpoints */
    vector<viewpoint> viewpoints;

    /** The path where the file is created */
    string path;

 public:
    /** @brief CTor */
    viewpointinfo(string path);

    /** @brief Adds the viewpoint information of a scanGrid */
    void addGrid(const scanGrid *grid);
    
    /** @brief Writes the internal list to file */
    void write(string filename);
};

#endif
