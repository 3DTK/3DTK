#ifndef __VIEWPOINT_H_
#define __VIEWPOINT_H_

#include <map>
#include <vector>

#include "scanGrid.h"

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
    typedef std::pair<long, long> viewpoint;
    
    /** Internal vector for storing the viewpoints */
    std::vector<viewpoint> viewpoints;

    /** The path where the file is created */
    std::string path;

 public:
    /** @brief CTor */
    viewpointinfo(std::string path);

    /** @brief Adds the viewpoint information of a scanGrid */
    void addGrid(const scanGrid *grid);
    
    /** @brief Writes the internal list to file */
    void write(std::string filename);
};

#endif
