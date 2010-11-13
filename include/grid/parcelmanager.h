#ifndef __PARCELMANAGER_H_
#define __PARCELMANAGER_H_

#include <string>
#include <map>
using std::map;

#include "grid/parcel.h"
#include "grid/parcelinfo.h"
#include <string>
using std::string;


#define PARCELINFOFILE "parcelinfo.conf"

/**
 * The parcelmanager manages all views of the map 
 * (Views are represented as parcels)
 * It provides methods for adding scangrids and creating the entire map.
 * It contains an internal memorymanagment for managing the parcels 
 * during runtime.
 * 
 * 
 * @author Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 * @date 15.02.08
 */
class parcelmanager
{
 private:
    /** Typedef for the map */
    typedef map<parcelinfo*, parcel*> parcelmap;
    
    /** The map for all parcelinfos and parcels */
    parcelmap parcels;

    /** The width of each parcel */
    int parcelwidth;
    /** The height of each parcel */
    int parcelheight;

    /** Last viewpoint (X) */
    long viewpointX;
    /** Last viewpoint (Z) */
    long viewpointZ;

    /** The resolution set as parameter */
    int resolution;

    /** The minimal found x value */
    long minX;
    /** The maximal found x value */
    long maxX;
    
    /** The minimal found z value */
    long minZ;
    /** The maximal found z value */
    long maxZ;

    
    /** The path where all infos should be stored */
    string path;

    /** @brief The method frees each (not used) parcel */ 
    void freeMemory(bool all);

    /** @brief The method clears all internal data */
    void clear();

    /** @brief The method loads a parcel specified by info */
    void loadParcel(parcelinfo *info);

    /** @brief The method creates a new parcel */
    void createParcel(long x, long z);

    /** @brief Method keeps min/Max-X/Z up to date */
    void updateOuterPoints(const grid* g);

 public:
    /** @brief CTor */
    parcelmanager(long width, long height, string path, int resolution, bool resume);

    /** @brief Dtor */
    ~parcelmanager();

    /** @brief The method adds a grid to all affected parcels */
    void addGrid(const grid* g, long vpX, long vpZ);

    /** @brief The method saves the infos created so far */
    void saveParcelinfo(string filename);

    /** @brief The method loads the parcelinfos from file */
    void loadParcelinfo(string filename);

    /** @brief The method combines all parcels to a worldmap and writes it */
    void writeWorld(string filename);

    /** @brief Merges all parcels into one grid and returns it */
    grid* createWorldGrid();
};

#endif
