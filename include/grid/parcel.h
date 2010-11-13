#ifndef __PARCEL_H_
#define __PARCEL_H_

#include "grid/grid.h"
#include <string>
using std::string;

/**
 * The class represents a section of the map and inherrits grid.
 * The grids will be added to the parcel
 * The parcel can be stored and read so that it must not permanant be
 * in the memory.
 * 
 * @author Andre Schemschatt, Uwe Hebbelmann, Sebastian Stock
 * @date 17.2.08
 */ 
class parcel : public grid
{
 public:
    /** @brief Ctor */
    parcel(long offSetX, long offSetZ, long sizeX, long sizeZ);

    /** @brief Adds a grid to the parcel */
    void addGrid(const grid* grid);

    /** @brief Creates a parcel from the file */
    static parcel* readParcel(std::string filename);
};

#endif
