#ifndef __GRIDWRITER_H_
#define __GRIDWRITER_H_

#include <fstream>
#include <vector>

#include "grid/grid.h"
#include <string>
using std::string;

/**
 * The class represents the baseclass for all writers.
 * Writers are used by the grid-class for writing the 
 * internal gridPoint-array to a file. 
 * Writers for a specific format must inherit this class.
 * 
 * Furthermore it provides static methods for creating writers
 * for each format.
 *
 * @author Andre Schemschat, Sebastian Stock, Uwe Hebbelmann
 * @date 20.02.08
 */
class gridWriter
{
 protected:
    /** The referenz to the used stream */
    std::ofstream stream;
    
    /** The filename of the file to be opened*/
    std::string filename;
 public:
    /** @brief CTor */
    gridWriter(std::string file);

    /** @brief Copy-CTor */
    gridWriter(std::ofstream &stream);

    /** @brief DTor */
    virtual ~gridWriter();
    
    /** @brief Method to write the grid */
    virtual void write(const grid& grid) = 0;
};

/**
 * Class for writing the grid as a PPM to a file.
 * The PPM is in a greyscaled format with 100 tones.
 *
 * @author Andre Schemschat, Uwe Hebbelmann, Sebastian Stock
 * @date 20.2.08
 */
class ppmWriter : public gridWriter
{
 public:
    /** @brief CTor */
    ppmWriter(std::string file);
    
    /** @brief CTor */
    ppmWriter(std::ofstream &stream);

    /** @brief Method to write the grid */
    virtual void write(const grid& grid);
};

/**
 * Class for writing the grid in the parcel-file-format
 * (this writer is used to store each parcel during the convert)
 *
 * @author Sebastian Stock, Andre Schemschatt, Uwe Hebbelmann
 * @date 20.2.08
 */
class parcelWriter : public gridWriter
{
 public:
    /** @brief CTor */
    parcelWriter(std::string file);
    
    /** @brief CTor */
    parcelWriter(std::ofstream &stream);
    
    /** @brief Method to write the grid */
    virtual void write(const grid& grid);
};

/**
 * Class for writing the grid in the gnuplot-file-format
 *
 * @author Sebastian Stock, Andre Schemschatt, Uwe Hebbelmann
 * @date 20.2.08
 */
class gnuplotWriter : public gridWriter
{
 public:
    /** @brief CTor */
    gnuplotWriter(std::string file);
    
    /** @brief CTor */
    gnuplotWriter(std::ofstream &stream);

    /** @brief Writemethod for gnuplot */
    virtual void write(const grid& grid);
};

/**
 * Class for writing all parcels to one world map (own format)
 * @author Sebastian Stock, Andre Schemschatt, Uwe Hebbelmann
 * @date 20.2.08
 */
class worldWriter : public gridWriter
{
 public:
    /** @brief CTor */
    worldWriter(std::string file, long minX, long maxX,
		long minZ, long maxZ, int resolution,
		long vpX, long vpZ);
    
    /** @brief CTor */
    worldWriter(std::ofstream &stream, long minX, long maxX,
		long minZ, long maxZ, int resolution,
		long vpX, long vpZ);

    /** @brief Writemethod for the world map */
    virtual void write(const grid& grid);
};

#endif
