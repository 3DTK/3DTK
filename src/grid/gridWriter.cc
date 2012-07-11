/*
 * gridWriter implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/gridWriter.h"
#include <iterator>
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;

/**
 * CTor of gridWriter. Tries to the file
 * specified by file and saves the stream if 
 * successfull
 *
 * @param file The filename of the file
 */
gridWriter::gridWriter(string file)
{   
    this->stream.open(file.c_str());
    if(!this->stream.good())
    {
	cerr << "ERROR: In gridWriter::gridWriter, unable to open the stream for gridWriter! " << endl;
	cerr << "(Filename: " << file << ")" << endl;
	exit(1);
    }  
}

/**
 * CTor for passing in an already opened stream.
 * The stream is validated, so it must already be open
 *
 * @param stream A reference to the stream to be used
 */
gridWriter::gridWriter(ofstream &stream)
{
}

/**
 * DTor. Closes the stream if open
 */
gridWriter::~gridWriter()
{
    if(this->stream.is_open())
	this->stream.close();
}

/**
 * CTor. Just calls gridWriter ctor.
 * See gridWriter::gridWriter(string) for more infos.
 * @param file The filename to be opend.
 */
ppmWriter::ppmWriter(string file)
    : gridWriter(file)
{}

/**
 * CTor. Just calls gridWriter ctor
 * See gridWriter::gridWriter(stream) for more infos.
 * @param stream the Stream
 */
ppmWriter::ppmWriter(ofstream &stream)
    : gridWriter(stream)
{}

/**
 * Method writes the grid to the ppm file format
 * @param grid the grid to write
 */
void ppmWriter::write(const grid& grid)
{
    stream << "P2" << endl;
    stream << "#Breite Hoehe" << endl;
    stream << grid.getSizeX() << " " << grid.getSizeZ() << endl;
    stream << "100" << endl;

    for(long i=grid.getSizeZ()-1; i >=0 ; --i)
    {
	for(long j=0; j < grid.getSizeX(); ++j)
	{
	    if(grid.points[j][i]->getPercent() < 0)
		stream << "50 ";
	    else
		stream << 100 - (int)(grid.points[j][i]->getPercent() * 100) << " ";
	}
	stream << endl;
    }
}

/**
 * CTor. Just calls gridWriter ctor.
 * See gridWriter::gridWriter(string) for more infos.
 * @param file The filename to be opend.
 */
parcelWriter::parcelWriter(string file)
    : gridWriter(file)
{
}

/**
 * CTor. Just calls gridWriter ctor
 * See gridWriter::gridWriter(stream) for more infos.
 * @param stream the Stream
 */
parcelWriter::parcelWriter(ofstream &stream)
    : gridWriter(stream)
{
}

/**
 * Method writes the grid to the parcel file format
 * @param grid the grid to write
 */
void parcelWriter::write(const grid& grid)
{
    stream << grid.getSizeX() << " " << grid.getSizeZ() << endl;
    stream << grid.getOffsetX() << " " << grid.getOffsetZ() << endl;

    for(long i = 0; i < grid.getSizeX(); ++i)
	for(long j = 0; j < grid.getSizeZ(); j++)
	    stream << grid.points[i][j]->getX() << " "
		   << grid.points[i][j]->getZ() << " "
		   << grid.points[i][j]->getCount() << " "
		   << grid.points[i][j]->getOccupied() << endl;
}
      
/**
 * CTor. Just calls gridWriter ctor
 * See gridWriter::gridWriter(stream) for more infos.
 * @param stream the Stream
 */
gnuplotWriter::gnuplotWriter(ofstream &stream)
    : gridWriter(stream)
{
}

/**
 * CTor. Just calls gridWriter ctor
 * See gridWriter::gridWriter(string) for more infos
 * @param file The filename
 */
gnuplotWriter::gnuplotWriter(string file)
    : gridWriter(file)
{
}

/**
 * Method writes the grid to the gnuplot file format
 * @param grid the grid to write
 */
void gnuplotWriter::write(const grid& grid)
{
    for(long i=0; i < grid.getSizeX(); ++i)
	for(long j=0; j < grid.getSizeZ(); ++j)
	    stream << grid.points[i][j]->getX() << " "
		   << grid.points[i][j]->getZ() << " "
		   << grid.points[i][j]->getPercent() << endl;
}

/**
 * CTor of worldwriter. Tries to open the stream and
 * writes some initial data about the world. (write is called
 * several times, so do it here)
 *
 * @param file The filename
 * @param minX The minimal found X value of all parcels
 * @param maxX The maximal found x value of all parcels
 * @param minZ The minimal found z value of all parcels
 * @param maxZ The maximal found z value of all parcels
 * @param resolution the resolution of the world
 * @param vpX the x coordinate of the viewpoint
 * @param vpZ the z coordinate of the viewpoint
 */
worldWriter::worldWriter(string file, long minX, long maxX, long minZ, long maxZ, int resolution, long vpX, long vpZ)
    : gridWriter(file)
{  
    stream << "1" << endl;
    stream << resolution << endl;
    stream << minX << " " << maxX << " " << minZ << " " << maxZ <<  endl;
    stream << vpX << " " << vpZ << endl;
}

 /**
 * CTor of worldwriter. Checks if the stream is valid and
 * writes some initial data about the world. (write is called
 * several times, so do it here)
 *
 * @param stream The stream to write to
 * @param minX The minimal found X value of all parcels
 * @param maxX The maximal found x value of all parcels
 * @param minZ The minimal found z value of all parcels
 * @param maxZ The maximal found z value of all parcels
 * @param resolution the resolution of the world
 * @param vpX the x coordinate of the viewpoint
 * @param vpZ the z coordinate of the viewpoint
 */
worldWriter::worldWriter(ofstream& stream, long minX, long maxX, long minZ, long maxZ, int resolution, long vpX, long vpZ)
    : gridWriter(stream)
{
    stream << "1" << endl;
    stream << resolution << endl;
    stream << minX << " " << maxX << " " << minZ << " " << maxZ <<  endl;    
    stream << vpX << " " << vpZ << endl;
}

/**
 * Method writes the grid to the world file format
 * @param grid the grid to write
 */
void worldWriter::write(const grid& grid)
{
    for(int i=0; i < grid.getSizeX(); ++i)
    {
	for(int j=0; j < grid.getSizeZ(); ++j)
	{
	    stream << grid.points[i][j]->getX() << " " 
		   << grid.points[i][j]->getZ() << " "
		   << grid.points[i][j]->getPercent() << endl;
	}
    }
}
