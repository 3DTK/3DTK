/*
 * parcelmanager implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/parcelmanager.h"
#include "slam6d/globals.icc"
#include "grid/gridWriter.h"
#include <fstream>
using std::ifstream;
using std::ofstream;
#include <iostream>
using std::cerr;
using std::endl;

/**
 * Ctor.
 * Sets the static size information of the parcelinfo-class.
 * If resume is true, it will try to load the last parcelinfo-file.
 * If the file was read successfully, it contains the parcels created
 * during the last run of the programm. (See the resumeflag in the
 * documentation for more infos)
 * 
 * 
 * @param width The width of each parcel
 * @param height The height of each parcel
 * @param path The path of the files
 * @param resolution The resolution of a cell
 * @param resume If true, last parcelinfofile will be loaded
 */
parcelmanager::parcelmanager(long width, long height,
			     string path, int resolution,
			     bool resume)
{
    this->parcelwidth = width;
    this->parcelheight = height;
    this->viewpointX = 0;
    this->viewpointZ = 0;
    this->path = path;
    this->resolution = resolution;
    
    this->minX = 0;
    this->maxX = 0;
    this->minZ = 0;
    this->maxZ = 0;
    
    parcelinfo::setParcelsize(width, height);

    // loadParcelinfo if programm should resume
    if(resume)
      loadParcelinfo(this->path + PARCELINFOFILE);
}

/**
 * Dtor.
 * Calls saveParcelinfo for saving infos about 
 * already created parcel.
 * It also clears the internal structur (calls clear())
 */
parcelmanager::~parcelmanager()
{
    saveParcelinfo(this->path + PARCELINFOFILE);
  
    clear();
}

/**
 * This methods removes all parcels with a not set
 * used-flag (in parcelinfo). They are saved to disk
 * before they are freed.
 * If all is set, all parcels are saved and free regardless
 * of their used-flag
 *
 * @param all Delete all parcels or not?
 */
void parcelmanager::freeMemory(bool all)
{
    parcelmap::iterator cur = this->parcels.begin();
    parcelmap::iterator end = this->parcels.end();
    
    // iterate through each parcel
    while(cur != end)
    {
	// if parcel is loaded and wasnt used (or all free)
	if(cur->second != NULL &&
	   (!cur->first->wasUsed() || all))
	{	    
	    // saving the parcel using the parcel format
            // (Must use parcelFormat, otherwise parcel cant be loaded again!)
	    string filename = cur->first->getFilename();
	    parcelWriter writer(filename);

	    writer.write(*cur->second);
	    
	    delete cur->second;
	    cur->second = NULL;
	    
	}
	
	// reset used-flag of parcelinfo
	parcelinfo *p = cur->first;
	p->resetUsed();
	
	++cur;
    }
}

/**
 * The method frees all created parcelinfos and clears the map.
 * First it calls freeMemory(true) to save all parcels in memory
 */ 
void parcelmanager::clear()
{
  freeMemory(true);
  
  parcelmap::iterator it = this->parcels.begin();
  parcelmap::iterator end = this->parcels.end();

  while(it != end)
  {
      delete it->first;      
      ++it;
  }

  this->parcels.clear();
}

/**
 * The methods loads the parcel specified by info
 * into memory. The method does check, if the 
 * parcel is already loaded, so calling this method
 * on an already loaded parcel does nothing.
 *
 * @param *info A pointer to the parcelinfo 
 */
void parcelmanager::loadParcel(parcelinfo* info)
{
    if(this->parcels[info] == NULL)
    	this->parcels[info] = parcel::readParcel(info->getFilename());

    updateOuterPoints(this->parcels[info]);    
}


/**
 * The method creates a new parcel. It calculates the offset
 * of the new parcel based on the absolute coordinates it is given.
 * The new parcel is added to the internal map, along with its
 * parcelinfo
 *
 * @param x The absolute x coordinate
 * @param z The absolute z coordinate
 */
void parcelmanager::createParcel(long x, long z)
{
    // calculate offset
    long offsetX = (long) (x / this->parcelwidth) * this->parcelwidth;
    if (x < 0) 
	offsetX = offsetX - this->parcelwidth;

    long offsetZ = (long) (z/ this->parcelheight) * this->parcelheight;
    if (z < 0)
	offsetZ = offsetZ - this->parcelheight;


    // create parcelinfo and parcel
    string filename = this->path + "parcel" + to_string(offsetX) + to_string(offsetZ) + ".pcl";
    
    parcelinfo *p = new parcelinfo(offsetX, offsetZ, filename);
    
    this->parcels[p] = new parcel(offsetX,
				  offsetZ,
				  this->parcelwidth,
				  this->parcelheight);  

    // update the new borders
    updateOuterPoints(this->parcels[p]);    
}

/**
 * Updates the minimal and maximal found limits
 * @param g The grid to update
 */
void parcelmanager::updateOuterPoints(const grid *g)
{
    if(g->getOffsetX() < minX)
	minX = g->getOffsetX();
    if(g->getOffsetZ() < minZ)
	minZ = g->getOffsetZ();
    
    if(g->getOffsetX() + g->getSizeX() > maxX)
	maxX = g->getOffsetX() + g->getSizeX();
    if(g->getOffsetZ() + g->getSizeZ() > maxZ)
	maxZ = g->getOffsetZ() + g->getSizeZ();
}


/**
 * The method adds the given grid to the needed parcels.
 * It checks which parcels are affected by the grid, loads them if
 * they already exist or creates new ones.
 * Afterwards it adds the information of the grid to each relevant
 * parcels
 *
 * @param g The grid which should be addeda
 * @param vpX The x-coordiante of the viewpoint
 * @param vpZ The z-coordinate of the viewpoint
 */
void parcelmanager::addGrid(const grid* g, long vpX, long vpZ)
{
  bool parcelFound;
 
  this->viewpointX = vpX;
  this->viewpointZ = vpZ;
  
  long iSize = g->getOffsetX() + g->getSizeX();
  long jSize = g->getOffsetZ() + g->getSizeZ();

  // iterate through all parcels needed in this grid
  for(int i=g->getOffsetX() ; i < iSize + this->parcelwidth; i += this->parcelwidth)
  {
      for(int j=g->getOffsetZ() ; j < jSize + this->parcelheight; j += this->parcelheight)
      {
	  parcelmap::iterator it = this->parcels.begin();
	  parcelmap::iterator end = this->parcels.end();
      
	  parcelFound = false;

	  // for each point, iterate through the existing parcels
	  while(it != end)
	  {
	      // parcel contains point
	      if(it->first->contains(i, j))
	      {
		  // load Parcel if not in memory
		  if(it->second == NULL)
		      loadParcel(it->first);
		  
		  // reset the counter
		  it->first->setUsed();
		  parcelFound = true;
	      }	

	      ++it;
	  }
	  
	  // if parcel was not found, create new
	  if(parcelFound == false)
	      createParcel(i, j);
      }
  }

  // remove all parcels not needed for the scan
  freeMemory(false); 

  
  // Call addGrid for each parcel in memory
  // The parcel only integrates the points which are
  // relevant to it 
  parcelmap::iterator it = this->parcels.begin();
  parcelmap::iterator end = this->parcels.end();
  
  while(it != end)
  {
      if(it->second != NULL)
	  it->second->addGrid(g);

      ++it;
  }
}


/**
 * The method saves the internal parcelinfo map to file,
 * so it can be restored for later use (e.g. for additional scans
 * based on the same scene)
 * 
 * @param filename The file where the parcelinfos are stored to
 */
void parcelmanager::saveParcelinfo(string filename)
{
  ofstream outfile(filename.c_str());
  
  // Check if stream is open and valid
  if(!outfile.good())
  {
    cerr << "[saveParcelinfo] Fehler, konnte Stream nicht oeffnen" << endl
	 << "(Filename: " << filename << ")" << endl;
      exit(1);
  }

 
  // iterate all entries and write them
  parcelmap::iterator it = this->parcels.begin();
  parcelmap::iterator end = this->parcels.end();
  
  while(it != end)
  {
       outfile << it->first->getOffsetX() << " " 
	       << it->first->getOffsetZ() << " " 
	       << it->first->getFilename() << endl; 

        ++it;
  }

  outfile.close();
}

/**
 * The method restores the parcelinfos written by saveParcelinfo.
 * 
 * @param filename The file where the parcelinfos are stored
 */
void parcelmanager::loadParcelinfo(string filename)
{
  ifstream infile(filename.c_str());

  // Stream ok?
  if(!infile.good())
  {
    cerr << "[loadparcelinfo] Fehler, konnte Stream nicht oeffnen" << endl
	 << "(filename: " << filename << ")" << endl;
    return;
  }

  // clear old information (should be already empty, but make sure)
  clear();

  long offsetX;
  long offsetZ;
  string file;

  // read all infos
  while(!infile.eof())
  {
    infile >> offsetX;
    if(infile.eof()) continue;
    infile >> offsetZ;
    if(infile.eof()) continue;
    infile >> file;
    if(infile.eof()) continue;

    parcelinfo *p = new parcelinfo(offsetX, offsetZ, file);
    this->parcels[p] = NULL;
  }

  infile.close();
}

/**
 * This method is able to write the entire world, based on all parcels and create and write lines.
 * Each parcel is loaded and then written into a single file.
 * Afterwards ALL parcels are stored to file and freed from memory.
 * Additionally the world is written in ppm format and lines can be created and written.
 *
 * The map is written in a format which can be read by the MapViewer of Group2.
 * The specification of the format are listed in the documentation
 * @param file the filename of the world
 */
void parcelmanager::writeWorld(string file)
{
    worldWriter writer(this->path + file,
		       this->minX, this->maxX, this->minZ, this->maxZ,
		       this->resolution,
		       this->viewpointX, this->viewpointZ);
  

    parcelmap::iterator it = parcels.begin();
    parcelmap::iterator end = parcels.end();
    
    while(it != end)
    {
	if(it->second == NULL)
	    loadParcel(it->first);

	writer.write(*it->second);
	
	++it;
    }
}



/**
 * The method creates a grid of the world.
 */
grid* parcelmanager::createWorldGrid()
{
    grid *g = new grid(this->minX, this->minZ,
		       this->maxX - this->minX + 1,
		       this->maxZ - this->minZ + 1);

    parcelmap::iterator it = parcels.begin();
    parcelmap::iterator end = parcels.end();
    
    // Go through all parcels
    while(it != end)
    {
	if(it->second == NULL)
	    loadParcel(it->first);

	for(int i=0; i < it->second->getSizeX(); ++i)
	{
	    for(int j=0; j < it->second->getSizeZ(); ++j)
	    {
		g->addPoint(*(it->second->points[i][j]));
	    }
	}
	
	++it;
    }
    return g;
}

