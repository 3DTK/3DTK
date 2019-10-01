/*
 * viewpointinfo implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include <fstream>

#include "grid/viewpointinfo.h"
#include <cstdlib>
#include <iostream>

/**
 * CTor.
 */
viewpointinfo::viewpointinfo(std::string path)
{
  this->path = path;
}

/**
 * The method adds the viewpointinfos of the given grid
 * to the internal list
 *
 * @param grid The grid with the new viewpointinfos
 */
void viewpointinfo::addGrid(const scanGrid *grid)
{
    this->viewpoints.push_back(std::make_pair(grid->getViewpointX(),
					 grid->getViewpointZ()));
}

/**
 * This methods writes the internal viewpointslist
 * to the file specified by filename
 *
 * @param filename The path and name of the file
 */
void viewpointinfo::write(std::string filename)
{
    std::string tmp(this->path + "/" + filename);

    std::ofstream stream(tmp.c_str());
    if(!stream.good())
    {
	std::cerr << " Unable to open file ! ("<< filename<<")"<< std::endl;
	exit(1);
    }


    std::vector<viewpoint>::iterator it = this->viewpoints.begin();
    std::vector<viewpoint>::iterator end = this->viewpoints.end();

    while(it != end)
    {
	stream << it->first << " " << it->second << std::endl;
	++it;
    }
}

