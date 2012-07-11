/*
 * viewpointinfo implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include <fstream>
using std::ofstream;

#include "grid/viewpointinfo.h"
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;
using std::make_pair;

/**
 * CTor.
 */
viewpointinfo::viewpointinfo(string path)
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
    this->viewpoints.push_back(make_pair(grid->getViewpointX(),
					 grid->getViewpointZ()));
}

/**
 * This methods writes the internal viewpointslist
 * to the file specified by filename
 *
 * @param filename The path and name of the file
 */
void viewpointinfo::write(string filename)
{
    string tmp(this->path + "/" + filename);
  
    ofstream stream(tmp.c_str());
    if(!stream.good())
    {
	cerr << " Unable to open file ! ("<< filename<<")"<< endl;
	exit(1);
    }


    vector<viewpoint>::iterator it = this->viewpoints.begin();
    vector<viewpoint>::iterator end = this->viewpoints.end();

    while(it != end)
    {
	stream << it->first << " " << it->second << endl;	
	++it;
    }
}

