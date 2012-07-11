/*
 * gridlines implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/gridlines.h"
#include "grid/hough.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;

#define MINIMALLINELENGTH 2

/**
 * CTOR
 * 
 * @param g Grid in which lines should be created 
 * @param max_distance the maximal relecant distance for points
 */
gridlines::gridlines(grid * g, int max_distance, double isSolidPoint)
{
    this->isSolidPoint = isSolidPoint;
    createLines(g, max_distance);
}    

/**
 * Creates the lines for the grid g and stores 
 * them in the vector "lines" (clears the vector first)
 * Uses hough transformation
 * 
 * @param g the grid that the lines will be created for
 * @param max_distance the maximal relevant distance for points 
 */
void gridlines::createLines(grid* g, int max_distance)
{
    // read the data out of the grid  
    double x, z;
    vector <double> vx;
    vector <double> vz;

    // initialize minimal and maximal x and z values
    double xmin = 8000.0, xmax = -8000.0, ymin = 8000.0, ymax = -8000.0;
    for (int i = 0; i < g->getSizeX(); i++) {
        for (int j = 0; j < g->getSizeZ(); j++) { 
	    //store the point if percentage > ISSOLIDPOINT
	    if (g->points[i][j]->getPercent() > isSolidPoint) {
	        x = g->points[i][j]->getX();
	        z = g->points[i][j]->getZ();

		vx.push_back(x);
		vz.push_back(z);
		
		// calculate minimal and maximal x and z values
		if (x > xmax) xmax =  x;
		if (x < xmin) xmin =  x;
		if (z > ymax) ymax =  z;
		if (z < ymin) ymin =  z;
	    }
	}
    }

    // cut the maximal and minimal values if they are too big
    if (xmax > max_distance) xmax = max_distance;
    if (ymax > max_distance) ymax = max_distance;
    if (xmin < -max_distance) xmin = -max_distance;
    if (ymin < -max_distance) ymin = -max_distance;
 
    // create arrays for the "hough" class and norm values 
//    double array_x[vx.size()], array_z[vz.size()];
    double *array_x = new double[vx.size()];
	double *array_z = new double[vz.size()];
    for (size_t t = 0; t < vx.size(); t++) {
	array_x[t] = vx.at(t) - xmin; 
	array_z[t] = vz.at(t) - ymin;
    }
 
    // set and get parameters for Hough transformation   
    int sht_nr = 80;
    double max_rho_d = sqrt(pow(xmax-xmin, 2) + pow(ymax-ymin, 2));

    //Set maximal distance
    SHT_set_max_distance(max_distance, vx.size());

    // result variables
    int sht_nr_line_pts;
    double *sht_x_line_pts = new double[vx.size()];
	double *sht_y_line_pts = new double[vx.size()];
    
    // get sht_resolutuion
    int sht_resolution = max_distance;
    sht_init(sht_resolution);

    // get sht_histogram
    int **sht_histogram;
    sht_histogram = SHT_alloc_histogram(sht_resolution, sht_resolution);
 
    // call of Hough transformation
    SHT_get_hough_lines(vx.size(), 
			array_x, 
			array_z,
			sht_resolution,
			max_rho_d,
			sht_histogram, 
			sht_nr,
			&sht_nr_line_pts, 
			sht_x_line_pts, 
			sht_y_line_pts, 
			xmin, 
			ymin, 
			1);
    
    cout << "Found " << sht_nr_line_pts/2 << " lines!" << endl; 

    // set results back -> norm
    for (int j = 0; j < sht_nr_line_pts; j++) {
	sht_x_line_pts[j] += xmin; 
	sht_y_line_pts[j] += ymin;
    }

    // delete all lines
    this->lines.clear();
   
    // store the calculated lines
    for (int i = 0; (i < sht_nr_line_pts); i+=2) {
	gridPoint* temp_point1 = new gridPoint((long) sht_x_line_pts[i], (long) sht_y_line_pts[i]);
	gridPoint* temp_point2 = new gridPoint((long) sht_x_line_pts[i+1], (long) sht_y_line_pts[i+1]);
	
	if (temp_point1->isSmallerThan(*temp_point2)) {
	    lines.push_back(line(temp_point1, temp_point2));
	} else {
	    lines.push_back(line(temp_point2, temp_point1));
	}	
    }

    SHT_free_histogram(sht_histogram, sht_resolution);

	delete[] array_x;
	delete[] array_z;
	delete[] sht_x_line_pts;
	delete[] sht_y_line_pts;
}

/**
 * Getter for the vector with the lines
 *
 * @returns the vector with the created lines
 */
vector<line>* gridlines::getLines()
{
    return &this->lines;
}

/**
 * Writes the lines to a gnuplot file
 *
 * @param file the file to write to
 */
void gridlines::writeGnuPlot(string file)
{
    ofstream str;
    str.open(file.c_str());
    if(!str.good())
    {
	cerr << "ERROR: In gridlines::writeGnuPlot, unable to open the stream! " 
	     << endl;
	cerr << "(Filename: " << file << ")" << endl;
	exit(1);
    }

    for(size_t i = 0; i < lines.size(); i++) {
	str << lines.at(i).getStartX() << " " << lines.at(i).getStartZ() << endl;
	str << lines.at(i).getEndX() << " " << lines.at(i).getEndZ() << endl;
	str << endl;
    }

    str.close();
}

/**
 * Writes the lines to a lin file
 *
 * @param file the file to write to
 */
void gridlines::writeLin(string file)
{
    ofstream str;
    str.open(file.c_str());
    if(!str.good())
    {
	cerr << "ERROR: In gridlines::writeLin, unable to open the stream! " 
	     << endl;
	cerr << "(Filename: " << file << ")" << endl;
	exit(1);
    }

    for(size_t i = 0; i < lines.size(); i++) {
	if(lines.at(i).getLength() < MINIMALLINELENGTH)
	    continue;

	str << "BEGIN" << endl;
	str << lines.at(i).getStartX() << " 0 " << lines.at(i).getStartZ() << endl;
	str << lines.at(i).getEndX() << " 0 " << lines.at(i).getEndZ() << endl;
	str << "END" << endl;
    }

    str.close();
}
