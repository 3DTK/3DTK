#ifndef __GRIDLINES_H_
#define __GRIDLINES_H_

#include <vector>
using std::vector;

#include <string>
using std::string;
#include "grid/grid.h"
#include "grid/line.h"


/**
 * Class calculates all lines of a grid using hough transformation.
 * Consideres all points with a percentage higher than "isSolidPoint" variable.
 * Contains a method to write the lines to file (GnuPlot or own lin)
 *
 * @author Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * @date 25.02.2008
 */
class gridlines
{
 private:
    /** Minimal percentage for a solid point relevant for a line */
    double isSolidPoint;

    /** contains all lines of the grid */
    vector<line> lines;

 public:
    /** @brief CTOR */
    gridlines(grid* g, int max_distance, double isSolidPoint);

    /** 
     * @brief Creates the lines for the grid g and stores them in the vector "lines" 
     */
    void createLines(grid* g, int max_distance);

    /** @brief Returns the vector with the lines */
    vector<line>* getLines();

    /** @brief Writes the lines to a gnuplot file */
    void writeGnuPlot(string file);

    /** @brief Writes the lines to a lin file */
    void writeLin(string file);
};

#endif
