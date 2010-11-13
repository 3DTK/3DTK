#ifndef __LINE_H_
#define __LINE_H_

#include "grid/gridPoint.h"

/** 
 * Class represents a line object starting at the gridPoint "start"
 * and ending at the gridPoint "end"
 *  
 * @author Sebastian Stock, Uwe Hebbelmann, Andre Schemschat
 * @date 25.02.2008
 */
class line
{
 private:
    /** The starting point of the line */
    gridPoint* start;

    /** The ending point of the line */
    gridPoint* end;

 public:
    /** @brief CTor */
    line(gridPoint* start, gridPoint* end);
    
    /** @brief Returns the length of the line */
    double getLength();

    /** 
     * Getter for the start x-coordinate 
     * @return the start x-coordinate
     */
    inline long getStartX() const{
	return start->getX();
    }

    /** 
     * Getter for the start z-coordinate 
     * @return the start z-coordinate
     */
    inline long getStartZ() const{
	return start->getZ();
    }

    /** 
     * Getter for the end x-coordinate
     * @return the end x-coordinate
     */
    inline long getEndX() const{
	return end->getX();
    }

    /** 
     * Getter for the end z-coordinate
     * @return the end z-coordinate
     */
    inline long getEndZ() const{
	return end->getZ();
    }
};

#endif
