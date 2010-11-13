#ifndef __GRIDPOINT_H_
#define __GRIDPOINT_H_

/**
 * The class represents a point in the grid. It contains its
 * absolute coordinates (x, z), a counter how often the point
 * has been found within a scan and a counter how often it has 
 * been found occupied. 
 * 
 * @author Sebastian Stock, Uwe Hebbelmann, Andre Schemschat
 * @date 11.02.2008
 */
class gridPoint
{
 private:
    /** The absolute x coordinate */
    long x;

    /** The absolute z coordinate */
    long z;

    /** Counter how often point was visited */
    unsigned int count;

    /** Counter how often point was occupied */
    unsigned int occupied;

 public:
    /** @brief CTor */
    gridPoint(long x, long z);
    
    /** @brief Adds amount to the internal counter */
    void addCount(unsigned int count, unsigned int occupied);

    /** @brief Adds fixed values to both counters */
    void addFixed(unsigned int count, unsigned int occupied);

    /** @brief Overwrites the internal counters directly */
    void setFixed(unsigned int count, unsigned int occupied);

    /** @brief Returns the occupied value as Percent */
    float getPercent() const;

    /**
     * Getter for the visited counter
     * @return Count
     */
    inline unsigned int getCount() const{
	return this->count;
    }

    /**
     * Getter for the occupied counter
     * @return Count of Occupieds
     */
    inline unsigned int getOccupied() const{
	return this->occupied;
    }
    
    /**
     * Getter for the absolute x coordinate
     * @return the absolute x coordinate
     */
    inline long getX() const {
	return this->x;
    }

    /**
     * Getter for the absolute z coordinate
     * @return the absolute z coordinate
     */
    inline long getZ() const {
	return this->z;
    }

    /**
     * Checks if the point is smaller than the given point
     * @param p the point to campare to
     * @return true if smaller else false
     */
    inline bool isSmallerThan(const gridPoint& p) const
    {
	if (getX() < p.getX()) return true;
	if (getX() > p.getX()) return false;
	if (getZ() < p.getZ()) return true;
	if (getZ() > p.getZ()) return false;

	return true;
    }
};

#endif
