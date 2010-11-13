#ifndef __PARCELINFO_H_
#define __PARCELINFO_H_

#include <string>

/**
 * The class contains the information about an already created parcel,
 * such as offset and filename, but not the actual data.
 * It also contains the parcel height and width as static members.
 *
 * @author Uwe Hebbelmann, Sebastian Stock, Andre SChemschat
 * @date 15.02.08
 */
class parcelinfo
{
 private:
    /** The x-offset of parcel (absolute coordiante) */
    long offsetX;

    /** The z-offset of parcel (absolute coordiante) */
    long offsetZ;
    
    /** The filename of the parcel */
    std::string filename;
    
    /** sets whether the parcel was needed for the last scan or not */
    bool used;
    
    /** The width of the parcel */
    static long parcelwidth;
    
    /** The height of the parcel */
    static long parcelheight;

 public:
    /** @brief Sets the parcelsize */
    static void setParcelsize(long width, long height);

    /** @brief CTor */
    parcelinfo(long offsetX, long offsetZ, std::string filename);

    /** @brief < operator */
    bool operator< (const parcelinfo &parcel) const;

    /** @brief == operator */
    bool operator== (const parcelinfo &parcel) const;
      
    /** @brief Checks if the parcel contains a given point*/
    bool contains(long x, long z) const;

    /** @brief Sets the used flag (true) */
    void setUsed();
    
    /** @brief Resets the used flag (false) */
    void resetUsed();
    
    /** @brief Returns whether the parcel was needed */
    int wasUsed() const;

    /** 
     * Returns the x-offset 
     * @return the x-offset of the parcel
     */
    inline int getOffsetX() const {
    	return this->offsetX;
    }

    /** 
     * Returns the z-offset 
     * @return the z-offset of the parcel
     */
    inline int getOffsetZ() const {
	return this->offsetZ;
    }

    /**
     * Getter for the filename 
     * @return the filename of the parcel
     */
    inline const std::string& getFilename() const {
	return this->filename;
    }
};

#endif
