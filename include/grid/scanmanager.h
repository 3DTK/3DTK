#include <vector>

#include "slam6d/scan.h"
#include "slam6d/managedScan.h"

/**
 * The class manages all scans and the frames of the scans
 * It reads all scans and frame information and holds an object for the
 * scans and the transformationmatrices
 *
 * @author Uwe Hebbelmann, Andre Schemschatt, Sebastian Stock
 * date 14.2.08
 */
class scanmanager {

 private:    
    /** Vector for the transformation of all scans */
    std::vector < std::vector <double*> > metaMatrix;

    /** @brief Reads the frame files that were created by Slam6D */
    void readFrames(string inputdir,
		    int start,
		    int end,
		    bool readInitial,
		    bool correctYAxis);
    
 public:
    /** @brief Ctor */
    scanmanager();

    /** @bried DTor */
    ~scanmanager();

    /** @brief Reads scans, frames and the transformationmatrix */
    void startscan(string inputdir, string outputdir, IOType scantype,
		   int start, int end, bool readInitial, 
		   int max_distance, int min_distance,
		   bool correctYAxis);

    /** @brief Getter for the number of scans */
    size_t getScanCount() const;
    
    /** @brief Returns scan with number i */
    Scan& getScan(int i);

    /** @brief Returns transformationmatrix with number i */
    const std::vector <double*>& getMatrix(int i);
};
