/*
 * colorGradient.h
 *
 *  Created on: Apr 29, 2013
 *      Author: rdumitru
 */

#ifndef COLORGRADIENT_H_
#define COLORGRADIENT_H_

//==============================================================================
//  Includes.
//==============================================================================
// C++ includes.
#include <vector>

//==============================================================================
//  Class declaration.
//==============================================================================
namespace floorplan {

class ColorGradient {
private:
    struct ColorPoint {
        float r, g, b;
        float val;
        ColorPoint(float red, float green, float blue, float value)
            : r(red), g(green), b(blue), val(value) {}
    };

    //-- an array of color points in ascending value
    std::vector<ColorPoint> color;

public:
    //-- default constructor
    explicit inline ColorGradient() {
        this->createJetMap();
    }

    //-- inserts a new color point into it's correct position
    void addColorPoint(const float &red, const float &green, const float &blue, const float &value);

    //-- inserts a new color point into it's correct position
    void clearGradient();

    //-- places a jet gradient into the "color" vector
    void createJetMap();

    //-- places a hot gradient into the "color" vector
    void createHotMap();

    //-- inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
    void getColorAtValue(const float& value, float &red, float &green, float &blue);
};

} /* namespace floorplan */

#endif /* COLORGRADIENT_H_ */
