/*
 * colorGradient.cc
 *
 *  Created on: Apr 29, 2013
 *      Author: rdumitru
 */

//==============================================================================
//  Includes.
//==============================================================================
#include "floorplan/colorGradient.h"

#include <algorithm>

//==============================================================================
//  Class implementation.
//==============================================================================
void floorplan::ColorGradient::addColorPoint(const float &red, const float &green, const float &blue, const float &value) {
    for(int i = 0; i < (int) color.size(); i++) {
        if(value < color[i].val) {
            color.insert(color.begin()+i, ColorPoint(red,green,blue, value));
            return;
        }
    }
    color.push_back(ColorPoint(red,green,blue, value));
}

void floorplan::ColorGradient::clearGradient() {
    color.clear();
}

void floorplan::ColorGradient::createJetMap() {
    color.clear();
    color.push_back( ColorPoint(0.0, 0.0, 0.0,   0.000f ) );
    color.push_back( ColorPoint(0.0, 0.0, 1.0,   0.166f ) );
    color.push_back( ColorPoint(0.0, 1.0, 1.0,   0.333f ) );
    color.push_back( ColorPoint(0.0, 1.0, 1.0,   0.333f ) );
    color.push_back( ColorPoint(0.0, 1.0, 0.0,   0.500f ) );
    color.push_back( ColorPoint(1.0, 1.0, 0.0,   0.666f ) );
    color.push_back( ColorPoint(1.0, 0.0, 0.0,   0.833f ) );
    color.push_back( ColorPoint(1.0, 1.0, 1.0,   1.000f ) );

//        color.push_back( ColorPoint(0, 0, 1,   0.0f ) );      // blue
//        color.push_back( ColorPoint(0, 1, 1,   0.25f) );      // cyan
//        color.push_back( ColorPoint(0, 1, 0,   0.5f ) );      // green
//        color.push_back( ColorPoint(1, 1, 0,   0.75f) );      // yellow
//        color.push_back( ColorPoint(1, 0, 0,   1.0f ) );      // red
}

void floorplan::ColorGradient::createHotMap() {
    color.clear();
    color.push_back( ColorPoint(0.0, 0.0, 0.0,   0.000f ) );
    color.push_back( ColorPoint(1.0, 0.0, 0.0,   0.333f ) );
    color.push_back( ColorPoint(1.0, 1.0, 0.0,   0.666f ) );
    color.push_back( ColorPoint(1.0, 1.0, 1.0,   1.000f ) );
}

void floorplan::ColorGradient::getColorAtValue(const float &value, float &red, float &green, float &blue ) {
    if(color.size() == 0) {
        return;
    }

    for(int i = 0; i < (int)color.size(); i++) {
        ColorPoint &currC = color[i];
        if(value < currC.val) {
            ColorPoint &prevC  = color[std::max(0, i-1)];
            float valueDiff    = (prevC.val - currC.val);
            float fractBetween = (valueDiff==0) ? 0 : (value - currC.val) / valueDiff;
            red   = (prevC.r - currC.r) * fractBetween + currC.r;
            green = (prevC.g - currC.g) * fractBetween + currC.g;
            blue  = (prevC.b - currC.b) * fractBetween + currC.b;
            return;
        }
    }

    red   = color.back().r;
    green = color.back().g;
    blue  = color.back().b;

    return;
}
