/**
 * @file commonTypes.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 15 Apr 2012
 *
 */

#ifndef COMMONTYPES_H_
#define COMMONTYPES_H_

//==============================================================================
//  Includes
//==============================================================================
#include "floorplan/point3d.h"
#include "floorplan/rotation3d.h"

#include <utility>

//==============================================================================
//  Typedefs
//==============================================================================
namespace floorplan {

typedef std::pair<floorplan::Point3d, floorplan::Rotation3d> Pose6d;

// the maximum image value
static const int MAX_IMG_VAL = 255;

} /* namespace floorplan */

#endif /* COMMONTYPES_H_ */
