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
#include "model/point3d.h"
#include "model/rotation3d.h"

#include <utility>

//==============================================================================
//  Typedefs
//==============================================================================
namespace model {

typedef std::pair<model::Point3d, model::Rotation3d> Pose6d;

// the maximum image value
static const int MAX_IMG_VAL = 255;

} /* namespace model */

#endif /* COMMONTYPES_H_ */
