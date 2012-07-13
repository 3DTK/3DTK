/**
 * @file rotation3d.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

#ifndef RAOTATION3D_H_
#define RAOTATION3D_H_

//==============================================================================
//  Defines
//==============================================================================

//==============================================================================
//  Includes
//==============================================================================

namespace model {

/**
 * A rotation in space.
 */
class Rotation3d {
public:
    double x;
    double y;
    double z;

    inline Rotation3d(const double& x = 0.0, const double& y = 0.0, const double& z = 0.0) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Rotation3d(const Rotation3d& other);

    Rotation3d& operator=(const Rotation3d& other);

    /**
     * Returns a rotation that represents the inverse of the current one.
     */
    Rotation3d getInverse();
};

} /* namespace model */

#endif /* RAOTATION3D_H_ */
