/**
 * @file candidateOpening.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 15 Apr 2012
 *
 */

#ifndef CANDIDATEOPENING_H_
#define CANDIDATEOPENING_H_

//==============================================================================
//  Includes
//==============================================================================
#include "model/plane3d.h"

#include <vector>

namespace model {

class CandidateOpening : public Plane3d {
public:
    static const unsigned int NR_FEATURES;   //!< The number of features required for the SVM.

public:
    std::vector<double> features;   //!< The 14 features required for the SVM.
    std::vector<int> edges;         //!< Coordinates of each edge (uh, lh, uv, lv).
    double edgeCoverage;            //!< Percentage on how much the edges correspond to edges detected in Canny.

public:
    // constructors and destructors
    CandidateOpening();
    CandidateOpening(const Point3d& pt, const Vector3d& normal,
            const std::vector<Point3d>& hull,
            const std::vector<double>& features, const std::vector<int>& edges);
    CandidateOpening(const CandidateOpening& other);
    virtual ~CandidateOpening();

    // operators
    CandidateOpening& operator=(const CandidateOpening& other);
};

} /* namespace model */

#endif /* CANDIDATEOPENING_H_ */
