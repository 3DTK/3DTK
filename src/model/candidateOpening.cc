/**
 * @file candidateOpening.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 15 Apr 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/candidateOpening.h"

//==============================================================================
//  Static member initializations
//==============================================================================
const unsigned int model::CandidateOpening::NR_FEATURES = 14;

//==============================================================================
//  Class implementation
//==============================================================================
model::CandidateOpening::CandidateOpening() {

}

model::CandidateOpening::CandidateOpening(const Point3d& pt, const Vector3d& normal,
        const std::vector<Point3d>& hull,
        const std::vector<double>& features, const std::vector<int>& edges) :
        Plane3d(pt, normal, hull)
{
    this->features = features;
    this->edges = edges;
}

model::CandidateOpening::CandidateOpening(const CandidateOpening& other) : Plane3d(other){
    this->features = other.features;
    this->edges = other.edges;
    this->edgeCoverage = other.edgeCoverage;
}

model::CandidateOpening::~CandidateOpening() {}

model::CandidateOpening& model::CandidateOpening::operator=(const CandidateOpening& other) {
    if (this != &other) {
        Plane3d::operator =(other);
        this->features = other.features;
        this->edges = other.edges;
        this->edgeCoverage = other.edgeCoverage;
    }

    return *this;
}
