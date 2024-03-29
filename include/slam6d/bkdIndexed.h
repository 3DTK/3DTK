/** @file
 *  @brief Representation of an optimized Bkd tree.
 *
 *  A Bkd-tree is like a kd-tree, but preserves balance and space
 *  utilization when inserting new or removing old points.
 *  It is therefore dynamicaly updated by handling multiple,
 *  logarithmic ordered kd-trees.
 *
 *  See:
 *  Procopiuc, O., Agarwal, P. K., Arge, L., & Vitter, J. S. (2003).
 *  Bkd-Tree: A Dynamic Scalable kd-Tree.
 *  Lecture Notes in Computer Science, 46–65.
 *  doi:10.1007/978-3-540-45072-6_4
 *
 *  TODO: the Bkd class derives from the SearchTree class.
 *  However, it does neither implement lock() nor unlock(), so it can't
 *  be used with the scanserver!
 *
 *  @author Fabian Arzberger, Uni Wuerzburg
 */

#ifndef __BKD_INDEXED_H_
#define __BKD_INDEXED_H_

#include "slam6d/kdIndexed.h"
#include "slam6d/searchTree.h"
#include <vector>

// This data structure stores trees.
// The trees themselfes do not have any pts stored.
// Instead, they operate on the "DataElem" structure by reorganisation.

struct ForestElemIndexed
{
    // Create an empty Forest element, i.e. an empty tree (called 'sprout')
    ForestElemIndexed() : empty(true), nrpts(0) {} // empty constructor, we use c++ init list
    // Create a non-empty Forest element, i.e. a KD-tree (called 'tree')
    ForestElemIndexed(KDtreeIndexed* t, size_t n) : empty(false), tree(t), nrpts(n) {}

    // Struct data
    bool empty;
    KDtreeIndexed* tree;
    size_t nrpts;
};

class BkdTreeIndexed : public SearchTree
{
public:
    BkdTreeIndexed(double **pts,
            int n,
            int bucketSize = 20);

    BkdTreeIndexed(int bucketSize = 20);

    virtual ~BkdTreeIndexed();

    // BKD specifics

    int insert(double*);

    int remove(double*);

    std::vector<size_t> collectPts() const;

    size_t size() const;

    std::string info() const;

    std::string _debug_info() const;

    friend std::ostream& operator<<(std::ostream&, const BkdTreeIndexed&);

    // SearchTree inheritance

    virtual double* FindClosest(double *_p,
                                double maxdist2,
                                int threadNum = 0) const;

    virtual std::vector<size_t> fixedRangeSearchAlongDir(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum = 0) const;

    virtual std::vector<size_t> fixedRangeSearchBetween2Points(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum = 0) const;

    virtual std::vector<size_t> kNearestNeighbors(double *_p,
								int k,
								int threadNum = 0) const;

    virtual std::vector<size_t> kNearestRangeSearch(double *_p,
								int k,
								double sqRad2,
								int threadNum = 0) const;

    virtual std::vector<size_t> fixedRangeSearch(double *_p,
								double sqRad2,
								int threadNum = 0) const;

    virtual std::vector<size_t> AABBSearch(double *_p,
                                double* _p0,
                                int threadNum = 0) const;

    virtual size_t segmentSearch_1NearestPoint(double *_p,
                                double* _p0,
                                double maxdist2,
                                int threadNum) const;

private:
    // Saves all the sub-trees (thus is called 'forest')
    std::vector<ForestElemIndexed> forest;
    std::vector<size_t> buffer;
    int bucketSize;

    void mergeTreesLogarithmic(int, int);
};

#endif // __BKD_H

