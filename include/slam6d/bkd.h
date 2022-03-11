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

#ifndef __BKD_H_
#define __BKD_H_

#include "slam6d/kd.h"
#include "slam6d/searchTree.h"
#include <vector>
#include <memory>

// This data structure stores trees.
// The trees themselfes do not have any pts stored.
// Instead, they operate on the "ForestElem" structure by reorganisation.
struct ForestElem
{
    // Create an empty Forest element, i.e. an empty tree (called 'sprout')
    ForestElem() : empty(true), nrpts(0) {} // empty constructor, we use c++ init list

    // Create a non-empty Forest element, i.e. a KD-tree (called 'tree')
    ForestElem(KDtree* t, size_t n) : empty(false), tree(t), nrpts(n) {}

    // Struct data
    bool empty;
    KDtree* tree;
    size_t nrpts;
};

class BkdTree : public SearchTree
{
public:
    BkdTree(double **pts,
            int n,
            int bucketSize = 20);

    BkdTree(int bucketSize = 20);

    virtual ~BkdTree();

    // BKD specifics

    // Insert multiple points
    void insert(std::vector<double*>&, int threadNum = 0);

    // Insert a single point
    void insert(double*, int threadNum = 0);

    /** @brief: Remove a point, but keeps the point in memory.
     * Works by reorganization of points in the leaf array.
     * Swaps the point to be removed with the last element in the leaf,
     * then decrements the leaf size.
     * @param pt: The coordinates of a point to be removed.
     * @param threadNum: The identifier of the thread (use omp_set_num_threads(...) first)
     * @result: The number of points that actually got removed.
     */
    int remove(double*, int threadNum= 0);

    std::vector<double*> collectPts(int threadNum = 0) const;

    size_t size() const;

    std::string info() const;

    std::string _debug_info() const;

    friend std::ostream& operator<<(std::ostream&, const BkdTree&);

    // SearchTree inheritance

    virtual double *FindClosest(double *_p,
                                double maxdist2,
                                int threadNum = 0) const;

    virtual double *FindClosestAlongDir(double *_p,
                                        double *_dir,
                                        double maxdist2,
                                        int threadNum = 0) const;

    virtual std::vector<Point> fixedRangeSearchAlongDir(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum = 0) const;

    virtual std::vector<Point> fixedRangeSearchBetween2Points(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum = 0) const;

    virtual std::vector<Point> kNearestNeighbors(double *_p,
								int k,
								int threadNum = 0) const;

    virtual std::vector<Point> kNearestRangeSearch(double *_p,
								int k,
								double sqRad2,
								int threadNum = 0) const;

    virtual std::vector<Point> fixedRangeSearch(double *_p,
								double sqRad2,
								int threadNum = 0) const;

    virtual std::vector<Point> AABBSearch(double *_p,
                                double* _p0,
                                int threadNum = 0) const;

    virtual double *segmentSearch_1NearestPoint(double *_p,
                                double* _p0,
                                double maxdist2,
                                int threadNum) const;

private:
    // Saves all the sub-trees (thus is called 'forest')
    std::vector<ForestElem> forest;
    std::vector<double*> buffer;
    int bucketSize;

    void mergeTreesLogarithmic(int, int, int threadNum = 0);
};

#endif // __BKD_H

