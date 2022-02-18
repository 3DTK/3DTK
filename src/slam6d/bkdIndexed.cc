#include "slam6d/bkdIndexed.h"
#include <list>

BkdTreeIndexed::BkdTreeIndexed(int bucketsize)
{
    // Init buffer mem
    this->bucketSize = bucketsize;
    buffer.reserve(bucketsize); // keeps size at 0, reserves memory
}

BkdTreeIndexed::BkdTreeIndexed(double **pts,
                 int n,
                 int bucketsize)
                 : BkdTreeIndexed(bucketsize)
{
    // nr of trees needed for n points with given bucketsize
    int nrtrees = ceil(log2(n/bucketsize));
    // Init forrest
    /*
     * this way, all points would fit into the forest.
     * however, we add another tree and put all the points into
     * that large tree, thus all trees before are empty.
     * This way, point insertion starts optimaly
     */
    for(int i = 0; i < nrtrees; ++i) {
        ForestElemIndexed sprout;
        forest.push_back( sprout );
    }
    ForestElemIndexed tree( new KDtreeIndexed(pts, n, bucketsize), n);
    forest.push_back( tree );
}

BkdTreeIndexed::~BkdTreeIndexed()
{
    for (size_t i = 0; i < forest.size(); ++i)
        forest.at(i).tree->~KDtreeIndexed();
    forest.clear();
    buffer.clear();
}

std::vector<size_t> BkdTreeIndexed::collectPts() const
{
    std::vector<size_t> res;
    for (size_t i = 0; i < forest.size(); ++i) {
        if (forest.at(i).empty) continue;
        std::vector<size_t> tree_res = forest.at(i).tree->CollectPts();
        res.insert( std::end(res), std::begin(tree_res), std::end(tree_res) );
    }
    if (buffer.size() > 0)
        res.insert( std::end(res), std::begin(buffer), std::end(buffer) );
    return res;
}

/**
 * Neat for debugging.
 * Shows size of the bkd-tree, i.e. all sub-trees and how many
 * points they include (the buffer also).
 */
std::string BkdTreeIndexed::info() const
{
    std::string info_str;
    std::stringstream info_ss;
    info_ss << "============= BKD INFO ===============" << std::endl;
    info_ss << "Buffer [" << std::to_string(buffer.size())
        << "/" << std::to_string(bucketSize) << "]" << std::endl;
    for (int i = 0; i < forest.size(); ++i) {
        info_ss << "Tree " << std::to_string(i) << ": "
            << std::to_string(forest.at(i).nrpts) << " pts." << std::endl;
    }
    info_ss << "======================================" << std::endl;
    info_str = info_ss.str();
    return info_str;
}

/**
 * Even more neat for debugging.
 * Shows size of the bkd-tree, i.e. all sub-trees and how many
 * points they include (the buffer also).
 * Furthermore, show all points inside the tree (use with care!)
 */
std::string BkdTreeIndexed::_debug_info() const
{
    std::string info_str;
    std::stringstream info_ss;
    info_ss << "============== BKD DEBUG ==============" << std::endl;
    info_ss << "Buffer [" << std::to_string(buffer.size())
        << "/" << std::to_string(bucketSize) << "]" << std::endl;
    for (size_t i = 0; i < buffer.size(); ++i) {
        info_ss << buffer[i] << " ";
    }
    info_ss << std::endl;
    for (size_t i = 0; i < forest.size(); ++i) {
        info_ss << "Tree " << std::to_string(i) << ": "
            << std::to_string(forest.at(i).nrpts) << " pts." << std::endl;
        if (forest.at(i).empty) continue;
        KDtreeIndexed *t = forest.at(i).tree;
        std::vector<size_t> data = t->CollectPts();

        for (size_t j = 0; j < data.size(); ++j) {
            info_ss << data.at(j) << " ";
        }
        info_ss << std::endl;
    }
    info_ss << "========================================" << std::endl;
    info_str = info_ss.str();
    return info_str;
}

size_t BkdTreeIndexed::size() const
{
    size_t res = 0;
    res += buffer.size();
    for (size_t i = 0; i < forest.size(); ++i)
        if(!forest.at(i).empty)
            res += forest.at(i).nrpts;
    return res;
}

std::ostream& operator<<(std::ostream &out, const BkdTreeIndexed &t)
{
    out << t.info().c_str();
    return out;
}

/**
 *  @brief Insert point <pt> into the Bkd tree.
 *  Keeps the tree balanced and fully space utilized.
 */
int BkdTreeIndexed::insert(double *pt)
{
    //  TODO: this is going to be hard.
    // We have to assign a new index for the point <pt> to m_data.
}

int BkdTreeIndexed::remove(double* pt)
{
    // TODO: this is going to be hard.
}

double *BkdTreeIndexed::FindClosest(double *_p,
                                double maxdist2,
                                int threadNum) const
{
    // size_t res = 0;
    // size_t tmp;
    // double minDist2 = __DBL_MAX__;
    // double dist;
    // // Search all the trees, pick best closest
    // for (size_t i = 0; i < forest.size(); ++i) {

    //     // Skip the empty trees.
    //     if (forest.at(i).empty) continue;
    //     KDtreeIndexed *tree = forest.at(i).tree;
    //     tmp = tree->FindClosest(_p, maxdist2, threadNum);
    //     dist = Dist2( tmp, _p );
    //     if ( dist < minDist2 ) {
    //         minDist2 = dist;
    //         res = tmp;
    //     }
    // }
    // // Also search the buffer if there is any nearer point
    // for (size_t i = 0; i < buffer.size(); ++i) {
    //     dist = Dist2( buffer[i] , _p);
    //     if (dist < minDist2) {
    //         minDist2 = dist;
    //         res = buffer[i];
    //     }
    // }
    // return res;
}

std::vector<size_t> BkdTreeIndexed::fixedRangeSearchAlongDir(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum) const
{

}

std::vector<size_t> BkdTreeIndexed::fixedRangeSearchBetween2Points(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum) const
{
    //TODO: implement this.
}

std::vector<size_t> BkdTreeIndexed::kNearestNeighbors(double *_p,
								int k,
								int threadNum) const
{

}

std::vector<size_t> BkdTreeIndexed::kNearestRangeSearch(double *_p,
								int k,
								double sqRad2,
								int threadNum) const
{
    //TODO: implement this.
}

std::vector<size_t> BkdTreeIndexed::fixedRangeSearch(double *_p,
								double sqRad2,
								int threadNum) const
{

}

/**
 * @brief Axis-aligned bounding box search.
 * @param _p: Minimum x, y, z values for the bounding box.
 * @param _p0: Maximum x, y, z values for the bouding box.
 * @return List of points that are inside the boudning box.
 */
std::vector<size_t> BkdTreeIndexed::AABBSearch(double *_p,
                                double* _p0,
                                int threadNum) const
{
    //TODO: implement this.
}

size_t BkdTreeIndexed::segmentSearch_1NearestPoint(double *_p,
                                double* _p0,
                                double maxdist2,
                                int threadNum) const
{
    //TODO: implement this.
}

/**
 *  @brief Merges the trees from index 0 up to index <index>.
 *  Take all the points that are in those trees, delete them, and setup
 *  a new tree with all the deleted points.
 */
void BkdTreeIndexed::mergeTreesLogarithmic(int index, int nrpts)
{

}
