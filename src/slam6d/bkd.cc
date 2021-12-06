#include "slam6d/bkd.h"
#include <list>

BkdTree::BkdTree(int bucketsize)
{
    // Init buffer mem
    this->bucketSize = bucketsize;
    buffer.reserve(bucketsize); // keeps size at 0, reserves memory
}

BkdTree::BkdTree(double **pts,
                 int n,
                 int bucketsize)
                 : BkdTree(bucketsize)
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
        ForestElem sprout;
        forest.push_back( sprout );
    }
    ForestElem tree(new KDtree(pts, n, bucketsize), n);
    forest.push_back( tree );
}

BkdTree::~BkdTree()
{
    for (uint i = 0; i < forest.size(); ++i)
        forest.at(i).tree->~KDtree();
    forest.clear();
    buffer.clear();
}

std::vector<double*> BkdTree::collectPts(int threadNum) const
{
    std::vector<double*> res;
    for (size_t i = 0; i < forest.size(); ++i) {
        if (forest.at(i).empty) continue;
        std::vector<double*> tree_res = forest.at(i).tree->CollectPts(threadNum);
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
std::string BkdTree::info() const
{
    std::string info_str;
    std::stringstream info_ss;
    info_ss << "============= BKD INFO ===============" << std::endl;
    info_ss << "Buffer [" << std::to_string(buffer.size())
        << "/" << std::to_string(bucketSize) << "]" << std::endl;
    for (size_t i = 0; i < forest.size(); ++i) {
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
std::string BkdTree::_debug_info() const
{
    std::string info_str;
    std::stringstream info_ss;
    //std::cout << "Collecting Buffer" << std::endl;
    //std::cout.flush();
    info_ss << "============== BKD DEBUG ==============" << std::endl;
    info_ss << "Buffer [" << std::to_string(buffer.size())
        << "/" << std::to_string(bucketSize) << "]" << std::endl;
    for (size_t i = 0; i < buffer.size(); ++i) {
        for (int j = 0; j < 3; ++j)
            info_ss << buffer[i][j] << " ";
        info_ss << std::endl;
    }
    //std::cout << "Buffer collected" << std::endl;
    //std::cout.flush();
    for (size_t i = 0; i < forest.size(); ++i) {
        info_ss << "Tree " << std::to_string(i) << ": "
            << std::to_string(forest.at(i).nrpts) << " pts." << std::endl;
        if (forest.at(i).empty) continue;
        KDtree *t = forest.at(i).tree;
        std::vector<double*> data = t->CollectPts();

        for (size_t j = 0; j < data.size(); ++j) {
            for (int k = 0; k < 3; ++k)
                info_ss << data.at(j)[k] << " ";
            info_ss << std::endl;
        }
    }
    info_ss << "========================================" << std::endl;
    info_str = info_ss.str();
    return info_str;
}

size_t BkdTree::size() const
{
    size_t res = 0;
    res += buffer.size();
    for (size_t i = 0; i < forest.size(); ++i) 
        if(!forest.at(i).empty) 
            res += forest.at(i).nrpts;
    return res;
}

std::ostream& operator<<(std::ostream &out, const BkdTree &t)
{
    out << t.info().c_str();
    return out;
}

/**
 *  @brief Insert point <pt> into the Bkd tree.
 *  Keeps the tree balanced and fully space utilized.
 */
void BkdTree::insert(double *pt, int threadNum)
{
    // Push point to the buffer
    #pragma omp critical
    buffer.push_back(pt);

    int b1 = buffer.size();
    int b2 = bucketSize;
    //std::cout << "Buffer: " << b1 << ", Bucket: " << b2 << std::endl;
    if (buffer.size() != bucketSize) return;
    // If the buffer is full
    if ( b1 == b2 ) {

        //std::cout << "MERGING NOW!" << std::endl;
        //std::cout << buffer.size() << " == " << bucketSize << std::endl;
        int nrnewpts = buffer.size();
        bool merged = false;

        // Search first free tree:
        // Iterate the forest
        for (int i = 0; i < forest.size(); ++i) {

            // Until you find an empty tree
            if ( forest.at(i).empty ) {
                // Merge all trees into the empty tree:
                mergeTreesLogarithmic(i, nrnewpts, threadNum);
                merged = true;
                break;
            }
            // If current tree is not empty, iterate further and store nrpts
            else
            {
                int npt = forest.at(i).nrpts;
                nrnewpts += npt;
            }
        }

        // All trees in the forest are already full
        if (!merged)
        {
            // add new elements
            int lastIndex = forest.size();
            ForestElem sprout; // just an empty dummy
            forest.push_back(sprout);
            // use logarithmic merging to get all prev trees into the new one
            mergeTreesLogarithmic( lastIndex, nrnewpts, threadNum );
        }

        // Clear buffer when point is inserted
        buffer.clear();
        //std::cout << "Cleared Buffer and finish." << std::endl;
        return;
    }
}

int BkdTree::remove(double* pt, int threadNum)
{
    // Search if the point is inside the buffer
    for (size_t i = 0; i < buffer.size(); ++i) {
        double d2 = Dist2(pt, buffer[i]);
        if (d2 < 0.000000001) {
            buffer.erase( buffer.begin() + i );
            return 1;
        }
    }
    // Traverse the trees if the point is not inside the buffer
    for (size_t i = 0; i < forest.size(); ++i) {
        if (!forest.at(i).empty) {
            if (forest.at(i).tree->Remove(pt, threadNum)) {
                forest.at(i).nrpts--;
                if(forest.at(i).nrpts == 0)
                    forest.at(i).empty = true;
                return 1;
            }
        }
    }
    return 0;
}

double *BkdTree::FindClosest(double *_p,
                                double maxdist2,
                                int threadNum) const
{
    double *res = 0;
    double *tmp;
    double minDist2 = __DBL_MAX__;
    double dist2;
    // Search all the trees, pick best closest
    for (size_t i = 0; i < forest.size(); ++i) {

        // Skip the empty trees.
        if (forest.at(i).empty) continue;
        KDtree *tree = forest.at(i).tree;
        tmp = tree->FindClosest(_p, maxdist2, threadNum);
        dist2 = Dist2( tmp, _p );
        if ( dist2 < minDist2 ) {
            minDist2 = dist2;
            res = tmp;
        }
    }
    // Also search the buffer if there is any nearer point
    for (size_t i = 0; i < buffer.size(); ++i) {
        dist2 = Dist2( buffer[i] , _p);
        if (dist2 < minDist2) {
            minDist2 = dist2;
            res = buffer[i];
        }
    }
    return res;
}

std::vector<Point> BkdTree::fixedRangeSearchAlongDir(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum) const
{

}

std::vector<Point> BkdTree::fixedRangeSearchBetween2Points(double *_p,
                                double *_dir,
                                double maxdist2,
                                int threadNum) const
{
    //TODO: implement this.
}

std::vector<Point> BkdTree::kNearestNeighbors(double *_p,
								int k,
								int threadNum) const
{
    std::list<Point> candidates;
    // Include k nearest from all the trees
    for(size_t i = 0; i < forest.size(); ++i) {
        if (forest.at(i).empty) continue;
        KDtree *t = forest.at(i).tree;
        std::vector<Point> kNNs = t->kNearestNeighbors(_p, k, threadNum);
        candidates.insert( end(candidates), begin(kNNs), end(kNNs) );
    }
    // Include the buffer
    for(size_t i = 0; i < buffer.size(); ++i)
        candidates.push_back( Point(buffer[i]) );
    // Sort candidates by their distance to the query point _p
    candidates.sort(
        [&](const Point &p1, const Point &p2) -> bool {
            return sqr(p1.x-_p[0])+sqr(p1.y-_p[1])+sqr(p1.z-_p[2])
                 < sqr(p2.x-_p[0])+sqr(p2.y-_p[1])+sqr(p2.z-_p[2]);
        }
    );
    // Pick first k points
    std::vector<Point> result;
    std::list<Point>::iterator beg = candidates.begin();
    k = std::min(k, (int)candidates.size());
    for(int i = 0; i < k; ++i) {
        result.push_back( *beg );
        ++beg;
    }
    return result;
}

std::vector<Point> BkdTree::kNearestRangeSearch(double *_p,
								int k,
								double sqRad2,
								int threadNum) const
{
    //TODO: implement this.
}

std::vector<Point> BkdTree::fixedRangeSearch(double *_p,
								double sqRad2,
								int threadNum) const
{
    std::vector<Point> allPts;
    for (size_t i = 0; i < forest.size(); ++i) {
        if (forest.at(i).empty) continue;
        KDtree *t = forest.at(i).tree;
        std::vector<Point> pts = t->fixedRangeSearch(_p, sqRad2, threadNum);
        allPts.insert( end(allPts), begin(pts), end(pts) );
    }
    for (size_t i = 0; i < buffer.size(); ++i) {
        if (Dist2(buffer[i], _p) <= sqRad2)
            allPts.push_back(buffer[i]);
    }
    return allPts;
}

/**
 * @brief Axis-aligned bounding box search.
 * @param _p: Minimum x, y, z values for the bounding box.
 * @param _p0: Maximum x, y, z values for the bouding box.
 * @return List of points that are inside the boudning box.
 */
std::vector<Point> BkdTree::AABBSearch(double *_p,
                                double* _p0,
                                int threadNum) const
{
    //TODO: implement this.
}

double *BkdTree::segmentSearch_1NearestPoint(double *_p,
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
void BkdTree::mergeTreesLogarithmic(int index, int nrpts, int threadNum)
{
    //std::cout << "Setting up new mem for pts" << std::endl;
        // setup new points memory
    double **newpts = new double*[nrpts];
    //std::cout << "Done." << std::endl;

        // copy the points
    int ptindex = 0;
    for (int k = 0; k < index; ++k) {

        //std::cout << "Getting pts for tree " << k << std::endl; 
        if (forest.at(k).empty) continue;
        // else collect and copy pts.
        std::vector<double*> datak = forest.at(k).tree->CollectPts(threadNum);
        //std::cout << "done." << std::endl;

        //std::cout << "copying points" << std::endl;
        for (size_t q = 0; q < forest.at(k).nrpts; ++q) {
            newpts[ptindex] = datak[q];
            ptindex++;
        }
        //std::cout << "done" << std::endl;

        // Free previous data fields
        forest.at(k).nrpts = 0;
        forest.at(k).empty = true;
        //datak.clear();
    }

        // add the points in the buffer
        //std::cout << "Getting points from the buffer " << std::endl;
    for (int i = 0; i < buffer.size(); ++i) {
        newpts[ptindex] = buffer.at(i);
        ptindex++;
    }
        //std::cout << "Done" << std::endl;

        // Allocate new points
        //std::cout << "Setting up new tree" << std::endl;
    forest.at(index).tree = new KDtree(newpts, nrpts, bucketSize);
    //std::cout << "done" << std::endl;
    //std::cout << "Updating tree fields" << std::endl;
    forest.at(index).nrpts = nrpts;
    forest.at(index).empty = nrpts == 0;
    //std::cout << "end merge." << std::endl;
    return;
}
