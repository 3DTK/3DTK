 /*
 * Copyright (C) David Redondo
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __GRAPH_CUT_BLOB_COLOR_H__
#define __GRAPH_CUT_BLOB_COLOR_H__

#include "segmentation/graph_cut/graph_cut.h"
#include "segmentation/graph_cut/util.h"

#include <opencv2/core/core.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>


class my_grid {

public:
    void put(const cv::Vec3f* v, int row, int col);
    const cv::Vec3f* get(int row, int col) {return grid[{row, col}];};
    int min_row  = 0, max_row = 0, min_col = 0, max_col = 0;
    void write_to_ppm(const std::string& path);
private:
    /*Compare of std::map<K, V, Compare> defaults to std::less<K> which does exactly what we want for pair*/
    std::map<std::pair<int, int>, const cv::Vec3f*> grid;
};

std::vector<segment> segment_plane(const plane_candidate& plane, double cell_size);

#endif
