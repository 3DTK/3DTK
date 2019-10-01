 /*
 * Copyright (C) David Redondo
 *
 * Released under the GPL version 3.
 *
 */

/**Implementation of the segmentation algorithm desrcibed by
 * Yu et al. 2008: Think Globally, Cluster Locally: A Unified Framework for Range Segmenation
 */
#ifndef __GRAPH_CUT_H__
#define __GRAPH_CUT_H__

#include <slam6d/fbr/scan_cv.h>

#include <opencv2/core/core.hpp>

#include <vector>

struct segment
{
    std::vector<cv::Vec3f> points;
    cv::Vec3d normal;
    double distance;
};

/**
 * Returns vector of segments with the first element being points not belonging to any segment
 */
std::vector<segment> segment_scan(fbr::scan_cv& scan, size_t panorama_width, size_t panorama_height, size_t window_size,
                                  size_t minimum_points, double tau, double cell_size);

extern bool gcs_verbose;
extern bool gcs_debug;

#endif
