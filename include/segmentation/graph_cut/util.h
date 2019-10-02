 /*
 * Copyright (C) David Redondo
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __GRAPH_CUT_UTIL_H__
#define __GRAPH_CUT_UTIL_H__

#include "segmentation/graph_cut/david_graph.h"
#include "segmentation/graph_cut/graph_cut.h"

#include <opencv2/core/core.hpp>

#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
    #include <direct.h>
    #define mkdir(path,mode) _mkdir (path)
#else
    #include <sys/stat.h>
#endif

template<typename T>
void print_matrix_region(const cv::Mat& mat,int begin_row, int end_row, int begin_col, int end_col)
{
    for(int row = begin_row; row < end_row; ++row) {
        for(int col = begin_col; col < end_col; ++col) {
            std::cout << mat.at<T>(row,col) << ";";
        }
        std::cout << '\n';
    }
}

double calc_plane(const std::set<std::pair<int, int>>& indizes, const cv::Mat& mat, double plane[4]);
double calc_plane(const std::vector<const cv::Vec3f*>& points,  double plane[4]);

/*Returns an color in RGB format {red, green, blue} */
std::array<unsigned char, 3> get_color(int index);

struct plane_candidate {
    cv::Vec3d normal;
    double distance;
    double std_error;
    std::vector<const cv::Vec3f*> points;
};

template <typename T, typename U>
double plane_point_distance(const cv::Vec<T, 3>& x, const cv::Vec<U, 3>& n, double p)
{
    return x[0]*n[0] + x[1]*n[1] + x[2]*n[2] - p;
}

void write_segments_to_scans(const std::vector<segment>& segments, const std::string& path, size_t start_index);

void write_segments_to_rgb_scan(const std::vector<segment>& segments, const std::string& path, size_t scan_num);

void write_components_to_scans(const cv::Mat& range_image, const std::vector<david_graph::grid_graph<double>>& components,
                               const std::string& path, size_t minimum_points);

template <typename T, int n>
std::string cvVecnT_string(const cv::Vec<T, n>& v)
{
    std::string s = '(' + std::to_string(v[0]);
    for(size_t i = 1; i < n; ++i) {
        s += ", " + std::to_string(v[i]);
    }
    s += ')';
    return s;
}


#endif
