#ifndef __GRAPH_CUT_UTIL_H__
#define __GRAPH_CUT_UTIL_H__

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

#ifdef _MSC_VER
    #include <direct.h>
    #define mkdir(path,mode) _mkdir (path)
#else
    #include <sys/stat.h>
#endif

struct identity {
    template<typename T>
    constexpr auto operator()(T&& v) const noexcept -> decltype(std::forward<T>(v))
    {
        return std::forward<T>(v);
    }
};

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

/*The three following functions write_* are probably a bit overengineered*/
template <typename PointStructureHolder, typename Header = const char* (*) (PointStructureHolder),
            typename PointStructureAccessor = identity, typename PointAccessor = identity>
void write_scan(const std::vector<PointStructureHolder>& scans, const std::string& path, size_t start_index=0,
            Header head = [](const PointStructureHolder& scan){return "";}, PointAccessor&& get_point = identity(), 
                PointStructureAccessor&& get_point_container = identity())
{
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    for(auto it = scans.cbegin(); it != scans.cend(); ++it) {
        const auto& point_structure = get_point_container(*it);
        std::ofstream file;
        char numstr[6];
        std::sprintf(numstr, "%03lu", start_index);
        file.open(path + "/scan" + numstr + ".3d");
        file << std::fixed << std::setprecision(4) << '#' << head(*it) << '\n';
        std::ofstream pose_file(path + "/scan" + numstr + ".pose");
        pose_file << "0 0 0\n0 0 0";
        for(const auto& point_descriptor : point_structure) {
            const auto& point = get_point(point_descriptor);
            file << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        }
        file.close();
        ++start_index;
    }
}


template <typename PointStructureHolder, typename PointStructureAccessor = identity, typename PointAccessor = identity>
void write_single_color_scan(const std::vector<PointStructureHolder>& scans, const std::string& path, size_t scan_num,
            PointAccessor&& get_point = identity(), PointStructureAccessor&& get_point_container = identity())
{
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    char numstr[6];
    std::sprintf(numstr, "%03lu", scan_num);
    std::FILE* scan_file = std::fopen((path+"/scan" + numstr + ".3d").c_str(), "w");
    std::ofstream pose_file(path + "/scan" + numstr + ".pose");
    pose_file << "0 0 0\n0 0 0";
    int counter = 0;
    for(auto it = scans.cbegin(); it != scans.cend(); ++it) {
        const auto& point_structure = get_point_container(*it);
        std::array<unsigned char, 3> color = get_color(counter);
        for(const auto& point_descriptor : point_structure) {
            const auto& point = get_point(point_descriptor);
            std::fprintf(scan_file, "%.4f %.4f %.4f %d %d %d\n", point[0], point[1], point[2], color[0], color[1], color[2]);
        }
        ++counter;
    }
    std::fclose(scan_file);
}


template <typename PointStructureHolder, typename PointStructureAccessor = identity, typename PointAccessor = identity>
void write_components(const std::vector<PointStructureHolder>& components,  const std::string& path, size_t minimum_points,
                      PointAccessor&& get_point = identity(),
                      PointStructureAccessor&& get_point_container = identity())
{
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    std::ofstream(path + "/scan000.3d");
    std::ofstream(path + "/scan000.pose");
    size_t comp_num = 1;
    for(const auto& component : components) {
        const auto& point_structure = get_point_container(component);
        size_t num_vertices = point_structure.size();
        std::ofstream file;
        if(num_vertices < minimum_points) {
            file.open(path + "/scan000.3d", std::ofstream::app);
        }
        else {
            char numstr[6];
            std::sprintf(numstr, "%03lu", comp_num);
            file.open(path + "/scan" + numstr + ".3d");
            std::ofstream pose_file(path + "/scan" + numstr + ".pose");
            pose_file << "0 0 0\n0 0 0";
            ++comp_num;
        }
        file << std::fixed << std::setprecision(4);
        for(const auto& point_descriptor : point_structure) {
            const auto& point = get_point(point_descriptor);
            file << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        }
        file.close();
    }
}

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
