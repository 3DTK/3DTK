 /*
 * Copyright (C) David Redondo
 *
 * Released under the GPL version 3.
 *
 */

#include "segmentation/graph_cut/blob_color.h"

#include <fstream>

void my_grid::put(const cv::Vec3f* v, int row, int col)
{
    grid[{row, col}] = v;
    if(row < min_row) min_row = row;
    if(row > max_row) max_row = row;
    if(col < min_col) min_col = col;
    if(col > max_col) max_col = col;
}

void my_grid::write_to_ppm(const std::string& path)
{
    std::ofstream picture(path);
    picture << "P1\n" << (max_col-min_col)+1 << " " << (max_row-min_row)+1 << '\n';
    for(int row = min_row; row <= max_row; ++row) {
        for(int col = min_col; col <= max_col; ++col) {
            int color = (grid[{row, col}] ? 1 : 0);
            picture << color  << " ";
        }
        picture << '\n';
    }
    picture.close();


}

inline std::pair<int, int> grid_coordinates(const cv::Vec3d& p, const cv::Vec3d& pv1, const cv::Vec3d& pv2)
{
    /* Operations could be written manually if performance is a concern*/
    return {int(p.dot(pv1)/(pv1.dot(pv1))), int(p.dot(pv2)/(pv2.dot(pv2)))};
}





std::vector<segment> segment_plane(const plane_candidate& plane, double cell_size)
{

    std::vector<cv::Vec3d> projected_points(plane.points.size());
    /*Projects points onto the plane*/
    auto project = [&](const cv::Vec3f* p){return cv::Vec3d{*p}- plane.normal*plane_point_distance(*p, plane.normal, plane.distance);};
    std::transform(plane.points.begin(), plane.points.end(), projected_points.begin(), project);
    my_grid grid;
    /*get the vectors on the plane and scale them to the grid size*/
    cv::Vec3d a = plane.normal*plane.distance - projected_points[0];
    a = a * cell_size/ cv::norm(a);
    cv::Vec3d b = a.cross(plane.normal); // b has now length grid_size, since <a,n = 90°
    for(size_t i = 0; i < projected_points.size(); ++i) {
        auto coordinates = grid_coordinates(projected_points[i] , a, b);
        grid.put(plane.points[i], coordinates.first, coordinates.second);
    }
    std::map<std::pair<int, int>, int> region_map;
    std::map<int, int> regions;
    int next_region = 0;
    int num_regions = 0;
    for(int row = grid.min_row; row <= grid.max_row; ++row) {
        for(int col = grid.min_col; col <= grid.max_col; ++col) {
            const cv::Vec3f* xu  = grid.get(row-1, col);
            const cv::Vec3f* xc  = grid.get(row, col);
            const cv::Vec3f* xl  = grid.get(row, col-1);
            if(xc){
                if(!xl &&  !xu) {
                    region_map[{row, col}] = next_region;
                    regions[next_region] = next_region;
                    ++next_region;
                    ++num_regions;
                } else if(!xl && xu) {
                    region_map[{row, col}] = region_map[{row-1, col}];
                } else if(xl && !xu) {
                    region_map[{row, col}] = region_map[{row, col-1}];
                } else {
                    int l_region = regions[region_map[{row, col-1}]];
                    int u_region = regions[region_map[{row-1, col}]];
                    if(l_region != u_region) {
                        --num_regions;
                        if(l_region < u_region) std::swap(l_region, u_region);
                        int tmp = l_region;
                        region_map[{row, col-1}] = u_region;
                        auto it = regions.begin();
                        while(it != regions.end()) {
                            if(it->second == tmp) regions[it->first] = regions[u_region];
                            ++it;
                        }
                    }
                    region_map[{row, col}] = u_region;
                }
            }
        }
    }

    std::map<int, int> indices;
    int next_index = 0;
    std::vector<segment> segments(num_regions, {{}, plane.normal, plane.distance});
    /* Looping though all points is necessary because the the points in the grid represent not all points (
     * if multiple points fall into the same cell) */
    for(size_t i = 0; i < plane.points.size(); ++i){
        int region = regions[region_map[grid_coordinates(projected_points[i], a, b)]];
        if(indices.find(region) == indices.end()) {
            indices[region] = next_index++;
        }
        segments[indices[region]].points.emplace_back(*plane.points[i]);

    }
    if (gcs_debug) {
        static int i=1;
        grid.write_to_ppm("debug/grids/grid"+std::to_string(i)+".ppm");
        std::ofstream picture("debug/grids/grid"+std::to_string(i)+"_labels.ppm", std::ios::binary);
        picture << "P6\n" << (grid.max_col-grid.min_col)+1 << " " << (grid.max_row-grid.min_row)+1 << "\n255\n";
        for(int row = grid.min_row; row <= grid.max_row; ++row) {
            for(int col = grid.min_col; col <= grid.max_col; ++col) {
                unsigned char c = 0xFF; std::array<unsigned char, 3> color{{c, c, c}};
                if(grid.get(row, col)){
                    int region = regions[region_map[{row, col}]];
                    color =  get_color(indices[region]);
                }
                picture.write((char*)(&color[0]),3);
                picture.flush();

            }
        }
        picture.close();
        ++i;
    }
    return segments;
}

