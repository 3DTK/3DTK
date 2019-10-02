 /*
 * Copyright (C) David Redondo
 *
 * Released under the GPL version 3.
 *
 */

#include "segmentation/graph_cut/util.h"

#include <newmat/newmatap.h>

double calc_plane(const std::set<std::pair<int, int>>& indizes, const cv::Mat& mat, double plane[4])
{
    using namespace NEWMAT;
    SymmetricMatrix A(3);
    A = 0;
    int n;
    n = indizes.size();
    if(n < 3) return 0;
    double cx, cy, cz;
    cx = 0.0;
    cy = 0.0;
    cz = 0.0;

    for (const auto& index : indizes) {
        const cv::Vec3f& p = mat.at<cv::Vec3f>(index.first, index.second);
        cx += p[0];
        cy += p[1];
        cz += p[2];
    }
    cx /= n;
    cy /= n;
    cz /= n;

   for (const auto& index : indizes) {
        const cv::Vec3f& p = mat.at<cv::Vec3f>(index.first, index.second);
        A(1, 1) += (p[0] - cx)*(p[0] - cx);
        A(2, 2) += (p[1] - cy)*(p[1] - cy);
        A(3, 3) += (p[2] - cz)*(p[2] - cz);
        A(1, 2) += (p[0] - cx)*(p[1] - cy);
        A(1, 3) += (p[0] - cx)*(p[2] - cz);
        A(2, 3) += (p[1] - cy)*(p[2] - cz);
    }

    DiagonalMatrix D;
    Matrix V;
    try {
        Jacobi(A,D,V);
    } catch (ConvergenceException) {
        cout << "couldn't find plane..." << endl;
        return 0;
    }
    /*
        cout << "A: "<< endl << A << endl;
        cout << "D: "<< endl << D << endl;
        cout << "V: "<< endl << V << endl;
        */
    int index;
    D.MinimumAbsoluteValue1(index);

    plane[0] = V(1,index);
    plane[1] = V(2,index);
    plane[2] = V(3,index);
    plane[3] = plane[0]*cx + plane[1]*cy + plane[2]*cz;

    double sum = 0.0;
    for(int i = 1; i < 4; i++) {
        sum += D(i);
    }
    sum = D(index)/sum;
    return sum;
}

double calc_plane(const std::vector<const cv::Vec3f*>& points, double plane[4])
{
    using namespace NEWMAT;
    SymmetricMatrix A(3);
    A = 0;
    int n;
    n = points.size();
    if(n < 3) return 0;
    double cx, cy, cz;
    cx = 0.0;
    cy = 0.0;
    cz = 0.0;

    for (const cv::Vec3f* p : points) {
        cx += (*p)[0];
        cy += (*p)[1];
        cz += (*p)[2];
    }
    cx /= n;
    cy /= n;
    cz /= n;

   for (const cv::Vec3f* p : points) {
        A(1, 1) += ((*p)[0] - cx)*((*p)[0] - cx);
        A(2, 2) += ((*p)[1] - cy)*((*p)[1] - cy);
        A(3, 3) += ((*p)[2] - cz)*((*p)[2] - cz);
        A(1, 2) += ((*p)[0] - cx)*((*p)[1] - cy);
        A(1, 3) += ((*p)[0] - cx)*((*p)[2] - cz);
        A(2, 3) += ((*p)[1] - cy)*((*p)[2] - cz);
    }

    DiagonalMatrix D;
    Matrix V;
    try {
        Jacobi(A,D,V);
    } catch (ConvergenceException) {
        cout << "couldn't find plane..." << endl;
        return 0;
    }
    /*
        cout << "A: "<< endl << A << endl;
        cout << "D: "<< endl << D << endl;
        cout << "V: "<< endl << V << endl;
        */
    int index;
    D.MinimumAbsoluteValue1(index);

    plane[0] = V(1,index);
    plane[1] = V(2,index);
    plane[2] = V(3,index);
    plane[3] = plane[0]*cx + plane[1]*cy + plane[2]*cz;

    double sum = 0.0;
    for(int i = 1; i < 4; i++) {
        sum += D(i);
    }
    sum = D(index)/sum;
    return sum;
}

void write_segments_to_scans(const std::vector<segment>& segments, const std::string& path, size_t start_index)
{
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    for(const segment& s : segments) {
        std::ofstream file;
        char numstr[6];
        std::sprintf(numstr, "%03lu", start_index);
        file.open(path + "/scan" + numstr + ".3d");
        file << std::fixed << std::setprecision(4);
        file << '#' << "n:" << cvVecnT_string(s.normal) << " d:" << s.distance << '\n';
        std::ofstream pose_file(path + "/scan" + numstr + ".pose");
        pose_file << "0 0 0\n0 0 0";
        for(const auto& point : s.points) {
            file << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        }
        file.close();
        ++start_index;
    }
}

void write_segments_to_rgb_scan(const std::vector<segment>& segments, const std::string& path, size_t scan_num)
{
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    char numstr[6];
    std::sprintf(numstr, "%03lu", scan_num);
    std::FILE* scan_file = std::fopen((path+"/scan" + numstr + ".3d").c_str(), "w");
    std::ofstream pose_file(path + "/scan" + numstr + ".pose");
    pose_file << "0 0 0\n0 0 0";
    int counter = 0;
    for(const segment& s : segments) {
        std::array<unsigned char, 3> color = get_color(counter);
        for(const auto& point : s.points) {
            std::fprintf(scan_file, "%.4f %.4f %.4f %d %d %d\n", point[0], point[1], point[2], color[0], color[1], color[2]);
        }
        ++counter;
    }
    std::fclose(scan_file);
}

void write_components_to_scans(const cv::Mat& range_image, const std::vector<david_graph::grid_graph<double>>& components,
                               const std::string& path, size_t minimum_points)
{
    mkdir(path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
    std::ofstream(path + "/scan000.3d");
    std::ofstream(path + "/scan000.pose");
    size_t comp_num = 1;
    for(const auto& component : components) {
        size_t num_vertices = component.vertices().size();
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
        for(const std::pair<int, int>& index : component.vertices()) {
            const auto& point = range_image.at<cv::Vec3f>(index.first, index.second);
            file << point[0] << ' ' << point[1] << ' ' << point[2] << '\n';
        }
        file.close();
    }
}
std::array<unsigned char, 3> get_color(int index)
{
    constexpr int colors[] = {
        0xFFFF00, 0x1CE6FF, 0xFF34FF, 0xFF4A46, 0x008941, 0x006FA6, 0xA30059,
        0xFFDBE5, 0x7A4900, 0x0000A6, 0x63FFAC, 0xB79762, 0x004D43, 0x8FB0FF, 0x997D87,
        0x5A0007, 0x809693, 0xFEFFE6, 0x1B4400, 0x4FC601, 0x3B5DFF, 0x4A3B53, 0xFF2F80,
        0x61615A, 0xBA0900, 0x6B7900, 0x00C2A0, 0xFFAA92, 0xFF90C9, 0xB903AA, 0xD16100,
        0xDDEFFF, 0x000035, 0x7B4F4B, 0xA1C299, 0x300018, 0x0AA6D8, 0x013349, 0x00846F,
        0x372101, 0xFFB500, 0xC2FFED, 0xA079BF, 0xCC0744, 0xC0B9B2, 0xC2FF99, 0x001E09,
        0x00489C, 0x6F0062, 0x0CBD66, 0xEEC3FF, 0x456D75, 0xB77B68, 0x7A87A1, 0x788D66,
        0x885578, 0xFAD09F, 0xFF8A9A, 0xD157A0, 0xBEC459, 0x456648, 0x0086ED, 0x886F4C,
        0x34362D, 0xB4A8BD, 0x00A6AA, 0x452C2C, 0x636375, 0xA3C8C9, 0xFF913F, 0x938A81,
        0x575329, 0x00FECF, 0xB05B6F, 0x8CD0FF, 0x3B9700, 0x04F757, 0xC8A1A1, 0x1E6E00,
        0x7900D7, 0xA77500, 0x6367A9, 0xA05837, 0x6B002C, 0x772600, 0xD790FF, 0x9B9700,
        0x549E79, 0xFFF69F, 0x201625, 0x72418F, 0xBC23FF, 0x99ADC0, 0x3A2465, 0x922329,
        0x5B4534, 0xFDE8DC, 0x404E55, 0x0089A3, 0xCB7E98, 0xA4E804, 0x324E72, 0x6A3A4C,
        0x83AB58, 0x001C1E, 0xD1F7CE, 0x004B28, 0xC8D0F6, 0xA3A489, 0x806C66, 0x222800,
        0xBF5650, 0xE83000, 0x66796D, 0xDA007C, 0xFF1A59, 0x8ADBB4, 0x1E0200, 0x5B4E51,
        0xC895C5, 0x320033, 0xFF6832, 0x66E1D3, 0xCFCDAC, 0xD0AC94, 0x7ED379, 0x012C58};
        const int color = colors[index % 127];

        return {{(unsigned char)(color>>16),  (unsigned char)(color>>8),  (unsigned char)(color)}};
}

