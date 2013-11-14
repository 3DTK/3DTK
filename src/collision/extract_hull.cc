/*
 * collision_sphere
 *
 * Copyright (C) Johannes Schauer
 *
 * Released under the GPL version 3.
 *
 */

#include <fstream>
#include <sstream>
#include <string>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

struct ParsingException : public std::exception
{
   std::string s;
   ParsingException(std::string ss) : s(ss) {}
   ~ParsingException() throw () {} // Updated
   const char* what() const throw() { return s.c_str(); }
};

void parse_options(int argc, char **argv, std::string &filename)
{
    po::options_description generic("Generic options");
    generic.add_options()
        ("help,h", "output this help message");

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("input", po::value<std::string>(&filename), "input");

    po::options_description all;
    all.add(generic).add(hidden);

    po::options_description cmdline_options;
    cmdline_options.add(generic);

    po::positional_options_description pd;
    pd.add("input", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vm);
    po::notify(vm);

    // display help
    if (vm.count("help")) {
        std::cout << cmdline_options;
        std::cout << "\nExample usage:\n"
             << "\t" << argv[0] << " input.obj\n";
        exit(0);
    }

    if (!vm.count("input")) {
        std::cout << "you have to specify an input file" << std::endl;
        exit(1);
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

void read_obj(std::string filename, std::vector<double*> &vertices, std::vector<size_t*> &faces)
{
    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)) {
        if (line.substr(0, 1) == "#" || line.length() == 0)
            continue;
        std::vector<std::string> tokens = split(line, ' ');
        if (tokens.size() <= 1) {
            throw ParsingException("only one token in this line");
        }
        if (tokens[0] == "mtllib" || tokens[0] == "usemtl" || tokens[0] == "vt") {
            std::cerr << "ignoring " << tokens[0] << " statement" << std::endl;
        } else if (tokens[0] == "v") {
            if (tokens.size() != 4)
                throw ParsingException("only supporting xyz coordinates");
            double x = std::stod(tokens[1]);
            double y = std::stod(tokens[2]);
            double z = std::stod(tokens[3]);
            double *vertex = new double[3]{x,y,z};
            vertices.push_back(vertex);
        } else if (tokens[0] == "f") {
            if (tokens.size() != 4)
                throw ParsingException("only supporting triangle faces");
            size_t v1 = std::stoul(split(tokens[1],'/')[0])-1;
            size_t v2 = std::stoul(split(tokens[2],'/')[0])-1;
            size_t v3 = std::stoul(split(tokens[3],'/')[0])-1;
            size_t *triangle = new size_t[3]{v1,v2,v3};
            faces.push_back(triangle);
        } else {
            throw ParsingException("no support for type "+tokens[0]);
        }
    }
}

/*
 * test if ray from p into direction d intersects with triangle v0, v1, v2
 *
 * algorithm from:
 *
 *     Möller, T., & Trumbore, B. (1997). Fast, minimum storage ray-triangle
 *     intersection. Journal of graphics tools, 2(1), 21-28.
 */
#define EPSILON 0.000001
#define CROSS(dest,v1,v2) \
    dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
    dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
    dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
#define SUB(dest,v1,v2) \
    dest[0]=v1[0]-v2[0]; \
    dest[1]=v1[1]-v2[1]; \
    dest[2]=v1[2]-v2[2];
int intersect_triangle(double *orig, double *dir,
                       double *vert0, double *vert1, double *vert2)
{
    double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    double det,inv_det;
    double u, v; // the intersection point with the triangle in terms of
                 // multiples of edge1 and edge2, respectively

    /* find vectors for two edges sharing vert0 */
    SUB(edge1, vert1, vert0);
    SUB(edge2, vert2, vert0);
    /* begin calculating determinant - also used to calculate U parameter */
    CROSS(pvec, dir, edge2);
    /* if determinant is near zero, ray lies in plane of triangle */
    det = DOT(edge1, pvec);
    if (det > -EPSILON && det < EPSILON)
        return 0;
    inv_det = 1.0 / det;
    /* calculate distance from vert0 to ray origin */
    SUB(tvec, orig, vert0);
    /* calculate U parameter and test bounds */
    u = DOT(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return 0;
    /* prepare to test V parameter */
    CROSS(qvec, tvec, edge1);
    /* calculate V parameter and test bounds */
    v = DOT(dir, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return 0;
    return 1;
}

void get_concave_hull(std::vector<double*> const &vertices, std::vector<size_t*> const &faces)
{
    std::cout << vertices.size() << std::endl;
    std::cout << faces.size() << std::endl;
    std::vector<bool> hull(vertices.size());
    /* for all vertices, test them against all faces */
    for (size_t i = 0; i < vertices.size(); ++i) {
        int intersecting_triangles = 0;
        for (std::vector<size_t*>::const_iterator it = faces.begin(); it != faces.end(); ++it) {
            double *dir = new double[3]{0.0, 1.0, 0.0};
            /* dont check the face that this vertex is part of */
            if ((*it)[0] == i || (*it)[1] == i || (*it)[2] == i)
                continue;
            intersecting_triangles += intersect_triangle(vertices[i], dir, vertices[(*it)[0]], vertices[(*it)[1]], vertices[(*it)[2]]);
            delete[] dir;
        }
        hull[i] = intersecting_triangles%2 == 0;
    }
    for (std::vector<size_t*>::const_iterator it = faces.begin(); it != faces.end(); ++it) {
        if (hull[(*it)[0]] && hull[(*it)[1]] && hull[(*it)[2]]) {
        } else if (!hull[(*it)[0]] && !hull[(*it)[1]] && !hull[(*it)[2]]) {
        } else {
            std::cout << "irregular" << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    std::string filename;
    parse_options(argc, argv, filename);
    std::vector<double*> vertices;
    std::vector<size_t*> faces;
    read_obj(filename, vertices, faces);
    get_concave_hull(vertices, faces);
}
