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
#include <math.h>
#include <unordered_set>
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
        std::cerr << cmdline_options;
        std::cerr << "\nExample usage:\n"
             << "\t" << argv[0] << " input.obj\n";
        exit(0);
    }

    if (!vm.count("input")) {
        std::cerr << "you have to specify an input file" << std::endl;
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

/*
struct edge_hash {
    inline std::size_t operator()(const std::pair<size_t,size_t> & v) const {
        return v.first^v.second; // this works because v.first != v.second
    }
};
*/

void fill_datastructures(std::vector<double*> const &vertices,
        std::vector<size_t*> const &faces,
        std::map<std::pair<size_t, size_t>, std::vector<std::pair<size_t,size_t>>> &edgetofaces,
        std::vector<std::unordered_set <size_t>> &vertextoneighbors)
{
    for (auto it = vertices.begin(); it != vertices.end(); ++it) {
        std::unordered_set<size_t> m;
        vertextoneighbors.push_back(m);
    }
    for (size_t i = 0; i < faces.size(); ++i) {
        // since edges are undirected, create edges for both directions
        std::pair<size_t, size_t> edge1a = std::make_pair(faces[i][0], faces[i][1]);
        std::pair<size_t, size_t> edge2a = std::make_pair(faces[i][1], faces[i][2]);
        std::pair<size_t, size_t> edge3a = std::make_pair(faces[i][2], faces[i][0]);
        std::pair<size_t, size_t> edge1b = std::make_pair(faces[i][1], faces[i][0]);
        std::pair<size_t, size_t> edge2b = std::make_pair(faces[i][2], faces[i][1]);
        std::pair<size_t, size_t> edge3b = std::make_pair(faces[i][0], faces[i][2]);
        // fill edgetofaces
        edgetofaces[edge1a].push_back(std::make_pair(i,faces[i][2]));
        edgetofaces[edge2a].push_back(std::make_pair(i,faces[i][0]));
        edgetofaces[edge3a].push_back(std::make_pair(i,faces[i][1]));
        edgetofaces[edge1b].push_back(std::make_pair(i,faces[i][2]));
        edgetofaces[edge2b].push_back(std::make_pair(i,faces[i][0]));
        edgetofaces[edge3b].push_back(std::make_pair(i,faces[i][1]));
        // fill vertextoneighbors
        vertextoneighbors[faces[i][0]].insert(faces[i][1]);
        vertextoneighbors[faces[i][0]].insert(faces[i][2]);
        vertextoneighbors[faces[i][1]].insert(faces[i][0]);
        vertextoneighbors[faces[i][1]].insert(faces[i][2]);
        vertextoneighbors[faces[i][2]].insert(faces[i][1]);
        vertextoneighbors[faces[i][2]].insert(faces[i][0]);
    }
}

double* calculatenormalofface(std::vector<double*> const &vertices, std::vector<size_t*> const &faces, size_t face)
{
    // n = e1 x e2 = (v0-v1) x (v0-v2)
    // we assume that vertices are arranged counterclockwise
    double e10 = vertices[faces[face][0]][0] - vertices[faces[face][1]][0];
    double e11 = vertices[faces[face][0]][1] - vertices[faces[face][1]][1];
    double e12 = vertices[faces[face][0]][2] - vertices[faces[face][1]][2];
    double e20 = vertices[faces[face][0]][0] - vertices[faces[face][2]][0];
    double e21 = vertices[faces[face][0]][1] - vertices[faces[face][2]][1];
    double e22 = vertices[faces[face][0]][2] - vertices[faces[face][2]][2];
    return new double[3]{e11*e22-e12*e21, e12*e20-e10*e22, e10*e21-e11*e20};
}

double calculateanglebetweenfaces(std::vector<double*> const &vertices,
        std::vector<size_t*> const &faces,
        size_t face1, size_t face2, std::pair<size_t,size_t> const &edge)
{
    // calculate normal vector of faces
    double *n1 = calculatenormalofface(vertices, faces, face1);
    double *n2 = calculatenormalofface(vertices, faces, face2);
    // calculate angle between two normals
    double a = acos(
            (n1[0]*n2[0]+n1[1]*n2[1]+n1[2]*n2[2])/
            (sqrt(n1[0]*n1[0]+n1[1]*n1[1]+n1[2]*n1[2])*
             sqrt(n2[0]*n2[0]+n2[1]*n2[1]+n2[2]*n2[2])));
    // calculate cross product of normals
    double *c = new double[3]{
            n1[1]*n2[2]-n1[2]*n2[1],
            n1[2]*n2[0]-n1[0]*n2[2],
            n1[0]*n2[1]-n1[1]*n2[0]
    };
    // compare that cross product to one of the edges
    // the direction of the edge determines which way round the angle is
    // generated
    double *vn = new double[3]{
            vertices[edge.first][0] - vertices[edge.second][0],
            vertices[edge.first][1] - vertices[edge.second][1],
            vertices[edge.first][2] - vertices[edge.second][2]
    };
    double b = c[0]*vn[0]+c[1]*vn[1]+c[2]*vn[2];
    if (b < 0) {
        a = 2*M_PI-a;
    }
    delete[] n1;
    delete[] n2;
    delete[] c;
    delete[] vn;
    return a;
}

void highestfacesanitycheck(std::vector<double*> const &vertices,
        std::vector<size_t*> const &faces,
        size_t highestface,
        int axis)
{
    // since we carefully picked the face with the vertices with the highest x
    // values, we now know which side of the face faces "outward"
    // we also rely on the fact that the vertices of each triangle are given
    // in counter clockwise order when the front is facing the viewer - this
    // is the case in the wavefront obj format
    // we check whether the "outward" we think we know agrees with the
    // triangle vertex order
    double *normal = calculatenormalofface(vertices, faces, highestface);
    // now the dot product with the normal and a vector just pointing in x
    // direction. Because x=[1,0,0], the result is just the x component;
    double numerator = normal[axis]; // x*normal
    // the magnitude of x=1, therefore the denominator is only the magnitude
    // of normal
    double denominator = sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]); // ||x||*||normal||
    // the angle is the inverse cosine of the division
    double angle = acos(numerator/denominator)*180.0/M_PI;
    std::cout << "angle between normal and \"up\" vector (should be less than 90 deg): " << angle << std::endl;

}

// this method cannot extract the surface in case that the volumes defined by
// two meshes intersect without sharing vertices
void surfacewalker(std::vector<double*> const &vertices,
        std::vector<size_t*> const &faces,
        std::map<std::pair<size_t, size_t>, std::vector<std::pair<size_t,size_t>>> const &edgetofaces,
        std::vector<std::unordered_set <size_t>> const &vertextoneighbors,
        std::vector<size_t> &surface)
{
    // axis for which to search for the max
    int axis = 1;
    // find an outer vertex
    // take the highest vertex
    size_t highestidx = 0;
    for (size_t i = 1; i < vertices.size(); ++i) {
        if (vertices[i][axis] > vertices[highestidx][axis])
            highestidx = i;
    }
    // of that vertex, find the edge to the vertex which is second highest
    auto vnit = vertextoneighbors[highestidx].begin();
    size_t sndhighestidx = *vnit;
    ++vnit;
    for (; vnit != vertextoneighbors[highestidx].end(); ++vnit) {
        if (vertices[*vnit][axis] > vertices[sndhighestidx][axis])
            sndhighestidx = *vnit;
    }
    // of all faces that this edge is part of, find the one with the highest
    // third vertex
    std::pair<size_t, size_t> highestedge = std::make_pair(highestidx,sndhighestidx);
    auto efit = edgetofaces.at(highestedge).begin();
    size_t highestface = efit->first;
    size_t trdhighestidx = efit->second;
    ++efit;
    for (; efit != edgetofaces.at(highestedge).end(); ++efit) {
        if (vertices[efit->second][axis] > vertices[trdhighestidx][axis]) {
            highestface = efit->first;
            trdhighestidx = efit->second;
        }
    }
    highestfacesanitycheck(vertices, faces, highestface, axis);
    // now that we have the seed, fill the following unordered_set of faces
    // with the result of the following walk
    std::vector<size_t> tocheck;
    std::unordered_set<size_t> checked;
    surface.push_back(highestface);
    tocheck.push_back(highestface);
    checked.insert(highestface);
    while (tocheck.size()) {
        size_t currentface = tocheck.back();
        tocheck.pop_back();
        // go through all edges (counterclockwise) and for each edge add the
        // face with the lowest angle to the current face
        
        size_t v0 = faces[currentface][0];
        size_t v1 = faces[currentface][1];
        size_t v2 = faces[currentface][2];

        for (auto &it: {std::make_pair(v0,v1),
                        std::make_pair(v1,v2),
                        std::make_pair(v2,v0)}) {
            std::vector<std::pair<size_t,size_t>> edgefaces = edgetofaces.at(it);
            auto it1 = edgefaces.begin();
            size_t face = it1->first;
            if (face == currentface) {
                ++it1;
                if (it1 == edgefaces.end()) { // this edge is only connected to a single face
                    continue;
                }
                face = it1->first;
            }
            // only start calculating the angle if this edge is attached to
            // more than two faces - we then need to calculate angles to make
            // a decision
            if (edgefaces.size() > 2) {
                double angle = calculateanglebetweenfaces(vertices, faces, currentface, face, it);
                ++it1;
                for (;it1 != edgefaces.end();++it1) {
                    if (it1->first == currentface) // skip the current face
                        continue;
                    double newangle = calculateanglebetweenfaces(vertices, faces, currentface, it1->first, it);
                    if (newangle < angle) {
                        angle = newangle;
                        face = it1->first;
                    }
                }
            }
            if (checked.count(face)) {
                // this face was already processed: do nothing
            } else {
                surface.push_back(face);
                tocheck.push_back(face);
                checked.insert(face);
            }
        }
    }
    std::cout << "number of faces on surface: " << surface.size() << std::endl;
}

void writeobj(std::vector<double*> const &vertices,
        std::vector<size_t*> const &faces,
        std::vector<size_t> const &surface)
{
    std::ofstream fsurface("surface.obj");
    // vector mapping the old vertex id to the new vertex id
    // because the printed obj might have less vertices
    std::vector<size_t> vertmap;
    vertmap.reserve(vertices.size());
    // write vertices
    size_t numverts = 0;
    for (auto &it : surface) {
        for (auto &it2 : {0,1,2}) {
            size_t vertid = faces[it][it2];
            if (vertmap[vertid] == 0) {
                numverts++;
                vertmap[vertid] = numverts;
                fsurface << "v " << vertices[vertid][0] << " "
                                 << vertices[vertid][1] << " "
                                 << vertices[vertid][2] << std::endl;
            }
        }
    }
    // write faces
    for (auto &it : surface) {
        fsurface << "f " << vertmap[faces[it][0]] << " "
                         << vertmap[faces[it][1]] << " "
                         << vertmap[faces[it][2]] << std::endl;
    }
    fsurface.close();
}

int main(int argc, char** argv)
{
    std::string filename;
    parse_options(argc, argv, filename);
    std::vector<double*> vertices;
    std::vector<size_t*> faces;

    read_obj(filename, vertices, faces);
    std::cerr << "number of vertices: " << vertices.size() << std::endl;
    std::cerr << "number of faces: " << faces.size() << std::endl;

    /* a map mapping an edge (index pair of vertex numbers) to a pair of a face
     * (index of the face) and the vertex number of the face that the edge is
     * not part of */
    std::map<std::pair<size_t, size_t>, std::vector<std::pair<size_t,size_t>>> edgetofaces;
    /* a vector assigning to each index representing a vertex number a set of
     * indices representing the neighbor vertices to which it is connected by
     * edges */
    std::vector<std::unordered_set <size_t>> vertextoneighbors;

    fill_datastructures(vertices, faces, edgetofaces, vertextoneighbors);

    size_t edgeswithmorethantwofaces = 0;
    for (auto it = edgetofaces.begin();
            it != edgetofaces.end(); ++it) {
        if (it->second.size() > 2)
            edgeswithmorethantwofaces++;
    }
    std::cerr << "size of map from edge to face (both edge directions): " << edgetofaces.size() << std::endl;
    std::cerr << "edges with more than two faces: " << edgeswithmorethantwofaces << std::endl;
    std::vector<size_t> surface;
    surfacewalker(vertices, faces, edgetofaces, vertextoneighbors, surface);
    writeobj(vertices, faces, surface);
    //get_concave_hull(vertices, faces);
}
