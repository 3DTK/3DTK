/**
 * @file graphicsAlg.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 18 Feb 2012
 *
 */

#ifndef GRAPHICSALG_H_
#define GRAPHICSALG_H_

//==============================================================================
//  Includes
//==============================================================================
#include "model/plane3d.h"
#include <vector>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Filtered_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Triangulation_hierarchy_2.h>
#include <CGAL/Alpha_shape_euclidean_traits_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_hierarchy_vertex_base_2.h>

namespace model {

class GraphicsAlg {
private:
    typedef double Type;
    typedef CGAL::Simple_cartesian<Type>  SC;
    typedef CGAL::Filtered_kernel<SC> K;

    typedef CGAL::Alpha_shape_euclidean_traits_2<K> Gt;
    typedef CGAL::Alpha_shape_vertex_base_2<Gt> Avb;

    typedef CGAL::Triangulation_hierarchy_vertex_base_2<Avb> Av;
    typedef CGAL::Triangulation_face_base_2<Gt> Tf;
    typedef CGAL::Alpha_shape_face_base_2<Gt,Tf> Af;

    typedef CGAL::Triangulation_default_data_structure_2<Gt,Av,Af> Tds;
    typedef CGAL::Delaunay_triangulation_2<Gt,Tds> Dt;
    typedef CGAL::Triangulation_hierarchy_2<Dt> Ht;

    /**
     * Computes the alpha edges.
     */
    template <class InputIterator, class OutputIterator>
    static void alpha_edges(InputIterator begin, InputIterator end,
            const Type& Alpha,
            bool mode,
            OutputIterator out);

public:
    typedef CGAL::Alpha_shape_2<Ht> Alpha_shape_2;

    typedef K::Point_2                          Point;
    typedef K::Segment_2                        Segment;
    typedef Alpha_shape_2::Face                 Face;
    typedef Alpha_shape_2::Vertex               Vertex;
    typedef Alpha_shape_2::Edge                 Edge;
    typedef Alpha_shape_2::Face_handle          Face_handle;
    typedef Alpha_shape_2::Vertex_handle        Vertex_handle;

    typedef Alpha_shape_2::Face_circulator      Face_circulator;
    typedef Alpha_shape_2::Vertex_circulator    Vertex_circulator;
    typedef Alpha_shape_2::Locate_type          Locate_type;

    typedef Alpha_shape_2::Face_iterator        Face_iterator;
    typedef Alpha_shape_2::Vertex_iterator      Vertex_iterator;
    typedef Alpha_shape_2::Edge_iterator        Edge_iterator;
    typedef Alpha_shape_2::Edge_circulator      Edge_circulator;

    typedef Alpha_shape_2::Alpha_iterator       Alpha_iterator;
    typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;

    /**
     * Sort planes HORIZONTALLY in a clockwise order, around the center of mass.
     * @warning in place sort
     */
    static void clockwiseSort(std::vector<model::Plane3d>& planes);

    /**
     * Computes the HORIZONTAL convex hull of the given planes.
     */
    static std::vector<model::Plane3d> getConcaveHull(std::vector<model::Plane3d> planes);

    /**
     * Computes the discrete line between two points using Bresenham's algorithm.
     * @param src the source point
     * @param dest the destination point
     * @param precision the precision at which the discrete line shall be drawn
     */
    static void getDiscreteLine(model::Point3d src, model::Point3d dest, double precision, const double& extraDist,
            std::vector<model::Point3d>& line);
};

} /* namespace model */

#endif /* GRAPHICSALG_H_ */
