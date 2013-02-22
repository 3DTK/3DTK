/**
 * Point Cloud Segmentation using Felzenszwalb-Huttenlocher Algorithm
 *
 * Copyright (C) Jacobs University Bremen
 *
 * Released under the GPL version 3.
 *
 * @author Mihai-Cotizo Sima
 * @author Andreas Nuechter
 */


#ifndef __FHGRAPH_H__
#define __FHGRAPH_H__

#include <vector>
#include <list>

#include <slam6d/point.h>
#include <slam6d/scan.h>
#include <segmentation/segment-graph.h>
#include <ANN/ANN.h>

class FHGraph {
public:
  FHGraph(std::vector< Point > *ps,
		double weight(Point, Point),
		double sigma,
		double eps,
		int neighbors,
		float radius);
  edge* getGraph();
  Point operator[](int index);
  int getNumPoints();
  int getNumEdges();

  void dispose();

private:
  void compute_neighbors(double weight(Point, Point), double eps);
  void do_gauss(double sigma);
  void without_gauss();

  std::vector<edge> edges;
  std::vector< Point > *pts;
  int V;
  int E;

  int nr_neighbors;
  float radius;

  struct he{ int x; float w; };
  std::vector< std::list<he> > adjency_list;
};

#endif
