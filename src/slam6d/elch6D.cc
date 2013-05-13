/*
 * elch6D implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */

/** @file graph balancer implementation and utility functions
 *  @author Jochen Sprickerhof. Inst. of CS, University of Osnabrueck, Germany.
 *
 * This file implements the paper (ecmr2009.pdf):
 * Jochen Sprickerhof, Andreas Nuechter, Kai Lingemann, Joachim Hertzberg.
 * An Explicit Loop Closing Technique for 6D SLAM,
 * In Proceedings of the 4th European Conference on Mobile Robots (ECMR '09),
 * Mlini/Dubrovnic, Croatia, September 2009.
 */

#include <fstream>
using std::ofstream;

#include <string>
using std::string;

#include <list>
using std::list;

#include <algorithm>
using std::swap;

#include <limits> //for old boost and new gcc
using std::numeric_limits;
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;
#include <boost/property_map/property_map.hpp>

#include "slam6d/globals.icc"
#include "slam6d/elch6D.h"

/*
void printout(graph_t &g, vector<Vertex> &p, vector<int> &d, double *weights)
{
  cout << "distances and parents:" << endl;
  graph_traits <graph_t>::vertex_iterator vi, vend;
  for(boost::tuples::tie(vi, vend) = vertices(g); vi != vend; vi++) {
    cout << "distance(" << *vi << ") = " << d[*vi] <<
      ", parent(" << *vi << ") = " << p[*vi] <<
      *vi << " " << weights[*vi] << endl;
  }
}
*/

/**
 * sets a filename for graph_weight_out and calls it
 * @param g the graph to save
 * @param first the first node
 * @param last the last node
 * @param weights the computed weights
 */
void elch6D::graph_weight_out(graph_t &g, int first, int last, double *weights)
{
  string name("graph_weight_" + to_string(num_vertices(g), 3) + ".dot");
  graph_weight_out(g, first, last, weights, name);
}

/**
 * writes a graphviz file with the graph labled with the computed weights
 * @param g the graph to save
 * @param first the first node
 * @param last the last node
 * @param weights the computed weights
 * @param out_file the file to write to
 */
void elch6D::graph_weight_out(graph_t &g, int first, int last,
                              double *weights, string &out_file)
{
  ofstream dot_file(out_file.c_str());
  dot_file << "graph D {" << endl;
  graph_traits <graph_t>::vertex_iterator vi, vend;
  for(boost::tuples::tie(vi, vend) = vertices(g); vi != vend; vi++) {
    dot_file << *vi << "[label=\"" << *vi << " (" <<
      weights[*vi] << ")\"];" << endl;
  }
  boost::property_map<graph_t, edge_weight_t>::type weightmap
    = get(boost::edge_weight, g);
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(boost::tuples::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    dot_file << source(*ei, g) << " -- " << target(*ei, g) <<
      "[label=\"" << get(weightmap, *ei) << "\"];" << endl;
  }
  dot_file << first << " -- " << last << "[color=\"green\"] }";
  dot_file.close();
}

/**
 * sets a filename and calls graph_pos_out
 * @param g the graph to save
 * @param allScans all laser scans
 */
void elch6D::graph_pos_out(graph_t &g, const vector <Scan *> &allScans)
{
  string name("graph_pos_" + to_string(num_vertices(g), 3) + ".dot");
  graph_pos_out(g, allScans, name);
}

/**
 * writes the graph using the computed scan poses
 * @param g the graph to save
 * @param allScans all laser scans
 * @param out_file the file to write to
 */
void elch6D::graph_pos_out(graph_t &g, const vector <Scan *> &allScans,
                           string &out_file)
{
  ofstream graph_file(out_file.c_str());
  ofstream graph2_file(("2" + out_file).c_str());
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(boost::tuples::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    if(source(*ei, g) + 1 != target(*ei, g)) {
      graph2_file << allScans[source(*ei, g)]->get_rPos()[0]
                  << " " << allScans[source(*ei, g)]->get_rPos()[1]
                  << " " << allScans[source(*ei, g)]->get_rPos()[2]
                  << endl;
      graph2_file << allScans[target(*ei, g)]->get_rPos()[0]
                  << " " << allScans[target(*ei, g)]->get_rPos()[1]
                  << " " << allScans[target(*ei, g)]->get_rPos()[2]
                  << endl << endl;
    } else {
      graph_file << allScans[source(*ei, g)]->get_rPos()[0]
                 << " " << allScans[source(*ei, g)]->get_rPos()[1]
                 << " " << allScans[source(*ei, g)]->get_rPos()[2]
                 << endl;
      graph_file << allScans[target(*ei, g)]->get_rPos()[0] << " "
                 << allScans[target(*ei, g)]->get_rPos()[1] << " "
                 << allScans[target(*ei, g)]->get_rPos()[2]
                 << endl << endl;
    }
  }
  graph_file.close();
  graph2_file.close();
}

/**
 * write graphviz file with real poses
 * @param g the graph
 * @param allScans all laser scans
 * @param out_file the file to write to
 */
void elch6D::dot_pos_out(graph_t &g, const vector <Scan *> &allScans,
                         string &out_file)
{
  ofstream dot_file(out_file.c_str());

  dot_file << "graph D {" << endl << "size=\"20,20\"" << endl;

  int n = num_vertices(g);
  for(int i = 0; i < n; i++) {
    dot_file << i << "[pos=\""
             << allScans[i]->get_rPos()[0]
             << "," << allScans[i]->get_rPos()[2]
             << "\", label=\"\", height=0.1, width=0.1, fixedsize=true];"
             << endl;
  }
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(boost::tuples::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    if(source(*ei, g) + 1 != target(*ei, g)) {
      dot_file << source(*ei, g) << " -- " << target(*ei, g)
               << "[color=\"green\"];" << endl;
    } else {
      dot_file << source(*ei, g) << " -- " << target(*ei, g) << ";" << endl;
    }
  }
  dot_file << "}";
  dot_file.close();
}

/**
 * graph balancer algorithm computes the weights
 * @param g the graph
 * @param f index of the first node
 * @param l index of the last node
 * @param weights array for the weights
 */
void elch6D::graph_balancer(graph_t &g, int f, int l, double *weights)
{
  list<int> crossings, branches;
  crossings.push_back(f);
  crossings.push_back(l);
  weights[f] = 0;
  weights[l] = 1;

  int *p = new int[num_vertices(g)];
  int *p_min = new int[num_vertices(g)];
  double *d = new double[num_vertices(g)];
  double *d_min = new double[num_vertices(g)];
  double dist;
  bool do_swap = false;
  list<int>::iterator si, ei, s_min, e_min;

  // process all junctions
  while(!crossings.empty()) {
    dist = -1;
    // find shortest crossing for all vertices on the loop
    for(si = crossings.begin(); si != crossings.end(); ) {
      dijkstra_shortest_paths(g,
                              *si,
                              predecessor_map(boost::make_iterator_property_map(p, get(boost::vertex_index, g))).
                              distance_map(boost::make_iterator_property_map(d, get(boost::vertex_index, g))));
      ei = si;
      ei++;
      // find shortest crossing for one vertex
      for(; ei != crossings.end(); ei++) {
        if(*ei != p[*ei] && (dist < 0 || d[*ei] < dist)) {
          dist = d[*ei];
          s_min = si;
          e_min = ei;
          do_swap = true;
        }
      }
      if(do_swap) {
        swap(p, p_min);
        swap(d, d_min);
        do_swap = false;
      }
      // vertex starts a branch
      if(dist < 0) {
        branches.push_back(*si);
        si = crossings.erase(si);
      } else {
        si++;
      }
    }

    if(dist > -1) {
      remove_edge(*e_min, p_min[*e_min], g);
      for(int i = p_min[*e_min]; i != *s_min; i = p_min[i]) {
        // even right with weights[*s_min] > weights[*e_min]! (math works)
        weights[i] = weights[*s_min] + (weights[*e_min] - weights[*s_min])
          * d_min[i] / d_min[*e_min];
        remove_edge(i, p_min[i], g);
        if(degree(i, g) > 0) {
          crossings.push_back(i);
        }
      }

      if(degree(*s_min, g) == 0) {
        crossings.erase(s_min);
      }

      if(degree(*e_min, g) == 0) {
        crossings.erase(e_min);
      }
    }
  }

  delete[] p;
  delete[] p_min;
  delete[] d;
  delete[] d_min;

  graph_traits <graph_t>::adjacency_iterator ai, ai_end;
  int s;

  // error propagation
  while(!branches.empty()) {
    s = branches.front();
    branches.pop_front();

    for(boost::tuples::tie(ai, ai_end) = adjacent_vertices(s, g); ai != ai_end; ++ai) {
      weights[*ai] = weights[s];
      if(degree(*ai, g) > 1) {
        branches.push_back(*ai);
      }
    }
    clear_vertex(s, g);
  }
}
