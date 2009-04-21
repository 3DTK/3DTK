#include <fstream>
using std::ofstream;

#include <string>
using std::string;

#include <limits.h>

#include <list>
using namespace std;

#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;

#include "elch6D.h"

/*void printout(graph_t &g, vector<Vertex> &p, vector<int> &d, double *weights)
{
  cout << "distances and parents:" << endl;
  graph_traits <graph_t>::vertex_iterator vi, vend;
  for(tie(vi, vend) = vertices(g); vi != vend; vi++) {
    cout << "distance(" << *vi << ") = " << d[*vi] <<
      ", parent(" << *vi << ") = " << p[*vi] <<
      *vi << " " << weights[*vi] << endl;
  }
}*/

void elch6D::graph_weight_out(graph_t &g, int first, int last, double *weights)
{
  string name("graph_weight_" + to_string(num_vertices(g), 3) + ".dot");
  graph_weight_out(g, first, last, weights, name);
}

void elch6D::graph_weight_out(graph_t &g, int first, int last, double *weights, string &out_file)
{
  ofstream dot_file(out_file.c_str());
  dot_file << "graph D {" << endl;
  graph_traits <graph_t>::vertex_iterator vi, vend;
  for(tie(vi, vend) = vertices(g); vi != vend; vi++) {
    dot_file << *vi << "[label=\"" << *vi << " (" <<
      weights[*vi] << ")\"];" << endl;
  }
  boost::property_map<graph_t, edge_weight_t>::type weightmap = get(boost::edge_weight, g);
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    dot_file << source(*ei, g) << " -- " << target(*ei, g) <<
      "[label=\"" << get(weightmap, *ei) << "\"];" << endl;
  }
  dot_file << first << " -- " << last << "[color=\"green\"] }";
  dot_file.close();
}

void elch6D::graph_pos_out(graph_t &g, const vector <Scan *> &allScans)
{
  string name("graph_pos_" + to_string(num_vertices(g), 3) + ".dot");
  graph_pos_out(g, allScans, name);
}

void elch6D::graph_pos_out(graph_t &g, const vector <Scan *> &allScans, string &out_file)
{
  ofstream graph_file(out_file.c_str());
  ofstream graph2_file(("2" + out_file).c_str());
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    if(source(*ei, g) + 1 != target(*ei, g)) {
      graph2_file << allScans[source(*ei, g)]->get_rPos()[0] << " " << allScans[source(*ei, g)]->get_rPos()[1] << " " << allScans[source(*ei, g)]->get_rPos()[2] << endl;
      graph2_file << allScans[target(*ei, g)]->get_rPos()[0] << " " << allScans[target(*ei, g)]->get_rPos()[1] << " " << allScans[target(*ei, g)]->get_rPos()[2] << endl << endl;
    } else {
      graph_file << allScans[source(*ei, g)]->get_rPos()[0] << " " << allScans[source(*ei, g)]->get_rPos()[1] << " " << allScans[source(*ei, g)]->get_rPos()[2] << endl;
      graph_file << allScans[target(*ei, g)]->get_rPos()[0] << " " << allScans[target(*ei, g)]->get_rPos()[1] << " " << allScans[target(*ei, g)]->get_rPos()[2] << endl << endl;
    }
  }
  graph_file.close();
  graph2_file.close();
}

void elch6D::dot_pos_out(graph_t &g, const vector <Scan *> &allScans, string &out_file)
{
  ofstream dot_file(out_file.c_str());

  dot_file << "graph D {" << endl << "size=\"20,20\"" << endl;

  int n = num_vertices(g);
  for(int i = 0; i < n; i++) {
    dot_file << i << "[pos=\"" <<
      allScans[i]->get_rPos()[0] << "," <<
      allScans[i]->get_rPos()[2] << "\", label=\"\", height=0.1, width=0.1, fixedsize=true];" << endl;
  }
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    if(source(*ei, g) + 1 != target(*ei, g)) {
      dot_file << source(*ei, g) << " -- " << target(*ei, g) << "[color=\"green\"];" << endl;
    } else {
      dot_file << source(*ei, g) << " -- " << target(*ei, g) << ";" << endl;
    }
  }
  dot_file << "}";
  dot_file.close();
}

void elch6D::graph_out(graph_t &g)
{
  string name("graph_" + to_string(num_vertices(g), 3) + ".dot");
  graph_out(g, name);
}

void elch6D::graph_out(graph_t &g, string &out_file)
{
  ofstream dot_file(out_file.c_str());
  write_graphviz(dot_file, g);
  dot_file.close();
}

void elch6D::slim_graph_out(graph_t g)
{
  string name("slim_graph_" + to_string(num_vertices(g), 3) + ".dot");
  slim_graph_out(g, name);
}

void elch6D::slim_graph_out(graph_t g, string &out_file)
{
  bool todo;
  graph_traits < graph_t >::vertex_iterator vi, vend;
  graph_traits < graph_t >::adjacency_iterator ai;
  do {
    todo = false;
    for(tie(vi, vend) = vertices(g); vi != vend; vi++) {
      int me = *vi;
      if(degree(me, g) == 2) {
        ai = adjacent_vertices(me, g).first;
        int one = *ai;
        ai++;
        int two = *ai;
        if(degree(one, g) == 2 && degree(two, g) == 2 && one != me && two != me && one != two) {
          clear_vertex(me, g);
          add_edge(one, two, g);
          remove_vertex(me, g);
          todo = true;
          break;
        }
      }
    }
  } while(todo);

  ofstream dot_file(out_file.c_str());
  write_graphviz(dot_file, g);
  dot_file.close();
}

void elch6D::graph_balancer(graph_t &g, int f, int l, double *weights)
{
  list<int> crossings, branches;
  crossings.push_back(f);
  crossings.push_back(l);
  weights[f] = 0;
  weights[l] = 1;

  vector<int> p(num_vertices(g)), p_min;
  vector<double> d(num_vertices(g)), d_min;
  double dist;
  list<int>::iterator si, ei, s_min, e_min;

  while(!crossings.empty()) {
    dist = -1;
    for(si = crossings.begin(); si != crossings.end(); si++) {
      dijkstra_shortest_paths(g, *si, boost::predecessor_map(&p[0]).distance_map(&d[0]));
      ei = si;
      ei++;
      for(; ei != crossings.end(); ei++) {
        if(*ei != p[*ei] && (dist < 0 || d[*ei] < dist)) {
          dist = d[*ei];
          p_min = p;
          d_min = d;
          s_min = si;
          e_min = ei;
        }
      }
      if(dist < 0) {
        branches.push_back(*si);
        si = crossings.erase(si);
        si--;
      }
    }

    if(dist > -1) {
      remove_edge(*e_min, p_min[*e_min], g);
      for(int i = p_min[*e_min]; i != *s_min; i = p_min[i]) {
        //Das geht sogar, wenn weights[*s_min] > weights[*e_min]!
        weights[i] = weights[*s_min] + (weights[*e_min] - weights[*s_min]) * d_min[i] / d_min[*e_min];
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

  graph_traits <graph_t>::adjacency_iterator ai, ai_end;
  int s;

  while(!branches.empty()) {
    s = branches.front();
    branches.pop_front();

    for(tie(ai, ai_end) = adjacent_vertices(s, g); ai != ai_end; ++ai) {
      weights[*ai] = weights[s];
      if(degree(*ai, g) > 1) {
        branches.push_back(*ai);
      }
    }
    clear_vertex(s, g);
  }
}
