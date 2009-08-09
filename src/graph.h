/** @file 
 *  @brief The 6D Lu Milios style SLAM in 6D
 *
 *  @author Dorit Borrman. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <vector>
using std::vector;
#include <string>
using std::string;
#include <fstream>
using std::ostream;

#include <boost/graph/adjacency_list.hpp>

/* Boost graph declaration */
using boost::edge_weight_t;
typedef boost::adjacency_list <
    boost::listS, boost::vecS, boost::undirectedS,
    boost::no_property,
    boost::property < edge_weight_t, double > >
  graph_t;

/**
 * @brief This class represent a directed network. 
 *        Each node corresponds to a laser scan.
 */
class Graph {

public: 

  Graph();
  Graph(const string &netfile);
  Graph(int nrScans, bool loop);
  Graph(double cldist, int loopsize);
  Graph(int nodes, double cldist2, int loopsize);
  
  int getLink(int i, int fromTo);
  void addLink(int i, int j);
  
  void setNrScans(int _nrScans);
  int getNrScans();
  int getNrLinks();
  
  int getStart();
  int getEnd();

  friend ostream& operator<<(ostream& os, Graph* gr);
  
private:
  /**
   * The basic network structure 
   */
  vector <int> from, to;

  /**
   * The number of scans contained in this Graph
   */
  int nrScans;

  /**
   * The very first Scan
   */
  int start;
};

#endif
