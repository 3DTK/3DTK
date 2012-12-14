/*
 * graph implementation
 *
 * Copyright (C) Dorit Borrmann, Jan Elseberg, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @brief The implementation of a graph
 * @author Dorit Borrman. Inst. of CS, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Inst. of CS, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 */

#include "slam6d/graph.h"

#include "slam6d/scan.h"
#include "slam6d/globals.icc"

#include <fstream>
using std::ifstream;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

/**
 * Constructor to create an empty graph
 */
Graph::Graph()
{
  nrScans = 0;
  start = 0;
}


/**
 * Constructor to read a Graph from a file.
 * The file must have the the number of scans in the first line,
 * the number of links to be read in the second and the links in
 * the links in the third and fourth line
 *
 * Example:
 * 4
 * 3
 * 0 1
 * 1 2
 * 3 4
 * 4 0
 *
 * @param netfile Filename of the file to read
 */
Graph::Graph(const string &netfile)
{
  nrScans = 0;
  start = 0;
  int local_nrScans, local_nrLinks;
  cout << "Reading network " << netfile;
  
  ifstream file(netfile.c_str());
  file >> local_nrScans
       >> local_nrLinks;

  int f, t;
  for(int j = 0; j < local_nrLinks; j++) {
    if (!file.good()) {
      cerr << "Error while reading network structure" << endl;
      exit(1);
    }
    file >> f >> t;
    cout << f << " " << t << endl;
    addLink(f, t);
  } 
  cout << " structure done." << endl;
}

/** 
 * Constructor builds a minimally connected Graph with a given number of scans.
 * The loop can optionally be closed,
 * so that the first will be connected to the last scan.
 * 
 * @param nScans The number of Scans
 * @param loop Specifies whether the first and last scan should be linked
 */
Graph::Graph(int nScans, bool loop)
{
  int nrLinks = 0;
  start = 0;
  nrScans = nScans;
  
  if (loop) {
    nrLinks = nScans;
  } else {    
    nrLinks = nScans - 1;
  }
  
  for(int i = 0 ;i < nrLinks; i++){
    from.push_back(i);
    if (loop) {
      to.push_back(i != nrLinks - 1 ?  i + 1 : 0) ;
    } else {
      to.push_back(i + 1);
    }
  }
}


Graph::Graph(int nodes, double cldist2, int loopsize)
{
  // nodes + 1
  start = 0;
  nrScans = nodes;
  int nrLinks = nodes - 1;
  
  for(int i = 0; i < nrLinks; i++){
    from.push_back(i);
    to.push_back(i + 1);
  }

  // nodes 
  for (int j = 0; j < nodes; j++) {
    for (int k = j + 1; k < nodes; k++) {
      if ((abs(k-j) > loopsize) &&
          (Dist2(Scan::allScans[j]->get_rPos(),
                 Scan::allScans[k]->get_rPos()) < cldist2)) {
        addLink(j, k);
      }
    }
  }
}



/** 
 * Returns the specified link
 * 
 * @param i The i-th link
 * @param fromTo 0 is the outgoing node and 1 the ingoing
 * @return An integer for the node
 */
int Graph::getLink(int i, int fromTo)
{
  if (i >= 0 && i < (int)from.size()) {
    if (fromTo == 0) return from[i];
     else return to[i];
  }
  return 0;
}


/** 
 * adds a link to a graph
 * 
 * @param i from node
 * @param j to node
 */
void Graph::addLink(int i, int j)
{
  int present = 0;
  for (unsigned int iterator = 0; iterator < from.size(); iterator++) {
    if (from[iterator] == i) present++;
    if (to[iterator] == i) present++;
  }
  if (present == 0) nrScans++;
  present = 0;
  for (unsigned int iterator = 0; iterator < from.size(); iterator++) {
    if (from[iterator] == j) present++;
    if (to[iterator] == j) present++;
  }
  if (present == 0) nrScans++;
  
  from.push_back(i);
  to.push_back(j);
}


/**
 * Returns the number of links
 * @return number of links
 */
int Graph::getNrLinks()
{
  return from.size();
}

/**
 * Returns the number of scans
 * @return number of scans
 */
int Graph::getNrScans()
{
  return nrScans;
}

/**
 * sets the number of scans
 * @param number of scans
 */
void Graph::setNrScans(int _nrScans)
{
  nrScans = _nrScans;
}

/**
 * Returns the number of the first scan
 * @return number of the first scan
 */
int Graph::getStart()
{
  return start;
}

/**
 * Returns the number of the last scan
 * @return number of the last scan
 */
int Graph::getEnd()
{
  return start+nrScans - 1;
}

/**
 * Prints out the Graph nicely formatted 
 * @param os the stream to print to
 * @param gr which Graph to print
 * @return the resulting output stream
 */
ostream& operator<<(ostream& os, Graph* gr)
{
  for(int i = 0; i < (int)gr->from.size() ; i++){
    os << "( " << gr->getLink(i,0) << " - " << gr->getLink(i,1) << " )" << endl;
  }
  return os;
}
