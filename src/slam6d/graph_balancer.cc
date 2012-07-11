/*
 * graph_balancer implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file stand alone graph balancer implementation
 * reads graph in graphviz format and optimizes it
 * @author Jochen Sprickerhof
 */

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <string>
using std::string;

#include <boost/graph/graphviz.hpp>

#include "slam6d/elch6D.h"

void usage(string progname) {
  cout << "Usage: " << progname << " OPTIONS FILE" << endl <<
    "\t -s \t\t first node in loop" << endl <<
    "\t -e \t\t last node in loop" << endl <<
    "\t -o \t\t output filename (must be given)" << endl;
}

int main(int argc, char *argv[])
{

  if(argc < 2) {
    usage(argv[0]);
    exit(1);
  }

  int start = 0, end = 4;
  string out_file;
  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "end",       required_argument,   0,  'e' },
    { "out",       required_argument,   0,  'o' },
    { "start",     required_argument,   0,  's' },
    //{ "reduce",    required_argument,   0,  'r' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  int c;
  while((c = getopt_long(argc, argv, "e:o:s:", longopts, NULL)) != -1) {
    switch (c) {
      case 'e':
        end = atoi(optarg);
        if(end < 0) {
          cerr << "Error: Cannot end at a negative number." << endl;
          exit(1);
        }
        break;
      case 'o':
        out_file = optarg;
        break;
      case 's':
        start = atoi(optarg);
        if(start < 0) {
          cerr << "Error: Cannot start at a negative number." << endl;
          exit(1);
        }
        break;
      case '?':
      default:
        exit(1);
    }
  }

  if(out_file.empty()) {
    usage(argv[0]);
    exit(1);
  }

  graph_t g(0);
  boost::dynamic_properties dp(boost::ignore_other_properties);
  boost::property_map<graph_t, boost::edge_weight_t>::type weight = get(boost::edge_weight, g);
  dp.property("weight", weight);

  std::ifstream gf(argv[optind]);
  read_graphviz(gf, g, dp);

  graph_t g_org(g);
  double *w = new double[num_vertices(g)];

  elch6D::graph_balancer(g, start, end, w);
  elch6D::graph_weight_out(g_org, start, end, w, out_file);

  delete[] w;
  exit(0);
}

