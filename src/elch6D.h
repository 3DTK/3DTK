/** 
 * @file graph balancer implementation and utility functions
 * @author Jochen Sprickerhof
 */

#ifndef __ELCH6D_H__
#define __ELCH6D_H__

#include "loopSlam6D.h"

class elch6D : public loopSlam6D {

  public:
    elch6D(bool _quiet, icp6Dminimizer *my_icp6Dminimizer, double mdm, int max_num_iterations, int rnd, bool eP, int anim, double epsilonICP, bool use_cache)
      : loopSlam6D(_quiet, my_icp6Dminimizer, mdm, max_num_iterations, rnd, eP, anim, epsilonICP, use_cache) {}

    /**
     * graph balancer algorithm computes the weights
     * @param g the graph
     * @param f index of the first node
     * @param l index of the last node
     * @param weights array for the weights
     */
    static void graph_balancer(graph_t &g, int f, int l, double *weights);

    /**
     * sets a filename for graph_weight_out and calls it
     * @param g the graph to save
     * @param first the first node
     * @param last the last node
     * @param weights the computed weights
     */
    static void graph_weight_out(graph_t &g, int first, int last, double *weights);
    /**
     * writes a graphviz file with the graph labled with the computed weights
     * @param g the graph to save
     * @param first the first node
     * @param last the last node
     * @param weights the computed weights
     * @param out_file the file to write to
     */
    static void graph_weight_out(graph_t &g, int first, int last, double *weights, string &out_file);
    /**
     * sets a filename and calls graph_pos_out
     * @param g the graph to save
     * @param allScans all laser scans
     */
    static void graph_pos_out(graph_t &g, const vector <Scan *> &allScans);
    /**
     * writes the graph using the computed scan poses
     * @param g the graph to save
     * @param allScans all laser scans
     * @param out_file the file to write to
     */
    static void graph_pos_out(graph_t &g, const vector <Scan *> &allScans, string &out_file);
    /**
     * write graphviz file with real poses
     * @param g the graph
     * @param allScans all laser scans
     * @param out_file the file to write to
     */
    static void dot_pos_out(graph_t &g, const vector <Scan *> &allScans, string &out_file);
    /**
     * sets filename and calls graph_out
     * @param g the graph
     */
    static void graph_out(graph_t &g);

    /**
     * uses boost write_graphviz to write the graph
     * @param g the graph
     * @param out_file the file to write to
     */
    static void graph_out(graph_t &g, string &out_file);
    /**
     * sets filename and calls slim_graph_out
     */
    static void slim_graph_out(graph_t g);
    /**
     * writes slim graph (supressing unimportant nodes)
     * @param g the graph
     * @param out_file the file to write to
     */
    static void slim_graph_out(graph_t g, string &out_file);
};

#endif
