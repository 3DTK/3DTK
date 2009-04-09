#ifndef __ELCH6D_H__
#define __ELCH6D_H__

#include "loopSlam6D.h"

class elch6D : public loopSlam6D {

  public:
    elch6D(bool _quiet, icp6Dminimizer *my_icp6Dminimizer, double mdm, int max_num_iterations, int rnd, bool eP, int anim, double epsilonICP, bool use_cache)
     : loopSlam6D(_quiet, my_icp6Dminimizer, mdm, max_num_iterations, rnd, eP, anim, epsilonICP, use_cache) {}

    static void graph_balancer(graph_t &g, int f, int l, double *weights);

    static void graph_weight_out(graph_t &g, int first, int last, double *weights);
    static void graph_weight_out(graph_t &g, int first, int last, double *weights, string &out_file);
    static void graph_pos_out(graph_t &g, const vector <Scan *> &allScans);
    static void graph_pos_out(graph_t &g, const vector <Scan *> &allScans, string &out_file);
    static void dot_pos_out(graph_t &g, const vector <Scan *> &allScans, string &out_file);
    static void graph_out(graph_t &g);
    static void graph_out(graph_t &g, string &out_file);
    static void slim_graph_out(graph_t g);
    static void slim_graph_out(graph_t g, string &out_file);
};

#endif
