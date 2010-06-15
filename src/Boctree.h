/**
 * @file 
 * @brief Efficient representation of an octree
 * @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef BOCTREE_H
#define BOCTREE_H

#include <stdio.h>


#include <vector>
using std::vector;
#include <list>
using std::list;
#include <set>
using std::set;

union bitunion;

/**
 * This is our preferred representation for the leaf nodes (as it is the most compact). 
 * BOctTree makes an array of this, the first containing the number of points (not the 
 * number of coordinates) stored.
 */
union dunion {
  double v;
  int    length;
};

// TODO create a wrapper class to implement different point representations
//typedef list<double*> pointrep;
typedef dunion pointrep;

/**
 * This struct represents the nodes of the octree
 *
 * child_pointer is a relative pointer to the first child of this node, as it is only
 * 48 bit this will cause issues on systems with more than 268 TB of memory. All children
 * of this node must be stored sequentially. If one of the children is a leaf, that
 * child will be a pointer to however a set of points is represented (pointrep *).
 *
 * valid is a bitmask describing wether the corresponding buckets are filled.
 *
 * leaf is a bitmask describing wether the correpsonding bucket is a leaf node.
 *
 * The representation of the bitmask is somewhat inefficient. We use 16 bits for only 
 * 3^8 possible states, so in essence we could save 3 bits by compression.
 *
 */
class bitoct{
  public:
#ifdef _MSC_VER
  __int64 child_pointer        : 48;
  unsigned valid              :  8;
  unsigned leaf               :  8;
#else
  signed long child_pointer   : 48;
  unsigned valid              :  8;
  unsigned leaf               :  8;
#endif
  /**
   * sets the child pointer of parent so it points to child 
   */
  static inline void link(bitoct &parent, bitunion *child) {
    parent.child_pointer = (long)((char*)child - (char*)&parent);
  }

  /**
   * Returns the children of this node (given as parent).
   */
  static inline void getChildren(bitoct &parent, bitunion* &children) {
    children = (bitunion*)((char*)&parent + parent.child_pointer);
    // n = __builtin_popcount(parent.valid); // number of children
  }
};

/**
 * This union combines an octree node with a pointer to a set of points. This allows
 * us to use both nodes and leaves interchangeably.
 *
 * points is a pointer to the point representation in use
 *
 * node is simply the octree node
 *
 */
union bitunion {
  pointrep *points;
  bitoct node;

  bitunion(pointrep *p) : points(p) {};
  bitunion(bitoct b) : node(b) {};
  bitunion() : points(0) {};           // needed for new []
};

/**
 * @brief Octree
 * 
 * A cubic bounding box is calculated
 * from the given 3D points. Then it
 * is recusivly subdivided into smaller
 * subboxes
 */
class BOctTree {

public:
  
  BOctTree(double **pts, int n, double _voxelSize);
  virtual ~BOctTree();
  
  void GetOctTreeCenter(vector<double*>&c);
  void GetOctTreeRandom(vector<double*>&c);
  
  long countNodes();
 
protected:
  void GetOctTreeCenter(vector<double*>&c, bitoct &node, double *center, double size);
  void GetOctTreeRandom(vector<double*>&c, bitoct &node);
  long countNodes(bitoct &node);
  void deletetNodes(bitoct &node);

  pointrep *branch( bitoct &node, list<double*> &points, double center[3], double size);

  void countPointsAndQueue(list<double*> &i_points,
 		double center[8][3], double size, bitoct &parent); 

  void countPointsAndQueue(double **pts, int n,
    double center[8][3], double size, bitoct &parent);

  void childcenter(double *pcenter, double *ccenter, double size, int i);


  /**
   * the root of the octree 
   */
  bitoct* root;

  
  /**
   * storing the center
   */
  double center[3];
  
  /**
   * storing the dimension
   */
  double size;

  /**
   * storing the voxel size
   */
  static double voxelSize;

};

#include <stack>
using std::stack;


struct wrapper {
  void *node;      // current node
  unsigned int index;  // index of the child (or point) that comes after this node in the stack
  unsigned int length;  // number of childs 
/*  bool leaf;               // wether this is a leaf, and should be interpreted as a set of points
  
  wrapper(bitunion *_node) : node(_node), index(0), length( __builtin_popcount(_node->node.valid) ), leaf(false) {};
  wrapper(bitoct *_node) : node(_node), index(0), length( __builtin_popcount(_node->valid) ), leaf(false) {};
  wrapper(double *_node, unsigned int nrc) : node(_node), index(0), length(nrc), leaf(true) {};
  */
  wrapper(bitunion *_node) : node(_node), index(0), length( __builtin_popcount(_node->node.valid) )  {};
  wrapper(bitoct *_node) : node(_node), index(0), length( __builtin_popcount(_node->valid) ) {};
  wrapper(double *_node, unsigned int nrc) : node(_node), index(0), length(nrc) {};
};

class Iterater_BOctTree : public BOctTree {
  stack<wrapper*> rec_stack;
  double *current;
  bool valid;

  public:
    Iterater_BOctTree(double **pts, int n, double _voxelSize) : BOctTree(pts, n, _voxelSize), current(0), valid(false) {}

    void first() {
      // reset stack
//      rec_stack.clear();
      rec_stack.push( new wrapper(root) );
      printf("calling first: %p\n",root);
      current = first(*root);
      valid = true;
    }

    bool is_valid() { return valid;}

    double *get() {
      return current;
    }

    void next() {
      wrapper *c_w = rec_stack.top();  // current iterator position in the octree

      // top of stack MUST be a leaf node, i.e. pointing to set of points
      pointrep *points = ((pointrep *)c_w->node);
      c_w->index++;               // notify wrapper


      // check if index is too large now, this means the current leaf has run out of
      // points, we must backtrack to the previos node in the stack and get the next point
      if (c_w->index == c_w->length) {
        printf("N index > \n");
        delete c_w;
        rec_stack.pop();   // remove leaf
        // now backtrack till we find a node that has unexplored children, or until the 
        // stack has been popped completely
        while (!rec_stack.empty()) {
          c_w = rec_stack.top();
        printf("N prev element %p \n", c_w->node);
          if (c_w->index == c_w->length) { // this node is fully explored
        printf("N prev element %p has been fully explored delete\n", c_w->node);
            delete c_w;
            rec_stack.pop();   // remove leaf
          } else {
        printf("N prev element %p has childs not yet explored \n", c_w->node);
            break;
          }
        }
        if (rec_stack.empty()) {
          valid = false;
          return;
        }

        // c_w has unexplored children, get the first point of the first unexplored child
        bitoct &node = *(bitoct*)c_w->node;
        bitunion *children;
        bitoct::getChildren(node, children);
        unsigned int count = 0;
        
        for (short i = 0; i < 8; i++) {
          if (  ( 1 << i ) & node.valid ) {   // if ith node exists
            if (count <= c_w->index) {   // not yet at the first unexplored child  // TODO FIXME can be improved by comparing to the deleted node above
              printf("N child %p is not next child\n", children);
              ++children; // next child
              ++count;
              continue;
            }
              c_w->index++;               // notify wrapper
              printf("N child %p _is_ next child\n", children);


            // otherwise we must check if child is leaf or not and proceed with getting the first point
            if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
              pointrep *points = children->points;
              unsigned int length = points[0].length;
              rec_stack.push( new wrapper(&(points[1].v), length) );
              current = &(points[1].v);
            } else { // recurse
              rec_stack.push( new wrapper(children) );  // push this node on stack
              current = first(children->node);
            }
            break;
          }
        }
      } else {
        current = &(points[c_w->index*3 ].v);  // set current point to the next point in the list

      }


    }


  private:

    /**
     * Finds the first point that is a child of the given node.
     * Assumes rec_stack is filled up to node. Will continue filling rec_stack with 
     * children of the node in order.
     *
     */
    double *first(bitoct &node) {
      printf("first %p\n", &node);
      bitunion *children;
      bitoct::getChildren(node, children);

      for (short i = 0; i < 8; i++) {
        if (  ( 1 << i ) & node.valid ) {   // if ith node exists
          if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
            pointrep *points = children->points;
            unsigned int length = points[0].length;
            rec_stack.push( new wrapper(&(points[1].v), length) );
            return &(points[1].v);
          } else { // recurse
            rec_stack.push( new wrapper(children) );  // push this node on stack
            return first(children->node);
          }
          ++children; // next child
        }
      }
    }




};


#endif
