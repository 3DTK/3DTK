#ifndef __PATHGRAPH_H__
#define __PATHGRAPH_H__
#include <map>
using std::map;
#include <list>
using std::list;
#include <climits>
#include <cstddef>


typedef struct {float x; float y; } PointXY;
typedef struct {float x; float y; float z; } PointXYZ;


/** @class PGNode 
  * A 2D-node in a PathGraph.
 */
struct PGNode{
  PGNode():smallestCost(LONG_MAX){}; 
  PointXY       point;       ///< 2D coordinates of node
  list<PGNode*> neighbours;  ///< List of neighbours of node
  double smallestCost;       ///< Smallest cost from start (used in AStar)
};

/**
  * @class PGEdge
  * Represents an edge in a PathGraph, using pointers to PGNode.
  */
struct PGEdge {
  PGEdge():isAlive(true){}
  PGNode* start;              ///< Start of edge
  PGNode* end;                ///< End of edge
  bool isAlive;               ///< useful for checking which edges have been checked already
  bool operator<(const PGEdge& e)const{
    PGNode*  min  = (start<end)?start:end;
    PGNode* emin  = (e.start<e.end)?e.start:e.end;
    if(min < emin) return true;
    if(min > emin) return false;

    PGNode*  max  = (start>end)?start:end;
    PGNode* emax  = (e.start>e.end)?e.start:e.end;
    return max < emax;
  }
};

/** @class PathGraph
  * Represents a graph as a list of nodes which know their neigbours.
  * @author Anton Fl&uuml;gge
  * @author Hannes Schulz
  */
class PathGraph {
  friend class PathGraphCreator;
  friend class VisibilityPathGraphCreator;

  public:
		/** @class Iter
			* pseudo-iterator useful to iterate over all edges in graph
			* Edges with points start and end equal are ignored.
			*/
		class Iter{
			private:
				map<PGEdge,bool> visitedEdges;         ///< keeps track of visited edges
				list<PGNode*>::iterator aktNode;       ///< it. to node in nodelist
				list<PGNode*>::iterator aktNeighbour;  ///< it. to node in aktNode's neighbour list
				list<PGNode*>::iterator lastNode;      ///< last node in nodelist
				list<PGNode*>::iterator lastNeighbour; ///< last node in aktNode's neighbour list
			public:
				/**
				 * @param f iterator of first in nodelist
				 * @param l iterator of last  in nodelist
				 */
				Iter(list<PGNode*>::iterator f,list<PGNode*>::iterator l)
					:aktNode(f),lastNode(l){
						if(f==l)
							return;
						aktNeighbour  = (*f)->neighbours.begin();
						lastNeighbour = (*f)->neighbours.end();
						if(aktNeighbour == lastNeighbour)
							(*this)++;
						visitedEdges[**this] = true;
						/*
						PGEdge pge = (**this);
						if(pge.start->point < pge.end->point)
							(*this)++;
						if(pge.start->point == pge.end->point)
							(*this)++;
						*/
					}
				/**
				 * delete edge
				 */
				inline void remove(PathGraph* pg){
					PGNode *anode = (*aktNode), *anei = (*aktNeighbour);
					(*this)++;
					pg->deleteEdge(anode,anei);
				}
				/**
				 * advance iterator
				 */
				inline void operator++(int){
					bool visited = true;
					while(visited){
						aktNeighbour++;
						if(aktNeighbour == lastNeighbour){
							aktNode++;
							if(aktNode != lastNode){
								aktNeighbour  = (*aktNode)->neighbours.begin();
								lastNeighbour = (*aktNode)->neighbours.end();
								if(lastNeighbour == aktNeighbour) // node does not have neighbours
									continue;
								/*
								PGEdge pge = (**this);
								if(pge.start->point < pge.end->point)
									continue;
								if(pge.start->point == pge.end->point)
									continue;
									*/
								if(visitedEdges.find(**this) != visitedEdges.end())   // edge visited
									continue;
								visitedEdges[**this] = true;
								visited = false;
							}else{
								visited = false;
							}
						}else{
							PGEdge pge = (**this);
							/*
							if(pge.start->point < pge.end->point)
								continue;
							if(pge.start->point == pge.end->point)
								continue;
								*/
							if(visitedEdges.find(pge) != visitedEdges.end())   // edge visited
								continue;
							visitedEdges[pge] = true;
							visited = false;
						}
					}
				}
				/// compare two iterators
				inline bool operator==(const Iter& i){
					if(aktNode == lastNode)
						if(i.aktNode == lastNode)
							return true;
					return (aktNode == i.aktNode) 
						&& (aktNeighbour == i.aktNeighbour);
				}
				/// @return true if no more edges in graph
				inline bool isLast(){ return aktNode == lastNode; }
				/// @return current edge
				inline PGEdge operator*(){
					PGEdge e;
					e.start = (*aktNode);
					e.end   = (*aktNeighbour);
					return e;
				}
		};
    PathGraph():ivpStartNode(NULL),ivpEndNode(NULL),startOnGraph(true),endOnGraph(true){}
    /// @return whether node is end node
    inline bool    isEndNode(PGNode* n){return n==ivpEndNode;}
    /** Inserts a node in the graph.
      * @param p coordinates of node
      * @return pointer to new PGNode (NOT PointXY!)
      */
    PGNode* insertNode(PointXY p);
    /** Inserts a node in the graph and connects it to all nodes already in the graph.
      * @param p coordinates of node
      */
    void insertNodeAndConnectToAll(PointXY p);
     /** Inserts a node in the graph, if not already exists.
      * @param p coordinates of node
      * @return pointer to existing PGNode if exists, else to new PGNode
      */
    PGNode* insertUniqueNode(PointXY p);
    /// inserts edge between two nodes (BOTH ways)
    void    insertEdge(PGNode*, PGNode*);
    /// delete a node and all its edges from the graph
    void    deleteNode(PGNode*);
    /// delete edge between two nodes (BOTH ways)
    void    deleteEdge(PGNode*,PGNode*);
    /** set the start node and insert it if necessary
     *  deletes old start node, if it was
     *  insert by an previous call of setStartNode
     * @return false iff a node at this point already exists
     */
    bool setStartNode(PointXY p);
    /** set the end node and insert it if necessary
     *  deletes old end node, if it was
     *  insert by an previous call of setEndNode
     * @return false iff a node at this point already exists
     */
    bool setEndNode(PointXY p);
    /// resets smallestCost in every node to zero.
		inline void    clearNodeCosts(){
			for(list<PGNode*>::iterator i=ivNodeList.begin();i!=ivNodeList.end();i++){
				(*i)->smallestCost = LONG_MAX;
			}
		}
    /// @return pointer to start node
    inline PGNode* getStartNode()      {return ivpStartNode;}
    /// @return pointer to end node
    inline PGNode* getEndNode()        {return ivpEndNode;}
    /** prints a ".map" file useful for plotting the graph
      * @param fn file name
      */
    void printGraphMap(const char*);
    /// save graph to file fn
    void    saveGraph(const char*);
    /// save path to file fn
    void    savePath(const char*);
    /// load graph from file fn
    void    loadGraph(const char*);
    ~PathGraph();
    void    insertOneDirEdge(PGNode*, PGNode*); ///< hack: do not use :)

    /// return an iterator to the first edge
    Iter firstEdge();
  private:
    list<PGNode*> ivNodeList;   ///< list of all nodes in graph
    PGNode*       ivpStartNode; ///< start node
    PGNode*       ivpEndNode;   ///< end node

    bool startOnGraph;          ///< false if start node is no part of the original graph
    bool endOnGraph;            ///< false if end node is no part of the original graph
};


#endif /* __PATHGRAPH_H__ */
