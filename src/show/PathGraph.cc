/*
 * PathGraph implementation
 *
 * Copyright (C)  Christof Soeger, Marcel Junker, Anton Fluegge, Hannes Schulz,
                  Andreas Nuechter, Kai Lingemann, Jan Elseberg
 *
 * Released under the GPL version 3.
 *
 */

#include "show/PathGraph.h"
#include <cstdio>

PGNode* PathGraph::insertNode(PointXY p){
  PGNode* n = new PGNode;
  n->point = p;
  ivNodeList.push_back(n);
  return n;
}
void PathGraph::insertNodeAndConnectToAll(PointXY p){
  PGNode* n = insertNode(p);
  list<PGNode*>::iterator iter;
  for( iter = ivNodeList.begin(); iter != ivNodeList.end(); iter++ ) {
    if((*iter)!=n)
      insertEdge(n,*iter);
  }
}

PGNode* PathGraph::insertUniqueNode(PointXY p){
  list<PGNode*>::iterator iter;
  for( iter = ivNodeList.begin(); iter != ivNodeList.end(); iter++ ) {
    if((*iter)->point.x == p.x && (*iter)->point.y == p.y)
      return *iter;
  }
  return insertNode(p);
}

void PathGraph::deleteNode(PGNode* n){
  list<PGNode*>::iterator i;
  list<PGNode*>::iterator j;
  for(i=ivNodeList.begin();i!=ivNodeList.end();i++){
    if(*i == n){
			while((j=(*i)->neighbours.begin()) != (*i)->neighbours.end())
        deleteEdge(*i,*j);
      delete *i;
      ivNodeList.erase(i);
      break;
    }
  }
}


bool PathGraph::setStartNode(PointXY p) {
  if (!startOnGraph)
    deleteNode(ivpStartNode);
  startOnGraph=false;
  PGNode* n=NULL;
  list<PGNode*>::iterator iter;
  for( iter = ivNodeList.begin(); iter != ivNodeList.end() && !startOnGraph; iter++ ) {
    if((*iter)->point.x == p.x && (*iter)->point.y == p.y) {
      n=*iter;
      startOnGraph=true;
    }
  }
  if (!startOnGraph) {
    n=insertNode(p);
  }
  ivpStartNode=n;
  return !startOnGraph;
}

bool PathGraph::setEndNode(PointXY p) {
  if (!endOnGraph) 
    deleteNode(ivpEndNode);
  endOnGraph=false;
  PGNode* n=NULL;
  list<PGNode*>::iterator iter;
  for( iter = ivNodeList.begin(); iter != ivNodeList.end() && !endOnGraph; iter++ ) {
    if((*iter)->point.x == p.x && (*iter)->point.y == p.y) {
      n=*iter;
      endOnGraph=true;
    }
  }
  if (!endOnGraph) {
    n=insertNode(p);
  }
  ivpEndNode=n;
  return !endOnGraph;
}



void PathGraph::printGraphMap(const char* fn){
  FILE *file;
  if((file = fopen(fn, "w"))==NULL){
    printf("# in PathGraph::printGraph : Could not open output file %s!\n", fn);
    return;
  }
#if 0 
  list<PGNode*>::iterator i,j;
  for(i=ivNodeList.begin();i!=ivNodeList.end();i++){
    for(j=(*i)->neighbours.begin();j!=(*i)->neighbours.end();j++){
      fprintf(file,"%i %i %i %i\n", (*i)->point.x,
                                  (*i)->point.y,
                                  (*j)->point.x,
                                  (*j)->point.y);
    }
  }
#else 
  PGEdge e;
  for(PathGraph::Iter i = this->firstEdge();!i.isLast();i++){
    e = *i;
    fprintf(file,"%f %f %f %f\n", e.start->point.x,
				  e.start->point.y,
				  e.end->point.x,
				  e.end->point.y);
  }
#endif
  fclose(file);
}
 
void PathGraph::savePath(const char* fn){
  FILE *file;
  if((file = fopen(fn, "w"))==NULL){
    printf("# in PathGraph::savePath : Could not open output file %s!\n", fn);
    return;
  }
  map<PGNode*,int> names;
  int i=0;
  list<PGNode*>::iterator it,it2;
  for(it=ivNodeList.begin();it!=ivNodeList.end();it++){
    names[*it] = i++;
  }
  for(it=ivNodeList.begin();it!=ivNodeList.end();it++){
    fprintf(file,"%i %f %f\n",names[*it], (*it)->point.x, (*it)->point.y);
  }
  fprintf(file,"x\n");
  i=0;
  for(it=ivNodeList.begin();it!=ivNodeList.end();it++,i++){
    fprintf(file,"%i %zu ",names[*it], (*it)->neighbours.size());
    for(it2=(*it)->neighbours.begin();it2!=(*it)->neighbours.end();it2++){
      fprintf(file,"%i ",names[*it2]);
    }
    fprintf(file,"\n");
  }
  fprintf(file,"x\n");
  //fprintf(file,"%i %i",names[ivpStartNode],names[ivpEndNode]);
  
  fclose(file);
  // printf("Printed graph to %s\n",fn);
}
 
void PathGraph::saveGraph(const char* fn){
  FILE *file;
  if((file = fopen(fn, "w"))==NULL){
    printf("# in PathGraph::saveGraph : Could not open output file %s!\n", fn);
    return;
  }
  map<PGNode*,int> names;
  int i=0;
  list<PGNode*>::iterator it,it2;
  for(it=ivNodeList.begin();it!=ivNodeList.end();it++){
    names[*it] = i++;
  }
  for(it=ivNodeList.begin();it!=ivNodeList.end();it++){
    fprintf(file,"%i %f %f\n",names[*it], (*it)->point.x, (*it)->point.y);
  }
  fprintf(file,"x\n");
  i=0;
  for(it=ivNodeList.begin();it!=ivNodeList.end();it++,i++){
    fprintf(file,"%i %zu ",names[*it], (*it)->neighbours.size());
    for(it2=(*it)->neighbours.begin();it2!=(*it)->neighbours.end();it2++){
      fprintf(file,"%i ",names[*it2]);
    }
    fprintf(file,"\n");
  }
  fprintf(file,"x\n");
  //fprintf(file,"%i %i",names[ivpStartNode],names[ivpEndNode]);
  
  fclose(file);
  // printf("Printed graph to %s\n",fn);
}
void PathGraph::loadGraph(const char* fn){
  FILE *file;
  if((file = fopen(fn, "r"))==NULL){
    printf("# in PathGraph::loadGraph : Could not open input file %s!\n", fn);
    return;
  }
  map<int,PGNode*> names;
  int name;
  PointXY p;
  int res;
  int i=0,j;
  char c;
  while(1){
    res = fscanf(file,"%i %f %f",&name,&(p.x),&(p.y));
    if(res != 3)
      break;
    i++;
    names[name] = this->insertNode(p);
  }
  res = fscanf(file,"%c",&c);
  int numNeighbours, neighbourName;
  i= 0;
  while(1){
    res = fscanf(file,"%i %i",&name,&numNeighbours);
    if(res != 2)
      break;
    i++;
    for(j=0; j<numNeighbours; j++){
      res = fscanf(file,"%i",&neighbourName);
      if(res != 1){
	printf("Break 2: fscanf = %i -- you should never see this line.\n",res);
	break;
      }
      this->insertOneDirEdge(names[name],names[neighbourName]);
    }
  }
  /*
  fscanf(file,"%c",&c);
  int startNr,endNr;
  fscanf(file,"%i %i",&startNr,&endNr);
  setStartNode(names[startNr]->point);
  setEndNode(names[endNr]->point);
  */
  fclose(file);
}

void PathGraph::insertEdge(PGNode* a, PGNode* b){
  a->neighbours.push_back(b);
  b->neighbours.push_back(a);
}

void PathGraph::insertOneDirEdge(PGNode* a, PGNode* b){
  a->neighbours.push_back(b);
}

void PathGraph::deleteEdge(PGNode* a, PGNode* b){
  a->neighbours.remove(b);
  b->neighbours.remove(a);
}

PathGraph::~PathGraph(){
  list<PGNode*>::iterator i;
  for(i=ivNodeList.begin();i!=ivNodeList.end();i++)
    delete *i;
}

PathGraph::Iter
PathGraph::firstEdge(){
  if(ivNodeList.empty()){
    return Iter(ivNodeList.end(),ivNodeList.end());
  }
  return Iter(ivNodeList.begin(),ivNodeList.end());
}
