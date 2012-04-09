/************************************************************************
*
*  lap.cpp
   version 1.0 - 4 September 1996
   author: Roy Jonker @ MagicLogic Optimization Inc.
   e-mail: roy_jonker@magiclogic.com

   Code for Linear Assignment Problem, according to 
   
   "A Shortest Augmenting Path Algorithm for Dense and Sparse Linear   
    Assignment Problems," Computing 38, 325-340, 1987
   
   by
   
   R. Jonker and A. Volgenant, University of Amsterdam.
*
*************************************************************************/

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#define BIG 100000

#if !defined TRUE
#define	 TRUE		1
#endif
#if !defined FALSE
#define  FALSE		0
#endif

typedef int Boolean;
typedef int row;
typedef int col;
typedef double cost;

cost lap(int dim, cost **assigncost,col *rowsol, row *colsol,cost *u,cost *v);

void checklap(int dim,cost **assigncost,col *rowsol,row *colsol,cost *u,cost *v);

