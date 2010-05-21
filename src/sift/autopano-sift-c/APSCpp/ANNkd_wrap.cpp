/** ANNkd_wrap.cpp  08 Mar 2008 TKSharpless

C-callable wrapper for ANN kd-tree search, for autopano-sift-c
The objective is to allow all legacy code to be compiled as c
because it is full of standards-violating pointer usages (that
MSVC++ doesn't notice).

**/

#include "ANN/ANN.h"

static ANNkd_tree * pkd = 0;

extern "C" 
void ANNkd_new( int sd, double ** pts, int n, int d ){

	annMaxPtsVisit( sd );

	pkd = new ANNkd_tree (		// build from point array
					pts,			// point array
					n,				// number of points
					d,				// dimension
					1,				// bucket size
				ANN_KD_SUGGEST);	// splitting method	


}

extern "C"
void ANNkd_search( double * pt, int nnabe, int *indx, double *dist, double nneps ){

	pkd->annkSearch( pt, nnabe, indx, dist, nneps );

}

extern "C" 
void ANNkd_delete( void ){

	if( pkd ) delete pkd;
	pkd = 0;

}

