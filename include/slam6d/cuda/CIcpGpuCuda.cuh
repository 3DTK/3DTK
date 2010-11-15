/** @file
 *  @brief GPU-ICP Algorithm
 *  @author Deyuan Qiu, University of Applied Sciences Bonn-Rhein-Sieg, Sankt Augustin, Germany.
 *                      Fraunhofer IAIS, Sankt Augustin, Germany.
 */

#ifndef CICPGPUCUDA_H
#define CICPGPUCUDA_H

#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <algorithm>					// min()
#include <time.h>

#include "ANN/ANN.h"					// ANN declarations
#include "ANN/ANNperf.h"				// kd-tree printing
#include "kd_tree.h"		  		// ANN node declaration

#include "newmat/newmat.h"
#include "newmat/newmatap.h"
using namespace NEWMAT;

#include "slam6d/cuda/CSystem.h"

/*
 * The maximum block size. For nVidia G80 architecture, 192 is suggested.
 */
//#define	BLOCKSIZE	512
//#define	BLOCKSIZE	256
#define	BLOCKSIZE	64
//#define	BLOCKSIZE	1


/*
 * The big enough size of the AoS that is going to allocate for the kd-tree.
 */
//#define	TREESIZE	262143
#define	TREESIZE  524288

#ifdef use_namespace
using namespace std;
#endif

/*
*	@class	CIcpGpuCuda
* 	@brief	Iterative Closest Point algorithm is implemented on a programmable graphic
* device. Kernels are implemented by CUDA (Compute Unified Device Architecture) GPGPU
* programming language (http://www.nvidia.com/object/cuda_home.html). To compile the code,
* nvcc compiler and related CUDA libraries must be installed. Kernel files are wrapped in
* CIcpGpuCuda_kernel.cuh and CIcpGpuCuda.cu. Attention: only NVidia GeForce G80 architecture
* and onwards graphic devices are garanteed to be supported.
* 	@author	Deyuan Qiu
* 	@date	2008.Nov.
*/
class CIcpGpuCuda{

public:

	/*
	*	standard constructor
	* @param	argc	passed from application main function.
	* @param	argv	passed from application main function.
	* @param	unWidth		the width of point cloud image.
	* @param	unHeight	the height of pint cloud image.
	*/
	CIcpGpuCuda(unsigned unWidth, unsigned unHeight, unsigned max_iter){
		init(unWidth, unHeight,max_iter);
	}

	/*
	*	standard destructor
	*/
	~CIcpGpuCuda();

	/*
	*	Set maximum iteration for ICP.
	* @param	unTimes	number of times ICP iterates maximally.
	*/
	void setMaxIteration(unsigned unTimes);

	/*
	*	Set maximum processing time for ICP, in milliseconds.
	* @param	dMilliseconds	ICP quits when elapsed time exceeds.
	*/
	void setMaxProcTime(double dMilliseconds);

	/*
	*	Set maximum deviation for ICP.
	* @param	dDeviation	ICP quits when specified deviation is achieved.
	*/
	void setMaxDeviation(double dDeviation);

	/*
	*	Set search radiuses for ICP. An iterative decreasing radius
	* is applied. Search radius decreases linearly from fRadiusMax to
	* fRadiusMin within unIterations iterations.
	* @param	fRadiusMax	Initial radius.
	* @param	fRadiusMin	Final radius.
	* @param	unIterations	Number of iterations, in which radius decreases.
	*/
	void setSearchRadius(float fRadiusMax, float fRadiusMin, unsigned unIterations);

	/*
	*	Get the number of points in point cloud. Model point cloud
	* and scene point cloud must have the same number of points.
	* @return	Number of points in point cloud.
	*/
	unsigned getSize(void);

	/*
	*	Get the the 2 dimensional pointer to the scene point cloud. Page-lock memory is
	* allocated and freed by the class. The data type must be casted to single pricision
	* float, and the array should be loaded as [3][N].
	* @return	2 dimensional pointer to scene point cloud.
	*/
	float** getScenePointer(void);

	/*
	*	Get the the 2 dimensional pointer to the model point cloud. Memory allocation
	* and freeing is handled by the class. The data type must be casted to double pricision
	* float, and the array should be loaded as [N][3]. Notice the difference to
	* getScenePointer().
	* @return	2 dimensional pointer to scene point cloud.
	*/
	double** getModelPointer(void);

	/*
	*	The method is called after point clouds are loaded, and before iteration() is called.
	*/
	void setPointClouds(void);

	/*
	*	ICP iterations.
	*/
	void iteration(void);

	/*
	*	Get the transformation matrix.
	* @return	The transformation matrix.
	*/
	Matrix* getMatrix(void);

	/**
     * These two functions are to set and get the tree pointer
     *  It is supposed to be created in scan file and passed to this class
	**/

	void setTreePointer(ANNkd_tree *);
     void getTreePointer(ANNkd_tree *&);

	double getTime(void);

        void setMinimums(float x, float y, float z);
        Matrix** getMatrices();

        void setTrans_Trans_inv(const double[], const double[]);

        void findNearestNeighbors(float, unsigned);

        Matrix fillHomoMatrix(Matrix* , double*);

        void computeCentroid(float*, float*, float*, float *&);
        
        Matrix computeHMatrix();

        void printMatrix(Matrix *);

     void setSize(unsigned int width, unsigned int height);	
	   
private:
	
	////////////////
	//cpu variables
	////////////////


	/*
	*	Initialization. Memories are allocated and default environmental state
	* is set. Called by standard constructor.
	* @param	unWidth		the width of point cloud image.
	* @param	unHeight	the height of pint cloud image.
	*/
	void init(unsigned unWidth, unsigned unHeight, unsigned max_iter);

	/*
	*	Initialization. Memories are allocated and default environmental state
	* is set. Called by init().
	* @param	unWidth		the width of point cloud image.
	* @param	unHeight	the height of pint cloud image.
	*/
	void setResolution(unsigned unWidth, unsigned unHeight);

	/*
	*	The internal search structure is set.
	*/
	void setTree(void);

	/*
	*	Set the model point cloud.
	*/
	void setModel(void);

	/*
	*	Set the scene point cloud.
	*/
	void setScene(void);

	/*
	*	Calculate the size of the search structure from its depth. The depth
	* is the number of levels of the tree. The search structure then is arraged
	* into a struture of arrays (SoA). A left-balanced binary tree is suggested.
	* @param	nDepth	Depth of the kd-tree.
	*/
	inline int depth2size(int nDepth);

	/*
	*	Rearrange the search structure into a structure of arrays (SoA). The rule
	* is: the N node's left child has the index of 2N, while the right child 2N+1.
	* Arrangement is fulfilled in recursion.
	* @param	root	pointer to the root node
	* @param	unStart	the currenet node
	*/
	void rearrange(ANNkd_ptr root, unsigned unStart);

	/*
	*	Get CUBLAS errors before this line.
	*/
	void getCublasErr(void);

	/*
	*	Get CUDA errors before this line.
	*/
	void getCudaErr(void);

	/*
	 * tree: structure of arrays (SoA)
	 */
	unsigned _unSizeTree;	//Size of the allocated memory.
	float* fSplit;
	unsigned* unIdx;
	unsigned* unAxis;
	bool* bIsLeaf;
	float* fLoBound;
	float* fHiBound;
	unsigned unSizeTree;	//Size of memory that the tree actually takes.

	/*
	 * transformation matrix
	 */
	float m[16];
        Matrix* trans;
        Matrix* trans_inv;

	/*
	 * kernel constants
	 */
	unsigned unSizeData;
	unsigned unNoThreads;
	unsigned unNoBlocks;
	unsigned _unWidth;
	unsigned _unHeight;

	/*
	 * search structure
	 */
	ANNkd_tree*	kdTree;
	ANNkdStats* st;

	/*
	 * data pointers
	 */
	float* fHstScn[3];
	float* fHstScnX;
	float* fHstScnY;
	float* fHstScnZ;
	double** h_idata;

	/*
	 * icp
	 */
	unsigned unMaxIteration;
	unsigned _unIterations;
	float fMaxProcTime;
	float fMaxDeviation;
	float _fSearchRadiusMax;
	float _fSearchRadiusMin;
	float _fRadiusStep;
	unsigned _unNoQSizeStep;
        float* temp_ones;
        float* ones;

	enum EnumIcpState { ICP_LIMIT 			= 0,
						ICP_PROCESSING 		= 1,
						ICP_NOTMATCHABLE 	= 2,
						ICP_MAXITERATIONS 	= 3,
						ICP_TIMEELAPSED 	= 4,
						ICP_SUCCESS 		= 5 };

	unsigned unPairs;
	unsigned* pNoPairs;
	Matrix* final_matrix;
	double _dElapsedTime;
        Matrix** matrices;

    /*
        Minimums of all values
    */
    float min_x;
    float min_y;
    float min_z;

	clock_t init_time; // To save the starting point of the timer Added by Shams

	////////////////
	//gpu variables
	////////////////

	float* fDevSplit;
	unsigned* unDevIdx;
	unsigned* unDevAxis;
	bool* bDevIsLeaf;
	float* fDevLoBound;
	float* fDevHiBound;
	float* fDevScnX;
	float* fDevScnY;
	float* fDevScnZ;
	float* fDist;
	float* fDistCpt; 			//compacted distance list
	unsigned* unMask;
	float* fDevMdlPairX;
	float* fDevMdlPairY;
	float* fDevMdlPairZ;
	float* fDevScnPairX;
	float* fDevScnPairY;
	float* fDevScnPairZ;

        /////////////
        float* cngfDevScnX;
        float* cngfDevScnY;
        float* cngfDevScnZ;
	float* cngfDevMdlPairX;
	float* cngfDevMdlPairY;
	float* cngfDevMdlPairZ;
	float* cngfDevScnPairX;
	float* cngfDevScnPairY;
	float* cngfDevScnPairZ;
        /////////////
	float* fCenModX;
	float* fCenModY;
	float* fCenModZ;
	float* fCenScnX;
	float* fCenScnY;
	float* fCenScnZ;
	unsigned* unNoPairs;

	//kd-tree based nearest neighbor search, using a priority queue.
	void class_nns_priority(
			float* fDevScnX,						//scene point cloud
			float* fDevScnY,
			float* fDevScnZ,
			float* fDist,							//squared distance between pairs, for deviation calculation
			float* fDevSplit, 						//kd-tree: position of splitting plain (inner node)
			unsigned* unDevIdx, 					//kd-tree: index of point (leaf node)
			unsigned* unDevAxis, 					//kd-tree: axis where splitting plain locates (inner node)
			bool* bDevIsLeaf, 						//kd-tree: node type (both nodes)
			float* fDevLoBound,						//kd-tree: lower bounding box (inner node)
			float* fDevHiBound,						//kd-tree: higher bounding box (inner node)
			unsigned* unMask,						//a 0-1 mask of pair and non-pairs.
			float* fDevMdlPairX,
			float* fDevMdlPairY,
			float* fDevMdlPairZ,
			float* fDevScnPairX,
			float* fDevScnPairY,
			float* fDevScnPairZ,
			float fSearchRadius,
			unsigned unSize,
			unsigned unWidth,
			unsigned unQStep);						//for dubugging thread


	//centralize a pointcloud
	void class_centralize(unsigned* unMask,
			float* fDevMdlPairX,
			float* fDevMdlPairY,
			float* fDevMdlPairZ,
			float* fDevScnPairX,
			float* fDevScnPairY,
			float* fDevScnPairZ,
			float fcm0,
			float fcm1,
			float fcm2,
			float fcs0,
			float fcs1,
			float fcs2,
			float* fCenteredModX,					//centered point cloud
			float* fCenteredModY,
			float* fCenteredModZ,
			float* fCenteredScnX,
			float* fCenteredScnY,
			float* fCenteredScnZ);

	//transform point cloud
	void class_transformation(float* fDevScnX,					//piont cloud to be transformed
			float* fDevScnY,
			float* fDevScnZ,
			float m00,	float m01,	float m02,	float m03,
			float m10,	float m11,	float m12,	float m13,
			float m20,	float m21,	float m22,	float m23);
};



#endif
