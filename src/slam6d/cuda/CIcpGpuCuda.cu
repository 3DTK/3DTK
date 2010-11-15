/** @file
 *  @brief GPU-ICP Algorithm
 *  @author Deyuan Qiu, University of Applied Sciences Bonn-Rhein-Sieg, Sankt Augustin, Germany.
 *                      Fraunhofer IAIS, Sankt Augustin, Germany.
 */


#include "slam6d/cuda/CIcpGpuCuda_kernel.cuh"
#include "slam6d/cuda/CIcpGpuCuda.cuh"
#include "slam6d/cuda/CSystem.h"

void CIcpGpuCuda::init(unsigned unWidth, unsigned unHeight, unsigned max_iter)
{
	// Initialize CUTIL
	int d;
	cudaSetDevice(0); // Since we have only one GPU I didn't initialize anything
	
	// It may cause problems if more GPUs take into account
	// The reason of doing that was initialization of it several times
	// Now it is not initialized several times and just once.

        unMaxIteration = max_iter;
        matrices = (Matrix**)malloc(sizeof(Matrix*) * max_iter);
        for(int i = 0 ; i < max_iter ; ++i){
            matrices[i] = new Matrix(4,4);
            Matrix* m = matrices[i];
            (*m)(1,1) = 1;(*m)(1,2) = 0;(*m)(1,3) = 0;(*m)(1,4) = 0;
            (*m)(2,1) = 0;(*m)(2,2) = 1;(*m)(2,3) = 0;(*m)(2,4) = 0;
            (*m)(3,1) = 0;(*m)(3,2) = 0;(*m)(3,3) = 1;(*m)(3,4) = 0;
            (*m)(4,1) = 0;(*m)(4,2) = 0;(*m)(4,3) = 0;(*m)(4,4) = 1;
        }

    // set data size
    setResolution(unWidth, unHeight);

    //    cout<<"unSizeData: "<<unSizeData<<endl;
    //    cout<<"Tree Size :"	<<TREESIZE<<endl;

    // Initialize CUBLAS
    cublasStatus statusCUBLAS = cublasInit();
    if (statusCUBLAS != CUBLAS_STATUS_SUCCESS) {
        cout<<"The error status is \n";
        cout<<statusCUBLAS<<endl;
        fprintf (stderr, "!!!! CUBLAS initialization error\n");
        exit(1);
    }

    // Initialize CUDPP
    CUDPPConfiguration config;
    config.datatype = CUDPP_FLOAT;
    config.algorithm = CUDPP_COMPACT;
    config.options = CUDPP_OPTION_FORWARD;
    result = cudppPlan(&compactplan, config, unSizeData, 1, 0);
    if (CUDPP_SUCCESS != result)	printf("Error creating CUDPPPlan\n");

    _unSizeTree = (unsigned)TREESIZE;
    CUDA_SAFE_CALL(cudaMallocHost((void**)&fSplit, _unSizeTree*sizeof(float)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&unIdx, _unSizeTree*sizeof(unsigned)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&unAxis, _unSizeTree*sizeof(unsigned)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&bIsLeaf, _unSizeTree*sizeof(bool)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&fLoBound, _unSizeTree*sizeof(float)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&fHiBound, _unSizeTree*sizeof(float)));

    CUDA_SAFE_CALL(cudaMallocHost((void**)&f4Mdl, unSizeData*sizeof(float4)));  // to be downloaded to texture

    // Host memory allocation
    CUDA_SAFE_CALL(cudaMallocHost((void**)&fHstScnX, unSizeData*sizeof(float)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&fHstScnY, unSizeData *sizeof(float)));
    CUDA_SAFE_CALL(cudaMallocHost((void**)&fHstScnZ, unSizeData*sizeof(float)));	//scene

    fHstScn[0]=fHstScnX;
    fHstScn[1]=fHstScnY;
    fHstScn[2]=fHstScnZ;

    CUDA_SAFE_CALL(cudaMallocHost((void**)&pNoPairs, sizeof(unsigned)));

    CSystem<double>::allocate(unSizeData, 3, h_idata);							//model

    // Device memory allocation
    CUDA_SAFE_CALL(cudaMalloc((void**)&fDist, unSizeData*sizeof(float)));
    CUDA_SAFE_CALL(cudaMalloc((void**)&fDistCpt, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&unMask, unSizeData*sizeof(unsigned)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevMdlPairX, unSizeData*sizeof(float)));	//pairs after shrinking
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevMdlPairY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevMdlPairZ, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevScnPairX, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevScnPairY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevScnPairZ, unSizeData*sizeof(float)));
        /////////////// Added by Shams
     CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevScnX,unSizeData*sizeof(float)));
     CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevScnY,unSizeData*sizeof(float)));
     CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevScnZ,unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevMdlPairX, unSizeData*sizeof(float)));	//pairs after shrinking
	CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevMdlPairY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevMdlPairZ, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevScnPairX, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevScnPairY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&cngfDevScnPairZ, unSizeData*sizeof(float)));

        ///////////////
	CUDA_SAFE_CALL(cudaMalloc((void**)&fCenModX, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fCenModY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fCenModZ, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fCenScnX, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fCenScnY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fCenScnZ, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&unNoPairs, sizeof(unsigned)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevScnX, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevScnY, unSizeData*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevScnZ, unSizeData*sizeof(float)));

	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevSplit, _unSizeTree*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&unDevIdx, _unSizeTree*sizeof(unsigned)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&unDevAxis, _unSizeTree*sizeof(unsigned)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&bDevIsLeaf, _unSizeTree*sizeof(bool)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevLoBound, _unSizeTree*sizeof(float)));
	CUDA_SAFE_CALL(cudaMalloc((void**)&fDevHiBound, _unSizeTree*sizeof(float)));

	CUDA_SAFE_CALL(cudaMallocArray(&cuArray, &cuDesc, _unWidth, _unHeight));		//to be bound to texture

	// Initialize states
	fMaxProcTime		=	0.0f;
	fMaxDeviation		=	0.0f;
	_fSearchRadiusMax	=	0.0f;
	_fSearchRadiusMin	=	0.0f;
	_fRadiusStep		=	0.0f;
	_unNoQSizeStep		=	0;
	_dElapsedTime		=	0.0;

        /*
	    * Array of ones to be used instead of abs sum
        */
	cudaMallocHost((void**)&temp_ones, unSizeData*sizeof(float));
	cudaMalloc((void**)&ones, unSizeData*sizeof(float));	// Array of ones
     for(int i = 0; i < unSizeData ; ++i)temp_ones[i] = 1.0f;
	cudaMemcpy(ones, temp_ones, unSizeData*sizeof(float), cudaMemcpyHostToDevice);
}

CIcpGpuCuda::~CIcpGpuCuda(){

    /////////////
	// tidy up
    /////////////
	CUDA_SAFE_CALL(cudaUnbindTexture(refTex));
	CUDA_SAFE_CALL(cudaFreeArray(cuArray));
	CUDA_SAFE_CALL(cudaFree(fDevSplit));
	CUDA_SAFE_CALL(cudaFree(unDevIdx));
	CUDA_SAFE_CALL(cudaFree(unDevAxis));
	CUDA_SAFE_CALL(cudaFree(bDevIsLeaf));
	CUDA_SAFE_CALL(cudaFree(fDevLoBound));
	CUDA_SAFE_CALL(cudaFree(fDevHiBound));
	CUDA_SAFE_CALL(cudaFree(fDevScnX));
	CUDA_SAFE_CALL(cudaFree(fDevScnY));
	CUDA_SAFE_CALL(cudaFree(fDevScnZ));
	CUDA_SAFE_CALL(cudaFree(fDist));
	CUDA_SAFE_CALL(cudaFree(fDistCpt));
	CUDA_SAFE_CALL(cudaFree(fCenModX));
	CUDA_SAFE_CALL(cudaFree(fCenModY));
	CUDA_SAFE_CALL(cudaFree(fCenModZ));
	CUDA_SAFE_CALL(cudaFree(fCenScnX));
	CUDA_SAFE_CALL(cudaFree(fCenScnY));
	CUDA_SAFE_CALL(cudaFree(fCenScnZ));
	CUDA_SAFE_CALL(cudaFree(unMask));
	CUDA_SAFE_CALL(cudaFree(fDevMdlPairX));
	CUDA_SAFE_CALL(cudaFree(fDevMdlPairY));
	CUDA_SAFE_CALL(cudaFree(fDevMdlPairZ));
	CUDA_SAFE_CALL(cudaFree(fDevScnPairX));
	CUDA_SAFE_CALL(cudaFree(fDevScnPairY));
	CUDA_SAFE_CALL(cudaFree(fDevScnPairZ));
     CUDA_SAFE_CALL(cudaFree(cngfDevScnX));
     CUDA_SAFE_CALL(cudaFree(cngfDevScnY));
     CUDA_SAFE_CALL(cudaFree(cngfDevScnZ));
	CUDA_SAFE_CALL(cudaFree(cngfDevMdlPairX));
	CUDA_SAFE_CALL(cudaFree(cngfDevMdlPairY));
	CUDA_SAFE_CALL(cudaFree(cngfDevMdlPairZ));
	CUDA_SAFE_CALL(cudaFree(cngfDevScnPairX));
	CUDA_SAFE_CALL(cudaFree(cngfDevScnPairY));
	CUDA_SAFE_CALL(cudaFree(cngfDevScnPairZ));

	CUDA_SAFE_CALL(cudaFree(unNoPairs));
	CUDA_SAFE_CALL(cudaFree(ones));


	CUDA_SAFE_CALL(cudaFreeHost(fSplit));
	CUDA_SAFE_CALL(cudaFreeHost(unIdx));
	CUDA_SAFE_CALL(cudaFreeHost(unAxis));
	CUDA_SAFE_CALL(cudaFreeHost(bIsLeaf));
	CUDA_SAFE_CALL(cudaFreeHost(fHstScnX));
	CUDA_SAFE_CALL(cudaFreeHost(fHstScnY));
	CUDA_SAFE_CALL(cudaFreeHost(fHstScnZ));
	CUDA_SAFE_CALL(cudaFreeHost(fLoBound));
	CUDA_SAFE_CALL(cudaFreeHost(fHiBound));
	CUDA_SAFE_CALL(cudaFreeHost(pNoPairs));
	CUDA_SAFE_CALL(cudaFreeHost(f4Mdl));
	CUDA_SAFE_CALL(cudaFreeHost(temp_ones));

	free(h_idata);


    /////////
    // Exit
    /////////
    bool bShutDownSuccess = true;

    // Done with CUDPP
    result = cudppDestroyPlan(compactplan);
    if (CUDPP_SUCCESS != result){
    	printf("Error destroying CUDPPPlan\n");
    	bShutDownSuccess = false;
    }

    // Done with CUBLAS
    cublasStatus statusCUBLAS = cublasShutdown();
    if (statusCUBLAS != CUBLAS_STATUS_SUCCESS) {
        fprintf (stderr, "!!!! shutdown error (A)\n");
        bShutDownSuccess = false;
    }

    // Done with ANN
    annClose();

}

void CIcpGpuCuda::setResolution(unsigned unWidth, unsigned unHeight){
	if (unWidth>0 && unHeight>0) {
		_unWidth = unWidth;
		_unHeight = unHeight;
		unSizeData = _unWidth*_unHeight;

		//configure block and grid size
		unsigned unBlockSize = (unsigned)BLOCKSIZE;	//192
		if(unSizeData<=64){
			unNoThreads = 64;
			unNoBlocks = 1;
		}
		else if(unSizeData<=128){
			unNoThreads = 128;
			unNoBlocks = 1;
		}
		else if(unSizeData<=unBlockSize){
			unNoThreads = unBlockSize;
			unNoBlocks = 1;
		}
		else{
			unNoThreads = unBlockSize;
			if(unSizeData%unBlockSize)	unNoBlocks = unSizeData/unBlockSize + 1;
			else	unNoBlocks = unSizeData/unBlockSize;
		}
		cout<<"unNoThreads: "<<unNoThreads<<'\t'<<"unNoBlocks: "<<unNoBlocks<<endl;
		dimBlock.x=unNoThreads;
		dimBlock.y=1;
		dimBlock.z=1;
		dimGrid.x=unNoBlocks;
		dimGrid.y=1;
		dimGrid.z=1;
	}
	else{
		cout<<"Each aspect of resolution must be bigger than 0."<<endl;
		exit(1);
	}
}

void CIcpGpuCuda::setMaxIteration(unsigned unTimes){
	if(unTimes>0)	unMaxIteration = unTimes;
	else{
		cout<<"Error setting maximum iterations."<<endl;
		exit(1);
	}
}

void CIcpGpuCuda::setMaxProcTime(double dMilliseconds){
	if(dMilliseconds>0)	fMaxProcTime = dMilliseconds;
	else{
		cout<<"Error setting maximum processing time."<<endl;
		exit(1);
	}
}

void CIcpGpuCuda::setMaxDeviation(double fDeviation){
	if(fDeviation>0)	fMaxDeviation = fDeviation;
	else{
		cout<<"Error setting maximum deviation."<<endl;
		exit(1);
	}
}

void CIcpGpuCuda::setSize(unsigned int width, unsigned int height){
    //The memory allocation is once done for all scans
    //we require to update the sizes for each scan matching process
    //The size which is set in constructor is the maximum possible,
    //and this size is associated to each pair.
    setResolution(width, height);
    cout<<"unSizeData: "<<unSizeData<<endl;

}

void CIcpGpuCuda::setSearchRadius(float fRadiusMax, float fRadiusMin, unsigned unIterations){
	if((fRadiusMax>=fRadiusMin)&&(unIterations>0)){
		_fSearchRadiusMax = fRadiusMax;
		_fSearchRadiusMin = fRadiusMin;
		_fRadiusStep = (_fSearchRadiusMax-_fSearchRadiusMin)/(float)unIterations;
		_unIterations = unIterations;
		_unNoQSizeStep = unIterations/(unsigned)NO_QSIZE;
	}
	else{
		cout<<"Error setting search radius."<<endl;
		exit(1);
	}
}

float** CIcpGpuCuda::getScenePointer() {
	return fHstScn;
}

double** CIcpGpuCuda::getModelPointer(void){
	return h_idata;
}

unsigned CIcpGpuCuda::getSize(void){
	return unSizeData;
}

void CIcpGpuCuda::setTreePointer(ANNkd_tree *tree){
    kdTree = tree;
}
void CIcpGpuCuda::getTreePointer(ANNkd_tree *&tree ){
    tree = kdTree;
}


void CIcpGpuCuda::setTree()
{

	// preparation
	st = new ANNkdStats();
	kdTree->getStats(*st);
	int nDepth = st->depth;
     // cout<<"level of the tree: "<<st->depth<<" (counted from 0)"<<endl;
	// decide size of array to be uploaded to GPU
	unSizeTree = depth2size(nDepth);
	// cout<<"unSizeTree: "<<unSizeTree<<endl;

	if (unSizeTree>_unSizeTree) {
		cout << "Not enough memory for tree construction. Tree size must be smaller than "
		     << unSizeTree <<endl;
		exit(1);
	}

	// rearrange
	ANNkd_split* pRoot = (ANNkd_split*)kdTree->getRoot();
	if (unSizeData>1) {
	  rearrange(pRoot, 1);
	} else {
	  cout<<"Not enough points in the tree."<<endl;
	  exit(1);
	}

	// download the tree
	CUDA_SAFE_CALL(cudaMemcpy(fDevSplit, fSplit, unSizeTree*sizeof(float), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(unDevIdx, unIdx, unSizeTree*sizeof(unsigned), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(unDevAxis, unAxis, unSizeTree*sizeof(unsigned), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(bDevIsLeaf, bIsLeaf, unSizeTree*sizeof(bool), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(fDevLoBound, fLoBound, unSizeTree*sizeof(float), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(fDevHiBound, fHiBound, unSizeTree*sizeof(float), cudaMemcpyHostToDevice));

	// clean up
//	delete kdTree;
	delete st;
}

void CIcpGpuCuda::setModel()
{
	// using texture memory
	for(unsigned i = 0; i < unSizeData; i++) {	// type cast
	  f4Mdl[i].x=(float)h_idata[i][0];
	  f4Mdl[i].y=(float)h_idata[i][1];
	  f4Mdl[i].z=(float)h_idata[i][2];
	}
	cudaMemcpyToArray(cuArray,0,0,f4Mdl,unSizeData*sizeof(float4),cudaMemcpyHostToDevice);
	cudaBindTextureToArray(refTex,cuArray);
}

void CIcpGpuCuda::setScene()
{
	CUDA_SAFE_CALL(cudaMemcpy(fDevScnX, fHstScnX, unSizeData*sizeof(float), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(fDevScnY, fHstScnY, unSizeData*sizeof(float), cudaMemcpyHostToDevice));
	CUDA_SAFE_CALL(cudaMemcpy(fDevScnZ, fHstScnZ, unSizeData*sizeof(float), cudaMemcpyHostToDevice));
}

inline int CIcpGpuCuda::depth2size(int nDepth)
{
	double nSize = 0;
	for(int i = 0;i <= nDepth; i++)
	{
		nSize += pow(2.0,(double)i);
	}
	return (int)(nSize+0.5);
}

void CIcpGpuCuda::rearrange(ANNkd_ptr root, unsigned unStart)
{
	if(root!=NULL && root!=KD_TRIVIAL){
		if (root->isLeaf()) {
			bIsLeaf[unStart-1] = true;
			unIdx[unStart-1] = (unsigned)(((ANNkd_leaf*)root)->getIdxArray())[0];
		}
		else {
			ANNkd_ptr nL = ((ANNkd_split*)root)->getLeftChild();
			ANNkd_ptr nR = ((ANNkd_split*)root)->getRightChild();

			bIsLeaf[unStart-1] = false;
			fSplit[unStart-1] = (float)((ANNkd_split*)root)->getCutVal();
			unAxis[unStart-1] = (unsigned)((ANNkd_split*)root)->getCutDim();
			fLoBound[unStart-1] = ((ANNkd_split*)root)->getLoBound();
			fHiBound[unStart-1] = ((ANNkd_split*)root)->getHiBound();
			rearrange(nL, unStart*2);
			rearrange(nR, unStart*2+1);
		}
	}
}

void CIcpGpuCuda::setPointClouds(void){
	setTree();
	setModel();
	setScene();
}

void CIcpGpuCuda::iteration(){

	//////////////////////////
	// prepare for iteration
	//////////////////////////

	EnumIcpState icpStat = ICP_PROCESSING;

	float fSearchRadius = _fSearchRadiusMax;
	unsigned unNoIter = 0;
	float fDeviation = 0.0;
	unsigned unQStep = 0;

	final_matrix = new Matrix(4,4);
	(*final_matrix)(1,1) = 1.0;(*final_matrix)(1,2) = 0.0;(*final_matrix)(1,3) = 0.0;(*final_matrix)(1,4) = 0.0;
	(*final_matrix)(2,1) = 0.0;(*final_matrix)(2,2) = 1.0;(*final_matrix)(2,3) = 0.0;(*final_matrix)(2,4) = 0.0;
	(*final_matrix)(3,1) = 0.0;(*final_matrix)(3,2) = 0.0;(*final_matrix)(3,3) = 1.0;(*final_matrix)(3,4) = 0.0;
	(*final_matrix)(4,1) = 0.0;(*final_matrix)(4,2) = 0.0;(*final_matrix)(4,3) = 0.0;(*final_matrix)(4,4) = 1.0;

	Matrix matrix(4,4);
	
	init_time = clock();

     // The main loop of ICP
	while(icpStat == ICP_PROCESSING){
       if (unNoIter <= _unIterations)
		unQStep = unNoIter/_unNoQSizeStep;

            findNearestNeighbors(fSearchRadius, unQStep);

            result=cudppCompact(compactplan, fDistCpt, (size_t*)unNoPairs,
                     fDist, unMask, (size_t)unSizeData);

	    if (CUDPP_SUCCESS != result)
                printf("Error cudppCompact\n");
            CUDA_SAFE_CALL(cudaMemcpy(pNoPairs, unNoPairs, sizeof(unsigned), cudaMemcpyDeviceToHost));
            unPairs = *pNoPairs;

            if (unPairs) {

		//////////////////////
		// transform estimation
		//////////////////////

        	// Compute centroids (assume all data are non-negative)
		float *fCm = new float(3);
		float *fCs = new float(3);


                computeCentroid(fDevMdlPairX,fDevMdlPairY,fDevMdlPairZ, fCm);
                computeCentroid(fDevScnPairX,fDevScnPairY,fDevScnPairZ, fCs);

                fDeviation = cublasSdot(unSizeData,fDistCpt,1,ones,1);
                fDeviation /= unPairs;
                getCublasErr();

		// check for termination conditions
		unNoIter++;
		if(unNoIter<_unIterations)	
			fSearchRadius-=_fRadiusStep;
		if(fDeviation <= fMaxDeviation)
			icpStat = ICP_SUCCESS;
		else if(unNoIter >= unMaxIteration + 1) // unNoIter starts from 1
			icpStat = ICP_MAXITERATIONS;
		else if ( (double)(clock() - init_time)/ (double)CLOCKS_PER_SEC * 1000>= fMaxProcTime )
			icpStat = ICP_TIMEELAPSED;
		else
		{
			//Calculate centered point pairs
			class_centralize(unMask,
					fDevMdlPairX,fDevMdlPairY,fDevMdlPairZ,fDevScnPairX,fDevScnPairY,fDevScnPairZ,
					fCm[0],fCm[1],fCm[2],fCs[0],fCs[1],fCs[2],
					fCenModX,fCenModY,fCenModZ,fCenScnX,fCenScnY,fCenScnZ);

			//Fill H matrix
                        Matrix H(3,3);
                        H = computeHMatrix();

			//SVD
			Matrix U(3,3);
			DiagonalMatrix Lamda(3);
			Matrix V(3,3);
			SVD(H,Lamda,U,V);

			//Get rotation
			Matrix R(3,3);
			R = V*(U.t());

			// Calculate translation
			double dTranslation[3];
			ColumnVector col_vec(3);
			for(unsigned j = 0; j < 3; j++)
				col_vec(j+1) = fCs[j];

			ColumnVector r_time_colVec = ColumnVector(R*col_vec);

			dTranslation[0] = fCm[0] - r_time_colVec(1);
			dTranslation[1] = fCm[1] - r_time_colVec(2);
			dTranslation[2] = fCm[2] - r_time_colVec(3);

               matrix = fillHomoMatrix(&R,dTranslation);

			*final_matrix = matrix * (*final_matrix);

                        for(int i = 1 ; i < 5 ; ++i)
                            for(int j = 1; j < 5 ; ++j){
                                (*matrices[unNoIter - 1])(i,j) = (matrix)(i,j);
                            }

			/////////////
			// transform
			/////////////

			class_transformation(fDevScnX, fDevScnY, fDevScnZ,
					(float)matrix(1,1),	(float)matrix(1,2),	(float)matrix(1,3),	(float)matrix(1,4),
					(float)matrix(2,1),	(float)matrix(2,2),	(float)matrix(2,3),	(float)matrix(2,4),
					(float)matrix(3,1),	(float)matrix(3,2),	(float)matrix(3,3),	(float)matrix(3,4));


			///////////////
			// termination
			///////////////

			}
		}//if(unPairs)
		else	icpStat = ICP_NOTMATCHABLE;
	}//while(icpStat == ICP_PROCESSING)

	_dElapsedTime = (double)(clock() - init_time)/(double)CLOCKS_PER_SEC * 1000.0;		//temporary
	cout<<"=========="<<endl;
	switch(icpStat)
	{
		case	ICP_LIMIT:	
        		cout<<"terminated: convergent limit reached."<<endl;
                        break;
		case	ICP_NOTMATCHABLE:	
                        cout<<"terminated: point clounds not matchable."<<endl;
				    /*
    	(*final_matrix)(1,1) = 1.0;(*final_matrix)(1,2) = 0.0;(*final_matrix)(1,3) = 0.0;(*final_matrix)(1,4) = 0.0;
	(*final_matrix)(2,1) = 0.0;(*final_matrix)(2,2) = 1.0;(*final_matrix)(2,3) = 0.0;(*final_matrix)(2,4) = 0.0;
	(*final_matrix)(3,1) = 0.0;(*final_matrix)(3,2) = 0.0;(*final_matrix)(3,3) = 1.0;(*final_matrix)(3,4) = 0.0;
	(*final_matrix)(4,1) = 0.0;(*final_matrix)(4,2) = 0.0;(*final_matrix)(4,3) = 0.0;(*final_matrix)(4,4) = 1.0;
*/
                        break;
		case	ICP_MAXITERATIONS:	
                        cout<<"terminated: maximum iteration exceeds."<<endl;
                        break;
		case	ICP_TIMEELAPSED:	
                        cout<<"terminated: maximum time elapsed."<<endl;
                	break;
		case	ICP_SUCCESS:	
                	cout<<"succeeded: maximum deviation reached."<<endl;
                        break;
	}
	cout<<"elapsed time:\t"<<_dElapsedTime<<"ms"<<endl;
	cout<<"iterations:\t"<<unNoIter - 1<<endl;
	cout<<"deviation:\t"<<fDeviation<<endl;
}

double CIcpGpuCuda::getTime(void){
	return _dElapsedTime;
}

Matrix* CIcpGpuCuda::getMatrix(void){
	return final_matrix;
}

void CIcpGpuCuda::getCublasErr()
{
	cublasStatus statusCUBLAS;
	statusCUBLAS = cublasGetError();
    if (statusCUBLAS != CUBLAS_STATUS_SUCCESS) {
    	cout<<"CUBLAS error: ";
    	switch(statusCUBLAS){
    	case CUBLAS_STATUS_NOT_INITIALIZED:	cout<<"CUBLAS library not initialized"<<endl;break;
    	case CUBLAS_STATUS_ALLOC_FAILED:	cout<<"resource allocation failed"<<endl;break;
    	case CUBLAS_STATUS_INVALID_VALUE:	cout<<"unsupported numerical value was passed to function"<<endl;break;
    	case CUBLAS_STATUS_MAPPING_ERROR:	cout<<"access to GPU memory space failed"<<endl;break;
    	case CUBLAS_STATUS_EXECUTION_FAILED:	cout<<"GPU program failed to execute"<<endl;break;
    	case CUBLAS_STATUS_INTERNAL_ERROR:	cout<<"an internal CUBLAS operation failed"<<endl;break;
    	default:	cout<<"undefined error"<<endl;
    	}
    }
//    	cout<<"error number: "<<statusCUBLAS<<endl;
//        fprintf (stderr, "CUBLAS error.\n");};
}

void CIcpGpuCuda::getCudaErr(void){
	cudaError_t error=cudaGetLastError();
	cout<<cudaGetErrorString(error)<<endl;
}

//kd-tree based nearest neighbor search, using a priority queue: no parameters are needed??
void CIcpGpuCuda::class_nns_priority(
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
		unsigned unQStep)
{
   wrapper_nns_priority( fDevScnX, fDevScnY, fDevScnZ,
		             	fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
			          unMask,
					fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
			          dimGrid, dimBlock, fSearchRadius,
					unSize, unWidth,
					unQStep );
}


//centralize a pointcloud
void CIcpGpuCuda::class_centralize(unsigned* unMask,
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
		float* fCenteredScnZ){
	wrapper_centralize(unMask,
			fDevMdlPairX,fDevMdlPairY,fDevMdlPairZ,fDevScnPairX,fDevScnPairY,fDevScnPairZ,
			fcm0,fcm1,fcm2,fcs0,fcs1,fcs2,
			fCenteredModX,fCenteredModY,fCenteredModZ,fCenteredScnX,fCenteredScnY,fCenteredScnZ,
			dimGrid, dimBlock);
}

// transform point cloud
void CIcpGpuCuda::class_transformation(float* fDevScnX,	// point cloud to be transformed
		                             float* fDevScnY,
		                             float* fDevScnZ,
		                             float m00, float m01, float m02, float m03,
		                             float m10, float m11, float m12, float m13,
		                             float m20, float m21, float m22, float m23)
{
   wrapper_transformation(fDevScnX, fDevScnY, fDevScnZ,
			           m00, m01, m02, m03,
			           m10, m11, m12, m13,
                          m20, m21, m22, m23,
                          dimGrid, dimBlock);
}


void CIcpGpuCuda::setMinimums(float x, float y, float z)
{
    min_x = x;
    min_y = y;
    min_z = z;
}
        
Matrix** CIcpGpuCuda::getMatrices(){
    return matrices;
}

void CIcpGpuCuda::setTrans_Trans_inv(const double tr[], const double tr_inv[]){
    trans = new Matrix(4,4);
    trans_inv = new Matrix(4,4);
    (*trans)(1,1) = tr[0]; (*trans)(2,1)=tr[1];(*trans)(3,1)=tr[2];(*trans)(4,1)=tr[3];
    (*trans)(1,2) = tr[4]; (*trans)(2,2)=tr[5];(*trans)(3,2)=tr[6];(*trans)(4,2)=tr[7];
    (*trans)(1,3) = tr[8]; (*trans)(2,3)=tr[9];(*trans)(3,3)=tr[10];(*trans)(4,3)=tr[11];
    (*trans)(1,4) = tr[12]; (*trans)(2,4)=tr[13];(*trans)(3,4)=tr[14];(*trans)(4,4)=tr[15];

    (*trans_inv)(1,1) = tr_inv[0]; (*trans_inv)(2,1)=tr_inv[1];(*trans_inv)(3,1)=tr_inv[2];(*trans_inv)(4,1)=tr_inv[3];
    (*trans_inv)(1,2) = tr_inv[4]; (*trans_inv)(2,2)=tr_inv[5];(*trans_inv)(3,2)=tr_inv[6];(*trans_inv)(4,2)=tr_inv[7];
    (*trans_inv)(1,3) = tr_inv[8]; (*trans_inv)(2,3)=tr_inv[9];(*trans_inv)(3,3)=tr_inv[10];(*trans_inv)(4,3)=tr_inv[11];
    (*trans_inv)(1,4) = tr_inv[12]; (*trans_inv)(2,4)=tr_inv[13];(*trans_inv)(3,4)=tr_inv[14];(*trans_inv)(4,4)=tr_inv[15];

}


void CIcpGpuCuda::findNearestNeighbors(float fSearchRadius, unsigned unQStep)
{
                // We make a copy of the existing scene point cloud in order to transform it
                cudaMemcpy(cngfDevScnX, fDevScnX, unSizeData*sizeof(float), cudaMemcpyDeviceToDevice);
                cudaMemcpy(cngfDevScnY, fDevScnY, unSizeData*sizeof(float), cudaMemcpyDeviceToDevice);
                cudaMemcpy(cngfDevScnZ, fDevScnZ, unSizeData*sizeof(float), cudaMemcpyDeviceToDevice);
                //////

                class_transformation(cngfDevScnX, cngfDevScnY, cngfDevScnZ ,
                    (*trans_inv)(1,1), (*trans_inv)(1,2), (*trans_inv)(1,3), (*trans_inv)(1,4),
                    (*trans_inv)(2,1), (*trans_inv)(2,2), (*trans_inv)(2,3), (*trans_inv)(2,4),
                    (*trans_inv)(3,1), (*trans_inv)(3,2), (*trans_inv)(3,3), (*trans_inv)(3,4)
                    );

		class_nns_priority( cngfDevScnX, cngfDevScnY, cngfDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				fSearchRadius, unSizeData, _unWidth, unQStep);

                class_transformation(fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ,
                    (*trans)(1,1), (*trans)(1,2), (*trans)(1,3), (*trans)(1,4),
                    (*trans)(2,1), (*trans)(2,2), (*trans)(2,3), (*trans)(2,4),
                    (*trans)(3,1), (*trans)(3,2), (*trans)(3,3), (*trans)(3,4)
                );

                class_transformation(fDevScnPairX, fDevScnPairY, fDevScnPairZ,
                    (*trans)(1,1), (*trans)(1,2), (*trans)(1,3), (*trans)(1,4),
                    (*trans)(2,1), (*trans)(2,2), (*trans)(2,3), (*trans)(2,4),
                    (*trans)(3,1), (*trans)(3,2), (*trans)(3,3), (*trans)(3,4)
                );


                /*fDevScnX
                cout<<"Trans Mat is : \n";
                printMatrix(trans);
                cout<<"Trans Inv Mat is : \n";
                printMatrix(trans_inv);
                */
                float tmpscn_x[10];
                float tmpscn_y[10];
                float tmpscn_z[10];
                float tmpmdl_x[10];
                float tmpmdl_y[10];
                float tmpmdl_z[10];
                cudaMemcpy(tmpscn_x, fDevScnPairX, 10*sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(tmpscn_y, fDevScnPairY, 10*sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(tmpscn_z, fDevScnPairZ, 10*sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(tmpmdl_x, fDevMdlPairX, 10*sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(tmpmdl_y, fDevMdlPairY, 10*sizeof(float), cudaMemcpyDeviceToHost);
                cudaMemcpy(tmpmdl_z, fDevMdlPairZ, 10*sizeof(float), cudaMemcpyDeviceToHost);
}

Matrix CIcpGpuCuda::fillHomoMatrix(Matrix* R, double* dTranslation){
			//Fill result
    Matrix matrix(4,4);
    matrix(1,1) = (*R)(1,1);
    matrix(1,2) = (*R)(1,2);
    matrix(1,3) = (*R)(1,3);
    matrix(1,4) = dTranslation[0];

    matrix(2,1) = (*R)(2,1);
    matrix(2,2) = (*R)(2,2);
    matrix(2,3) = (*R)(2,3);
    matrix(2,4) = dTranslation[1];

    matrix(3,1) = (*R)(3,1);
    matrix(3,2) = (*R)(3,2);
    matrix(3,3) = (*R)(3,3);
    matrix(3,4) = dTranslation[2];

    matrix(4,1) = 0;
    matrix(4,2) = 0;
    matrix(4,3) = 0;
    matrix(4,4) = 1;
    return matrix;
}


void CIcpGpuCuda::computeCentroid(float* x, float* y, float* z, float*& center){
                     /*
                        cublasSasum function works only with absolute values.
                        So I created an array of ones and used a dot product
                        in order to simulate a simple sum function
                    */

                    center[0] = cublasSdot(unSizeData,x,1,ones,1);
                    center[0] /=unPairs;
                    center[1] = cublasSdot(unSizeData,y,1,ones,1);
                    center[1] /=unPairs;
                    center[2] = cublasSdot(unSizeData,z,1,ones,1);
                    center[2] /=unPairs;
}


Matrix CIcpGpuCuda::computeHMatrix(){
			Matrix H(3,3);
			H = 0.0;

			unsigned unSizeOfSec = 200000; // need to be tuned for best performance!// +++ Fill by gpu +++
			if(unSizeData<=unSizeOfSec){
				H(1,1) = (double)cublasSdot(unSizeData,fCenScnX,1,fCenModX,1);
				H(1,2) = (double)cublasSdot(unSizeData,fCenScnX,1,fCenModY,1);
				H(1,3) = (double)cublasSdot(unSizeData,fCenScnX,1,fCenModZ,1);
				H(2,1) = (double)cublasSdot(unSizeData,fCenScnY,1,fCenModX,1);
				H(2,2) = (double)cublasSdot(unSizeData,fCenScnY,1,fCenModY,1);
				H(2,3) = (double)cublasSdot(unSizeData,fCenScnY,1,fCenModZ,1);
				H(3,1) = (double)cublasSdot(unSizeData,fCenScnZ,1,fCenModX,1);
				H(3,2) = (double)cublasSdot(unSizeData,fCenScnZ,1,fCenModY,1);
				H(3,3) = (double)cublasSdot(unSizeData,fCenScnZ,1,fCenModZ,1);
			}
			else{
				unsigned unSections = (unsigned)(unSizeData/unSizeOfSec);//cout<<"unSections: "<<unSections<<endl;
				unsigned unStub = (unsigned)(unSizeData%unSizeOfSec);//cout<<"unStub: "<<unStub<<endl;
				for(unsigned i=0;i<unSections;i++){
					H(1,1) += (double)cublasSdot(unSizeOfSec,fCenScnX+i*unSizeOfSec,1,fCenModX+i*unSizeOfSec,1);
					H(1,2) += (double)cublasSdot(unSizeOfSec,fCenScnX+i*unSizeOfSec,1,fCenModY+i*unSizeOfSec,1);
					H(1,3) += (double)cublasSdot(unSizeOfSec,fCenScnX+i*unSizeOfSec,1,fCenModZ+i*unSizeOfSec,1);
					H(2,1) += (double)cublasSdot(unSizeOfSec,fCenScnY+i*unSizeOfSec,1,fCenModX+i*unSizeOfSec,1);
					H(2,2) += (double)cublasSdot(unSizeOfSec,fCenScnY+i*unSizeOfSec,1,fCenModY+i*unSizeOfSec,1);
					H(2,3) += (double)cublasSdot(unSizeOfSec,fCenScnY+i*unSizeOfSec,1,fCenModZ+i*unSizeOfSec,1);
					H(3,1) += (double)cublasSdot(unSizeOfSec,fCenScnZ+i*unSizeOfSec,1,fCenModX+i*unSizeOfSec,1);
					H(3,2) += (double)cublasSdot(unSizeOfSec,fCenScnZ+i*unSizeOfSec,1,fCenModY+i*unSizeOfSec,1);
					H(3,3) += (double)cublasSdot(unSizeOfSec,fCenScnZ+i*unSizeOfSec,1,fCenModZ+i*unSizeOfSec,1);
				}
				if(unStub){
					H(1,1) += (double)cublasSdot(unStub,fCenScnX+unSections*unSizeOfSec,1,fCenModX+unSections*unSizeOfSec,1);
					H(1,2) += (double)cublasSdot(unStub,fCenScnX+unSections*unSizeOfSec,1,fCenModY+unSections*unSizeOfSec,1);
					H(1,3) += (double)cublasSdot(unStub,fCenScnX+unSections*unSizeOfSec,1,fCenModZ+unSections*unSizeOfSec,1);
					H(2,1) += (double)cublasSdot(unStub,fCenScnY+unSections*unSizeOfSec,1,fCenModX+unSections*unSizeOfSec,1);
					H(2,2) += (double)cublasSdot(unStub,fCenScnY+unSections*unSizeOfSec,1,fCenModY+unSections*unSizeOfSec,1);
					H(2,3) += (double)cublasSdot(unStub,fCenScnY+unSections*unSizeOfSec,1,fCenModZ+unSections*unSizeOfSec,1);
					H(3,1) += (double)cublasSdot(unStub,fCenScnZ+unSections*unSizeOfSec,1,fCenModX+unSections*unSizeOfSec,1);
					H(3,2) += (double)cublasSdot(unStub,fCenScnZ+unSections*unSizeOfSec,1,fCenModY+unSections*unSizeOfSec,1);
					H(3,3) += (double)cublasSdot(unStub,fCenScnZ+unSections*unSizeOfSec,1,fCenModZ+unSections*unSizeOfSec,1);
				}
			}
			getCublasErr();
                        return H;

}

void CIcpGpuCuda::printMatrix(Matrix* mat){
    for(int i = 1 ; i < 5 ; ++i)
        cout<<(*mat)(i,1)<< " "<<(*mat)(i,2)<< " "<<(*mat)(i,3)<<
            " "<<(*mat)(i,4)<< endl;
}
