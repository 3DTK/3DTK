/** @file
 *  @brief GPU kernel operation
 *  @author Deyuan Qii, University of Applied Sciences Bonn-Rhein-Sieg, Sankt Augustin, Germany.
 *                      Fraunhofer IAIS, Sankt Augustin, Germany.
 */

#ifndef CICPGPUCUDA_KERNEL_CUH
#define CICPGPUCUDA_KERNEL_CUH

#include <cuda.h>		//cuda
#include <cutil.h>		//cuda utility
#include "cublas.h"		//cublas
#include "cudpp/cudpp.h"	//cudpp

#define NO_QSIZE	6	//number of different queue size
#define QSIZE1		1
#define QSIZE2		1
#define QSIZE3		1
#define QSIZE4		1
#define QSIZE5		1
#define QSIZE6		1
#define DIST(x,y)	((x[0]-y[0])*(x[0]-y[0])+(x[1]-y[1])*(x[1]-y[1])+(x[2]-y[2])*(x[2]-y[2]))
#define nns_kernel_h(QSelector)	__global__ void nns_priority##QSelector(\
		float* fDevScnX,\
		float* fDevScnY,\
		float* fDevScnZ,\
		float* fDist,\
		float* fDevSplit,\
		unsigned* unDevIdx,\
		unsigned* unDevAxis,\
		bool* bDevIsLeaf,\
		float* fDevLoBound,\
		float* fDevHiBound,\
		unsigned* unMask,\
		float* fDevMdlPairX,\
		float* fDevMdlPairY,\
		float* fDevMdlPairZ,\
		float* fDevScnPairX,\
		float* fDevScnPairY,\
		float* fDevScnPairZ,\
		float fSearchRadius,\
		unsigned unSize,\
		unsigned unWidth);

//global variable
texture<float4, 2, cudaReadModeElementType> refTex;
dim3 dimBlock;
dim3 dimGrid;
float4* f4Mdl;
cudaArray* cuArray;
cudaChannelFormatDesc cuDesc = cudaCreateChannelDesc<float4>();
CUDPPResult result;
CUDPPHandle compactplan;


nns_kernel_h(1)
nns_kernel_h(2)
nns_kernel_h(3)
nns_kernel_h(4)
nns_kernel_h(5)
nns_kernel_h(6)

__global__ void centralize(unsigned* unMask,
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
		float* fCenteredModX,
		float* fCenteredModY,
		float* fCenteredModZ,
		float* fCenteredScnX,
		float* fCenteredScnY,
		float* fCenteredScnZ);

__global__ void transformation(float* fDevScnX,
		float* fDevScnY,
		float* fDevScnZ,
		float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23);

extern "C"
void wrapper_nns_priority(//float* fDevModX, //
//		float* fDevModY, //
//		float* fDevModZ,//
		float* fDevScnX,//
		float* fDevScnY,//
		float* fDevScnZ,//
		float* fDist,
		float* fDevSplit, //
		unsigned* unDevIdx, //
		unsigned* unDevAxis, //
		bool* bDevIsLeaf, //
		float* fDevLoBound,//
		float* fDevHiBound,//
//		unsigned* unDevResult,
		unsigned* unMask,
		float* fDevMdlPairX,
		float* fDevMdlPairY,
		float* fDevMdlPairZ,
		float* fDevScnPairX,
		float* fDevScnPairY,
		float* fDevScnPairZ,
//		float* fDevMdlPairTempX,
//		float* fDevMdlPairTempY,
//		float* fDevMdlPairTempZ,
//		unsigned* temp,
		dim3 dimGrid,
		dim3 dimBlock,
		float fSearchRadius,
		unsigned unSize,
		unsigned unWidth,
		unsigned unQStep){

	//kernel selection
	switch(unQStep)
	{
		case	0:	nns_priority1<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);	break;
		case	1:	nns_priority2<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);	break;
		case	2:	nns_priority3<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);	break;
		case	3:	nns_priority4<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);	break;
		case	4:	nns_priority5<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);	break;
		case	5:	nns_priority6<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);	break;
		default:	nns_priority6<<<dimGrid, dimBlock>>>(/*fDevMdlX, fDevMdlY, fDevMdlZ,*/ fDevScnX, fDevScnY, fDevScnZ,
				fDist, fDevSplit, unDevIdx, unDevAxis, bDevIsLeaf, fDevLoBound, fDevHiBound,
				/*unDevResult,*/ unMask, fDevMdlPairX, fDevMdlPairY, fDevMdlPairZ, fDevScnPairX, fDevScnPairY, fDevScnPairZ,
				/*temp,*/ fSearchRadius, unSize, unWidth);
	}

}

extern "C"
void wrapper_centralize(unsigned* unMask,
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
		float* fCenteredModX,
		float* fCenteredModY,
		float* fCenteredModZ,
		float* fCenteredScnX,
		float* fCenteredScnY,
		float* fCenteredScnZ,
		dim3 dimGrid,
		dim3 dimBlock){
	centralize<<<dimGrid, dimBlock>>>(unMask,
			fDevMdlPairX,fDevMdlPairY,fDevMdlPairZ,fDevScnPairX,fDevScnPairY,fDevScnPairZ,
			fcm0,fcm1,fcm2,fcs0,fcs1,fcs2,
			fCenteredModX,fCenteredModY,fCenteredModZ,fCenteredScnX,fCenteredScnY,fCenteredScnZ);
}

extern "C"
void wrapper_transformation(float* fDevScnX,
		float* fDevScnY,
		float* fDevScnZ,
		float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23,
		dim3 dimGrid, dim3 dimBlock){
	transformation<<<dimGrid, dimBlock>>>(fDevScnX, fDevScnY, fDevScnZ,
			m00,	m01,	m02,	m03,
			m10,	m11,	m12,	m13,
			m20,	m21,	m22,	m23);
}

//An internal queue structure.
struct Q{
public:
	Q(unsigned* q_idx, float* q_dist, unsigned unMemSize, unsigned unSize = 0){
		_qhead_idx = q_idx;
		_qtail_idx = q_idx;
		_qhead_dist = q_dist;
		_qtail_dist = q_dist;
		_unMemSize = unMemSize;
		_unSize = unSize;
	}

	__device__ unsigned getSize(){
		return _unSize;
	}

	__device__ bool notEmpty(){
		if(_qhead_idx==_qtail_idx)	return false;
		else return true;
	}

	__device__ void Insert(unsigned unNode, float fDist){
		if(_unSize<_unMemSize){
			*_qtail_idx = unNode;
			*_qtail_dist = fDist;
			_qtail_idx++;
			_qtail_dist++;
			_unSize++;
		}
	}

	__device__ void Extr_Min(unsigned* idx, float* min){
		float fMin = HUGE;
		unsigned unIdx = 0;
		unsigned unI = 0;

		for(unsigned i=0;i<_unSize;i++){
			if(_qhead_dist[i]<fMin){
				fMin = _qhead_dist[i];
				unIdx = _qhead_idx[i];
				unI = i;
			}
		}

		_qtail_idx--;
		_qtail_dist--;
		if(_qtail_idx!=_qhead_idx+unI){
			_qhead_idx[unI] = *_qtail_idx;
			_qhead_dist[unI] = *_qtail_dist;
		}
		_unSize--;

		*min = fMin;
		*idx = unIdx;
	}

private:
	unsigned* _qhead_idx;
	unsigned* _qtail_idx;
	float* _qhead_dist;
	float* _qtail_dist;
	unsigned _unMemSize;
	unsigned _unSize;

};

#include "slam6d/cuda/CIcpGpuCuda_kernel.cu"

#endif
