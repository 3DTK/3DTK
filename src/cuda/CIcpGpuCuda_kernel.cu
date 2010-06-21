/** @file
 *  @brief GPU kernel operation
 *  @author Deyuan Qii, University of Applied Sciences Bonn-Rhein-Sieg, Sankt Augustin, Germany.
 *                      Fraunhofer IAIS, Sankt Augustin, Germany.
 */


#define nns_kernel(QSelector)	__global__ void nns_priority##QSelector(\
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
		unsigned unWidth){\
\
/*/////////////*/\
/* preparation */\
/*/////////////*/\
	const unsigned unSizeData = unSize;\
	const int nWidth = (int)unWidth;\
	const unsigned tid = blockIdx.x*blockDim.x + threadIdx.x;\
	const unsigned unQSize = (unsigned)QSIZE##QSelector;\
	const float fRadius = fSearchRadius*fSearchRadius;\
\
/*////////*/\
/* kernel */\
/*////////*/\
	if(tid<unSizeData){\
\
	float fQ[3] = {fDevScnX[tid], fDevScnY[tid], fDevScnZ[tid]}; 		/* extract query point to register*/\
	float fM[3] = {0.0f, 0.0f, 0.0f};\
\
	unsigned Q_Idx[unQSize];\
	unsigned* unQ_Idx = &Q_Idx[0];\
	float Q_Dist[unQSize];\
	float* fQ_Dist = &Q_Dist[0];\
	Q q(unQ_Idx, fQ_Dist, unQSize);\
	float nn_dist = HUGE; 												/* initial distance*/\
	unsigned node = 0;\
	unsigned* u = &node;\
	float fDistance = 0.0;\
	float* rd = &fDistance; 											/* distance to rectangle*/\
	q.Insert(1, 0.0); 													/* start with root of tree*/\
\
	unsigned unNode = 0;\
	unsigned unNodeIdx = 0;\
	float fRd = 0.0f;\
	unsigned cd = 0;\
	float old_off, new_rd;\
	float new_off = 0.0f;\
	unsigned unPointIdx = 0;\
	int nPointIdxX = 0;\
	int nPointIdxY = 0;\
	float4 f4Mdl;\
	float fM_temp[3];\
	float fDist_temp = 0.0f;\
\
	while (q.notEmpty()) { 												/* repeat until queue is empty*/\
		q.Extr_Min(u, rd); 												/* closest node to query point*/\
		unNode = *u;													/* extract data to register*/\
		unNodeIdx = unNode-1;\
		fRd = *rd;\
		if (fRd >= nn_dist)						 						/* further from nearest so far*/\
			break;\
		while (!bDevIsLeaf[unNodeIdx]) { 								/* descend until leaf found*/\
			cd = unDevAxis[unNodeIdx]; 									/* cutting dimension*/\
			new_off = fQ[cd] - fDevSplit[unNodeIdx]; 					/* offset to further child*/\
			if (new_off < 0) { 											/* q is below cutting plane*/\
				old_off = fQ[cd] - fDevLoBound[unNodeIdx]; 				/* compute offset*/\
				if (old_off > 0) 										/* overlaps interval*/\
					old_off = 0;\
				new_rd = fRd - old_off*old_off 							/* distance to further child*/\
						+ new_off*new_off;\
				q.Insert(unNode*2+1, new_rd); 							/* enqueue hi_child for later*/\
				unNode *= 2; 											/* visit lo_child next*/\
				unNodeIdx = unNode-1;\
			}\
			else { 														/* q is above cutting plane*/\
				old_off = fQ[cd] - fDevHiBound[unNodeIdx];\
				if(old_off < 0)\
					old_off = 0;\
				new_rd = fRd - old_off*old_off 							/* distance to further child*/\
						+ new_off*new_off;\
				q.Insert(unNode*2, new_rd);\
\
				unNode = 2*unNode + 1;\
				unNodeIdx = unNode-1;\
			}\
		}\
		unPointIdx = unDevIdx[unNodeIdx];\
		nPointIdxX=(int)unPointIdx%nWidth;								/*access via texture*/\
		nPointIdxY=(int)unPointIdx/nWidth;\
		f4Mdl=tex2D(refTex,(float)nPointIdxX,(float)nPointIdxY);\
		fM_temp[0]=f4Mdl.x;\
		fM_temp[1]=f4Mdl.y;\
		fM_temp[2]=f4Mdl.z;\
		fDist_temp = DIST(fM_temp,fQ);\
		if(fDist_temp<nn_dist){\
			nn_dist = fDist_temp;\
			fM[0]=fM_temp[0];\
			fM[1]=fM_temp[1];\
			fM[2]=fM_temp[2];\
		}\
	}\
\
\
	if(nn_dist>fRadius){												/*if non-pair*/\
		unMask[tid]=0;\
		fDevMdlPairX[tid]=0.0f;	fDevMdlPairY[tid]=0.0f;	fDevMdlPairZ[tid]=0.0f;\
		fDevScnPairX[tid]=0.0f;	fDevScnPairY[tid]=0.0f;	fDevScnPairZ[tid]=0.0f;\
	}\
	else{\
		unMask[tid]=1;\
		fDist[tid] = nn_dist;			/*return nearest distance for deviation calculation*/\
		fDevMdlPairX[tid]=fM[0];	fDevMdlPairY[tid]=fM[1];	fDevMdlPairZ[tid]=fM[2];\
		fDevScnPairX[tid]=fQ[0];	fDevScnPairY[tid]=fQ[1];	fDevScnPairZ[tid]=fQ[2];\
	}\
	}\
}

nns_kernel(1)
nns_kernel(2)
nns_kernel(3)
nns_kernel(4)
nns_kernel(5)
nns_kernel(6)

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
		float* fCenteredScnZ){
	const unsigned tid = blockIdx.x*blockDim.x + threadIdx.x;

	if(unMask[tid]){
		fCenteredModX[tid] = fDevMdlPairX[tid] - fcm0;
		fCenteredModY[tid] = fDevMdlPairY[tid] - fcm1; 
		fCenteredModZ[tid] = fDevMdlPairZ[tid] - fcm2; 
		fCenteredScnX[tid] = fDevScnPairX[tid] - fcs0;
		fCenteredScnY[tid] = fDevScnPairY[tid] - fcs1; 
		fCenteredScnZ[tid] = fDevScnPairZ[tid] - fcs2; 
	}else{
		fCenteredModX[tid] = 0.0f;
		fCenteredModY[tid] = 0.0f; 
		fCenteredModZ[tid] = 0.0f; 
		fCenteredScnX[tid] = 0.0f;
		fCenteredScnY[tid] = 0.0f; 
		fCenteredScnZ[tid] = 0.0f; 
	}
}


__global__ void transformation(float* fDevScnX,
		float* fDevScnY,
		float* fDevScnZ,
		float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23){
	const unsigned tid = blockIdx.x*blockDim.x + threadIdx.x;
	double adTmp[3];
	adTmp[0] = fDevScnX[tid] * m00 +
			   fDevScnY[tid] * m01 +
			   fDevScnZ[tid] * m02 + m03;
	adTmp[1] = fDevScnX[tid] * m10 +
			   fDevScnY[tid] * m11 +
			   fDevScnZ[tid] * m12 + m13;
	adTmp[2] = fDevScnX[tid] * m20 +
			   fDevScnY[tid] * m21 +
			   fDevScnZ[tid] * m22 + m23;
	fDevScnX[tid] = adTmp[0];
	fDevScnY[tid] = adTmp[1];
	fDevScnZ[tid] = adTmp[2];		
}
