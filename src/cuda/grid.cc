#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include "cuda/grid.h"
#include "cuda/grid_kernel.h"
#include "cuda/tools.h"
#include <cuda_runtime.h>
#include "slam6d/globals.icc"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include <cfloat>	//DBL_MAX



void FindBoundingBox(double *d_xyz, double *bbox, unsigned int size)
{
	// X       Y       Z
	bbox[0]=bbox[2]=bbox[4]=DBL_MAX;	//MIN
	bbox[1]=bbox[3]=bbox[5]=-DBL_MAX;	//MAX

	for(unsigned int i=0;i<size;i++)
	{
		if(d_xyz[3*i+0]<bbox[0])
			bbox[0]=d_xyz[3*i+0];
		if(d_xyz[3*i+0]>bbox[1])
			bbox[1]=d_xyz[3*i+0];

		if(d_xyz[3*i+1]<bbox[2])
			bbox[2]=d_xyz[3*i+1];
		if(d_xyz[3*i+1]>bbox[3])
			bbox[3]=d_xyz[3*i+1];

		if(d_xyz[3*i+2]<bbox[4])
			bbox[4]=d_xyz[3*i+2];
		if(d_xyz[3*i+2]>bbox[5])
			bbox[5]=d_xyz[3*i+2];
	}
}


/**
 * Constructor
 *
 */
CuGrid::CuGrid(int cuda_device)
{
	device=cuda_device;
	SetCudaDevice(device);

	d_m_xyz=0;
	d_d_xyz=0;
	msize=dsize=0;
}
CuGrid::~CuGrid()
{
	SetCudaDevice(device);
	cudaFree(d_m_xyz);
	cudaFree(d_d_xyz);
}
/**
 * SetM
 *
 * Copy the model data
 *
 * @param m_xyz array of xyz values (size = 3*numpoints)
 * @param numpoints number of points (!)
 */
void CuGrid::SetM(double *m_xyz, int numpoints)
{
	SetCudaDevice(device);
	msize=numpoints;

	cudaFree(d_m_xyz);

	cudaMallocManaged((void**)&d_m_xyz, 3*numpoints*sizeof(double),cudaMemAttachGlobal);
    cudaDeviceSynchronize();
	memcpy(d_m_xyz,m_xyz, 3*numpoints*sizeof(double));
}
/**
 * SetD
 *
 * Copy the dataset data
 *
 * @param d_xyz array of xyz values (size = 3*numpoints)
 * @param numpoints number of points (!)
 */
void CuGrid::SetD(double *d_xyz, int numpoints)
{
	SetCudaDevice(device);
	dsize=numpoints;

	cudaFree(d_d_xyz);

	cudaMallocManaged((void**)&d_d_xyz, 3*numpoints*sizeof(double),cudaMemAttachGlobal);
	cudaDeviceSynchronize();

	memcpy(d_d_xyz,d_xyz, 3*numpoints*sizeof(double));

	double *bbox_env;
	cudaMallocManaged((void**)&bbox_env,sizeof(double)*6,cudaMemAttachGlobal);
    cudaDeviceSynchronize();

	FindBoundingBox(d_d_xyz, bbox_env, dsize);

	double scale_inv=fabs(bbox_env[0]);
	for(int i=1;i<6;++i)
	{
		if(scale_inv<fabs(bbox_env[i]))
		{
			scale_inv=fabs(bbox_env[i]);
		}
	}
	params.scale=0.95/scale_inv;

	//Scale environment;
	//TODO: Use CudaTransformScan here!
	//for (int i=0;i<dsize*3;++i)
	//{
	//	d_d_xyz[i]*=params.scale;
	//}
	//Create scale matrix
	double *mat_scale;
	cudaMallocManaged((void**)&mat_scale,sizeof(double)*16,cudaMemAttachGlobal);

	for(int i=0;i<16;++i)
	{
		mat_scale[i]=0;
	}
	mat_scale[0]=mat_scale[5]=mat_scale[10]=mat_scale[15]=params.scale;
    cudaDeviceSynchronize();
	cudaTransformScan(d_d_xyz,mat_scale,dsize);
	cudaFree(mat_scale);
}



std::vector<int> CuGrid::fixedRangeSearch()
{
	std::vector<int> output;

	if(!msize)
	{
		printf("Model M is empty!\n");
		return output;
	}
	if(!dsize)
	{
		printf("Model D is empty!\n");
		return output;
	}

	SetCudaDevice(device);


	//NNS
	unsigned int *index_m;
	unsigned int *index_d;
	unsigned int *_d_keysReference;
	int *_d_table_of_buckets;
	int *d_num_points_in_bucket;
	int *_d_NN;
	double *_d_temp_distances;

	CheckCudaError();

	cudaMallocManaged((void**)&index_m,msize*sizeof(unsigned int),cudaMemAttachGlobal);
	cudaMallocManaged((void**)&index_d,dsize*sizeof(unsigned int),cudaMemAttachGlobal);
	cudaMallocManaged((void**)&_d_keysReference,msize*sizeof(unsigned int),cudaMemAttachGlobal);
	int sizeBuckets =  params.buckets()*params.buckets()*params.buckets()*sizeof(int);
	cudaMallocManaged((void**)&_d_table_of_buckets,sizeBuckets,cudaMemAttachGlobal);
	cudaMallocManaged((void**)&d_num_points_in_bucket,sizeBuckets,cudaMemAttachGlobal);

	cudaMallocManaged((void**)&_d_NN,dsize*sizeof(int),cudaMemAttachGlobal);
	cudaMallocManaged((void**)&_d_temp_distances,dsize*sizeof(double),cudaMemAttachGlobal);

	CheckCudaError();

	cudaFindNN(d_m_xyz,d_d_xyz,index_m,
					index_d,
					_d_keysReference,
					_d_table_of_buckets,
					d_num_points_in_bucket,
					_d_NN,
					 msize,
					 dsize,
					 params.buckets(),
					_d_temp_distances,
					params.max_dist);

	cudaDeviceSynchronize();

	//Fill output
	output.resize(dsize);
	memcpy(output.data(),_d_NN,dsize*sizeof(int));


	cudaFree(index_m);
	cudaFree(index_d);
	cudaFree(_d_keysReference);
	cudaFree(_d_table_of_buckets);
	cudaFree(d_num_points_in_bucket);
	cudaFree(_d_NN);
	cudaFree(_d_temp_distances);




	return output;
}

void CuGrid::SetRadius(double radius)
{
	//printf("scale=%lf   MIN_dist = %lf   MAX_dist = %lf\n",params.scale,(2/512.0)/params.scale,(2/16.0)/params.scale);
	//Choose the number of buckets
	params.buckets(16);
	for(int i=16;i<=512;i*=2)
	{
		//printf("buckets= %d    -> max distance = %lf m (%lf)\n",i,(2.0/i)/params.scale,(2.0/i));
		if(radius>=(2.0/i)/params.scale)
			break;
		params.buckets(i);
	}

	params.max_dist=radius*params.scale;

}
void CuGrid::TransformM(double *mat)
{
	double *d_mat;
	cudaMallocManaged((void**)&d_mat,16*sizeof(double),cudaMemAttachGlobal);
	cudaDeviceSynchronize();
	memcpy(d_mat,mat,16*sizeof(double));
	cudaTransformScan(d_m_xyz,d_mat,msize);
}
double* CuGrid::GetM()
{
	return d_m_xyz;
}
double* CuGrid::GetD()
{
	return d_d_xyz;
}
unsigned int CuGrid::GetMsize()
{
	return msize;
}
unsigned int CuGrid::GetDsize()
{
	return dsize;
}
