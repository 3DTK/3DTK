#include <cuda_runtime.h>
#include "cuda/grid_kernel.h"
#include <cstdio>
#include <string>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

#include <cfloat>

#define CUDA_CHECK_ERROR(); {cudaError_t error=cudaGetLastError(); if(error){printf("CUDA error: %d (%s) in %s line %d\n",error,cudaGetErrorString(error),__FILE__,__LINE__);exit(-1);}}


__global__ void TransformPoints(double *outx, double *outy, double *outz, double m[16], double *x, double *y, double *z,int size)
{
	int i=threadIdx.x+blockIdx.x*1024+blockIdx.y*1024*1024;


	if(i<size)
	{
		double xx=x[i];
		double yy=y[i];
		double zz=z[i];

		outx[i]=m[0]*xx+m[4]*yy+m[8]*zz+m[12];
		outy[i]=m[1]*xx+m[5]*yy+m[9]*zz+m[13];
		outz[i]=m[2]*xx+m[6]*yy+m[10]*zz+m[14];
	}
}

__global__ void TransformPoints(double *outx, double *outy, double *outz, double m[16], int size)
{
	int i=threadIdx.x+blockIdx.x*1024+blockIdx.y*1024*1024;


	if(i<size)
	{
		double xx=outx[i];
		double yy=outy[i];
		double zz=outz[i];

		outx[i]=m[0]*xx+m[4]*yy+m[8]*zz+m[12];
		outy[i]=m[1]*xx+m[5]*yy+m[9]*zz+m[13];
		outz[i]=m[2]*xx+m[6]*yy+m[10]*zz+m[14];
	}
}
__global__ void TransformPoints(double *outxyz, double m[16], double *xyz,int size)
{
	int i=threadIdx.x+blockIdx.x*1024+blockIdx.y*1024*1024;


	if(i<size)
	{
		double xx=xyz[3*i+0];
		double yy=xyz[3*i+1];
		double zz=xyz[3*i+2];

		outxyz[3*i+0]=m[0]*xx+m[4]*yy+m[8]*zz+m[12];
		outxyz[3*i+1]=m[1]*xx+m[5]*yy+m[9]*zz+m[13];
		outxyz[3*i+2]=m[2]*xx+m[6]*yy+m[10]*zz+m[14];
	}
}

__global__ void TransformPoints(double *outxyz, double m[16], int size)
{
	int i=threadIdx.x+blockIdx.x*1024+blockIdx.y*1024*1024;


	if(i<size)
	{
		double xx=outxyz[3*i+0];
		double yy=outxyz[3*i+1];
		double zz=outxyz[3*i+2];

		outxyz[3*i+0]=m[0]*xx+m[4]*yy+m[8]*zz+m[12];
		outxyz[3*i+1]=m[1]*xx+m[5]*yy+m[9]*zz+m[13];
		outxyz[3*i+2]=m[2]*xx+m[6]*yy+m[10]*zz+m[14];
	}
}


void cudaTransformScan(double *destx, double *desty, double *destz, double *mat, double *srcx, double *srcy, double *srcz, unsigned int size)
{
	int by=1+int(size/(1024*1024.0));
	dim3 blocks(1024,by),threads(1024);
	//printf("TransformPoints<<<(%d,%d,%d),(%d,%d,%d)>>>\n",blocks.x,blocks.y,blocks.z,threads.x,threads.y,threads.z);
	
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
	TransformPoints<<<blocks,threads>>>(destx,desty,destz,mat,srcx,srcy,srcz,size);
	CUDA_CHECK_ERROR();
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
}

void cudaTransformScan(double *destx, double *desty, double *destz, double *mat, unsigned int size)
{
	int by=1+int(size/(1024*1024.0));
	dim3 blocks(1024,by),threads(1024);
	
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
	TransformPoints<<<blocks,threads>>>(destx,desty,destz,mat,size);
	CUDA_CHECK_ERROR();
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
}
void cudaTransformScan(double *destxyz, double *mat, double *srcxyz, unsigned int size)
{
	int by=1+int(size/(1024*1024.0));
	dim3 blocks(1024,by),threads(1024);
	
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
	TransformPoints<<<blocks,threads>>>(destxyz,mat,srcxyz,size);
	CUDA_CHECK_ERROR();
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
}
void cudaTransformScan(double *destxyz, double *mat, unsigned int size)
{
	int by=1+int(size/(1024*1024.0));
	dim3 blocks(1024,by),threads(1024);
	
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
	TransformPoints<<<blocks,threads>>>(destxyz,mat,size);
	CUDA_CHECK_ERROR();
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
}

void getKernelDim(dim3 &_block, dim3 &_thread, int num_points)
{
	_block.x = 1024;	_block.y = 1; _block.z = ceil(num_points/double(1024*1024.0));
	_thread.x = 1024; _thread.y = 1; _thread.z = 1;

}
/*
void getKernelDimBuckets(dim3 &_block, dim3 &_thread, int num_buckets)
{
	_block.x = num_buckets;  _block.y = num_buckets; _block.z = 1;
	_thread.x = num_buckets; _thread.y = 1; _thread.z = 1;
}

*/

__device__ int getThreadId(int bx, int by, int bz, int tx, int ty, int tz)
{
	return tx + bx*1024 + bz*1024*1024;
}

__global__ void
kernel_ComputeIndexes(unsigned int *_d_index, double *_x, double *_y, double *_z, 
					 int _amountOfPoints, int num_buckets)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);

	if(index < _amountOfPoints)
	{
		double x = _x[index];
		double y = _y[index];
		double z = _z[index];

		double border;
		int curr_ind;
		double cur_two_power_x;

		int upto, diff;

		if(num_buckets == 1024){upto = 10; diff = 1073741823;}
		if(num_buckets == 512){upto = 9; diff   = 134217727;}
		if(num_buckets == 256){upto = 8; diff   = 16777215;}
		if(num_buckets == 128){upto = 7; diff   = 2097151;}
		if(num_buckets == 64){upto = 6; diff    = 262143;}
		if(num_buckets == 32){upto = 5; diff    = 32767;}
		if(num_buckets == 16){upto = 4; diff    = 4095;}
		///////////////
		border = 0;
		curr_ind = 0;
		cur_two_power_x = 0;
		//__syncthreads();
	
		cur_two_power_x = 1.0f;
		for(int kk=0;kk<upto;kk++)
		{
		cur_two_power_x = cur_two_power_x / 2.0f;
			if(x <= border)
			{	
			curr_ind = 2 * curr_ind + 1; 
			border = border - cur_two_power_x;
			}else
			{
			curr_ind = 2 * curr_ind + 2;
			border = border + cur_two_power_x;	
			};
		}
			
		border = 0.0f;
		cur_two_power_x = 1.0f;
		for(int kk=0;kk<upto;kk++)
		{
			cur_two_power_x = cur_two_power_x / 2.0f;	
			if(y <= border)
			{	
			curr_ind = 2 * curr_ind + 1; 
			border = border - cur_two_power_x;
			}else
			{
			curr_ind = 2 * curr_ind + 2;
			border = border + cur_two_power_x;	
			};
		}
	
	
		border = 0.0f;
		cur_two_power_x = 1.0f;
		for(int kk=0;kk<upto;kk++)
		{
			cur_two_power_x = cur_two_power_x / 2.0f;	
			if(z <= border)
			{	
			curr_ind = 2 * curr_ind + 1; 
			border = border - cur_two_power_x;
			}else
			{
			curr_ind = 2 * curr_ind + 2;
			border = border + cur_two_power_x;	
			};
		}
	
		curr_ind = curr_ind-diff;//16777215;//SUM_INDEXES_IN_LEVELS;
		_d_index[index] = curr_ind;
		///////////////
	}
}

__global__ void
kernel_ComputeIndexes(unsigned int *_d_index, double *xyz, 
					 int _amountOfPoints, int num_buckets)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);

	if(index < _amountOfPoints)
	{
		double x = xyz[3*index+0];
		double y = xyz[3*index+1];
		double z = xyz[3*index+2];

		double border;
		int curr_ind;
		double cur_two_power_x;

		int upto, diff;

		if(num_buckets == 1024){upto = 10; diff = 1073741823;}
		if(num_buckets == 512){upto = 9; diff   = 134217727;}
		if(num_buckets == 256){upto = 8; diff   = 16777215;}
		if(num_buckets == 128){upto = 7; diff   = 2097151;}
		if(num_buckets == 64){upto = 6; diff    = 262143;}
		if(num_buckets == 32){upto = 5; diff    = 32767;}
		if(num_buckets == 16){upto = 4; diff    = 4095;}
		///////////////
		border = 0;
		curr_ind = 0;
		cur_two_power_x = 0;
		//__syncthreads();
	
		cur_two_power_x = 1.0f;
		for(int kk=0;kk<upto;kk++)
		{
		cur_two_power_x = cur_two_power_x / 2.0f;
			if(x <= border)
			{	
			curr_ind = 2 * curr_ind + 1; 
			border = border - cur_two_power_x;
			}else
			{
			curr_ind = 2 * curr_ind + 2;
			border = border + cur_two_power_x;	
			};
		}
			
		border = 0.0f;
		cur_two_power_x = 1.0f;
		for(int kk=0;kk<upto;kk++)
		{
			cur_two_power_x = cur_two_power_x / 2.0f;	
			if(y <= border)
			{	
			curr_ind = 2 * curr_ind + 1; 
			border = border - cur_two_power_x;
			}else
			{
			curr_ind = 2 * curr_ind + 2;
			border = border + cur_two_power_x;	
			};
		}
	
	
		border = 0.0f;
		cur_two_power_x = 1.0f;
		for(int kk=0;kk<upto;kk++)
		{
			cur_two_power_x = cur_two_power_x / 2.0f;	
			if(z <= border)
			{	
			curr_ind = 2 * curr_ind + 1; 
			border = border - cur_two_power_x;
			}else
			{
			curr_ind = 2 * curr_ind + 2;
			border = border + cur_two_power_x;	
			};
		}
	
		curr_ind = curr_ind-diff;
		_d_index[index] = curr_ind;
	}
}

__global__ void kernel_CountPointsInBuckets(unsigned int *_d_index, int *_d_table_of_buckets, int *d_num_points_in_bucket, int numpoints_m)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);
	int temp_index_of_bucket;
	int temp_previous_index_of_bucket;
	int amount_of_points = 0;

	if(index < numpoints_m)
	{
		if(index == 0)
		{
			amount_of_points = 1;
			temp_index_of_bucket = _d_index[0];
			for(int k = 1 ; k < numpoints_m; k++)
			{
				if(_d_index[index+k] == temp_index_of_bucket)
				{
					amount_of_points++;
				}
				else
					break;
			}
			_d_table_of_buckets[temp_index_of_bucket] = 0;
			d_num_points_in_bucket[temp_index_of_bucket] = amount_of_points;
		}
		else //index != 0
		{
			amount_of_points = 1;
			temp_index_of_bucket = _d_index[index];
			temp_previous_index_of_bucket = _d_index[index-1];
			if(temp_index_of_bucket != temp_previous_index_of_bucket)
			{
				_d_table_of_buckets[temp_index_of_bucket] = index;
				for(int k = 1 ; k < numpoints_m; k++)
				{
					if(index+k < numpoints_m)
					{
						if(_d_index[index+k] == temp_index_of_bucket)
						{
							amount_of_points++;
						}
						else
							break;
					}
					else
						break;
				}
			d_num_points_in_bucket[temp_index_of_bucket] = amount_of_points;	
			}
		}
		
	}
	
}

__global__ void kernel_FindNN(int *_d_NN, unsigned int *_d_index, int * d_num_points_in_bucket, 
					int * _d_table_of_buckets, unsigned int * _d_sorted_table_of_points, int numpoints_d, 
					int _thresholdOfPointsINNER, int _thresholdOfPointsOUTER, int num_buckets,
					double *_d_temp_distances, int _case, 
					double _additionalDistanceCheck,
					double *_xRef, double *_yRef, double *_zRef,
					double *_xToAlign, double *_yToAlign, double *_zToAlign, int repeat, int step)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);

	if(index < numpoints_d)
	{
		int temp_curr_ind = _d_index[index];
		int curr_ind = 0;
		switch(_case)
		{
			case 0:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets - 1;	break;}
			case 1:	{curr_ind = temp_curr_ind - num_buckets - 1;	break;	}
			case 2:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets - 1;	break;	}
			case 3:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - 1;	break;	}
			case 4:	{curr_ind = temp_curr_ind - 1;	break;	}
			case 5:	{curr_ind = temp_curr_ind + num_buckets * num_buckets - 1;break;	}
			case 6:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets - 1;	break;	}
			case 7:	{curr_ind = temp_curr_ind + num_buckets - 1;	break;	}
			case 8:	{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets - 1;	break;}
			case 9:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets;	break;	}
			case 10:{curr_ind = temp_curr_ind - num_buckets;break;	}
			case 11:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets;	break;}
			case 12:{curr_ind = temp_curr_ind - num_buckets * num_buckets;	break;	}
			case 13:{curr_ind = temp_curr_ind;	break;	}
			case 14:{curr_ind = temp_curr_ind + num_buckets * num_buckets;break;	}
			case 15:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets;	break;	}
			case 16:{curr_ind = temp_curr_ind + num_buckets;break;	}
			case 17:{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets;	break;	}
			case 18:{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets + 1;	break;	}
			case 19:{curr_ind = temp_curr_ind - num_buckets + 1;	break;	}
			case 20:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets + 1;	break;	}
			case 21:{curr_ind = temp_curr_ind - num_buckets * num_buckets + 1;break;	}
			case 22:{curr_ind = temp_curr_ind + 1;	break;	}
			case 23:{curr_ind = temp_curr_ind + num_buckets * num_buckets+1;	break;	}
			case 24:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets + 1;	break;	}
			case 25:{curr_ind = temp_curr_ind + num_buckets + 1;break;	}
			case 26:{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets + 1;	break;
			}
		}
		
		////////////////////////////////////d_compute_min_distances//////////////////////////////////////////////////////////////////////
		double d_min_distance = _d_temp_distances[index];
		double x_reference;
		double y_reference;
		double z_reference;

		double xRef = _xToAlign[index];
		double yRef = _yToAlign[index];
		double zRef = _zToAlign[index];

		//printf("A%d\n",index);

		int index_result = -1;
		int index_of_bucket = curr_ind;

		if(index_of_bucket<0)index_of_bucket = 0;
		if(index_of_bucket >= num_buckets*num_buckets*num_buckets) index_of_bucket = num_buckets*num_buckets*num_buckets-1;

		int number_of_points_in_bucket = d_num_points_in_bucket[index_of_bucket];//tu konflikt czytania
		int first_element_index = _d_table_of_buckets[index_of_bucket];				//tu konflikt czytania

		double temp_min_dist;

		if(_case == 13)
		{
			if(number_of_points_in_bucket > _thresholdOfPointsINNER)
				number_of_points_in_bucket = _thresholdOfPointsINNER;

		}else
		{
			if(number_of_points_in_bucket > _thresholdOfPointsOUTER)
				number_of_points_in_bucket = _thresholdOfPointsOUTER;
		}

		int endIndex = (repeat+1) * step;
		if(endIndex > number_of_points_in_bucket)
		{
			endIndex = number_of_points_in_bucket;
		}

		//printf("B%d  index_of_bucket = %d endIndex=%d\n",index,index_of_bucket,endIndex);

		for(int ii = repeat * step; ii < endIndex; ii++)
		{
			int index_of_point_in_sorted_table_of_points = _d_sorted_table_of_points[first_element_index + ii];//tu konflikt czytania

			x_reference = _xRef[index_of_point_in_sorted_table_of_points];
			y_reference = _yRef[index_of_point_in_sorted_table_of_points];
			z_reference = _zRef[index_of_point_in_sorted_table_of_points];


			temp_min_dist = sqrtf((xRef - x_reference)*(xRef -x_reference) +  
								  (yRef - y_reference)*(yRef -y_reference) + 
								  (zRef - z_reference)*(zRef -z_reference)); 

			if(temp_min_dist < d_min_distance) 
			{
				d_min_distance = temp_min_dist;
				index_result = index_of_point_in_sorted_table_of_points;
			}
		}
		//printf("C%d\n",index);

		if(index_result != -1)
		{
			if(d_min_distance < _additionalDistanceCheck )
			{
				_d_NN[index] = index_result;
				_d_temp_distances[index] = d_min_distance;
				//__syncthreads();
			}
		}
	}
}
__global__ void kernel_FindNN_ALL(int *_d_NN, unsigned int *_d_index, int * d_num_points_in_bucket, 
					int * _d_table_of_buckets, unsigned int * _d_sorted_table_of_points, int numpoints_d, 
					int num_buckets,
					double *_d_temp_distances, int _case, 
					double _additionalDistanceCheck,
					double *_xRef, double *_yRef, double *_zRef,
					double *_xToAlign, double *_yToAlign, double *_zToAlign)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);

	if(index < numpoints_d)
	{
		int temp_curr_ind = _d_index[index];
		int curr_ind = 0;
		switch(_case)
		{
			case 0:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets - 1;	break;}
			case 1:	{curr_ind = temp_curr_ind - num_buckets - 1;	break;	}
			case 2:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets - 1;	break;	}
			case 3:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - 1;	break;	}
			case 4:	{curr_ind = temp_curr_ind - 1;	break;	}
			case 5:	{curr_ind = temp_curr_ind + num_buckets * num_buckets - 1;break;	}
			case 6:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets - 1;	break;	}
			case 7:	{curr_ind = temp_curr_ind + num_buckets - 1;	break;	}
			case 8:	{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets - 1;	break;}
			case 9:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets;	break;	}
			case 10:{curr_ind = temp_curr_ind - num_buckets;break;	}
			case 11:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets;	break;}
			case 12:{curr_ind = temp_curr_ind - num_buckets * num_buckets;	break;	}
			case 13:{curr_ind = temp_curr_ind;	break;	}
			case 14:{curr_ind = temp_curr_ind + num_buckets * num_buckets;break;	}
			case 15:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets;	break;	}
			case 16:{curr_ind = temp_curr_ind + num_buckets;break;	}
			case 17:{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets;	break;	}
			case 18:{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets + 1;	break;	}
			case 19:{curr_ind = temp_curr_ind - num_buckets + 1;	break;	}
			case 20:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets + 1;	break;	}
			case 21:{curr_ind = temp_curr_ind - num_buckets * num_buckets + 1;break;	}
			case 22:{curr_ind = temp_curr_ind + 1;	break;	}
			case 23:{curr_ind = temp_curr_ind + num_buckets * num_buckets+1;	break;	}
			case 24:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets + 1;	break;	}
			case 25:{curr_ind = temp_curr_ind + num_buckets + 1;break;	}
			case 26:{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets + 1;	break;
			}
		}
		
		////////////////////////////////////d_compute_min_distances//////////////////////////////////////////////////////////////////////
		double d_min_distance = _d_temp_distances[index];
		double x_reference;
		double y_reference;
		double z_reference;

		double xRef = _xToAlign[index];
		double yRef = _yToAlign[index];
		double zRef = _zToAlign[index];

		//printf("A%d\n",index);

		int index_result = -1;
		int index_of_bucket = curr_ind;

		if(index_of_bucket<0)index_of_bucket = 0;
		if(index_of_bucket >= num_buckets*num_buckets*num_buckets) index_of_bucket = num_buckets*num_buckets*num_buckets-1;

		int number_of_points_in_bucket = d_num_points_in_bucket[index_of_bucket];//tu konflikt czytania
		int first_element_index = _d_table_of_buckets[index_of_bucket];				//tu konflikt czytania

		double temp_min_dist;

		

		int endIndex = number_of_points_in_bucket;
		

		//printf("B%d  index_of_bucket = %d endIndex=%d\n",index,index_of_bucket,endIndex);

		for(int ii = 0; ii < endIndex; ii++)
		{
			int index_of_point_in_sorted_table_of_points = _d_sorted_table_of_points[first_element_index + ii];//tu konflikt czytania

			x_reference = _xRef[index_of_point_in_sorted_table_of_points];
			y_reference = _yRef[index_of_point_in_sorted_table_of_points];
			z_reference = _zRef[index_of_point_in_sorted_table_of_points];


			temp_min_dist = sqrtf((xRef - x_reference)*(xRef -x_reference) +  
								  (yRef - y_reference)*(yRef -y_reference) + 
								  (zRef - z_reference)*(zRef -z_reference)); 

			if(temp_min_dist < d_min_distance) 
			{
				d_min_distance = temp_min_dist;
				index_result = index_of_point_in_sorted_table_of_points;
			}
		}
		//printf("C%d\n",index);

		if(index_result != -1)
		{
			if(d_min_distance < _additionalDistanceCheck )
			{
				_d_NN[index] = index_result;
				_d_temp_distances[index] = d_min_distance;
				//__syncthreads();
			}
		}
	}
	
}
__global__ void kernel_FindNN_ALL(int *_d_NN, unsigned int *_d_index, int * d_num_points_in_bucket, 
					int * _d_table_of_buckets, unsigned int * _d_sorted_table_of_points, int numpoints_d, 
					int num_buckets,
					double *_d_temp_distances, int _case, 
					double _additionalDistanceCheck,
					double *mxyz,
					double *dxyz)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);

	if(index < numpoints_d)
	{
		int temp_curr_ind = _d_index[index];
		int curr_ind = 0;
		switch(_case)
		{
			case 0:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets - 1;	break;}
			case 1:	{curr_ind = temp_curr_ind - num_buckets - 1;	break;	}
			case 2:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets - 1;	break;	}
			case 3:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - 1;	break;	}
			case 4:	{curr_ind = temp_curr_ind - 1;	break;	}
			case 5:	{curr_ind = temp_curr_ind + num_buckets * num_buckets - 1;break;	}
			case 6:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets - 1;	break;	}
			case 7:	{curr_ind = temp_curr_ind + num_buckets - 1;	break;	}
			case 8:	{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets - 1;	break;}
			case 9:	{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets;	break;	}
			case 10:{curr_ind = temp_curr_ind - num_buckets;break;	}
			case 11:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets;	break;}
			case 12:{curr_ind = temp_curr_ind - num_buckets * num_buckets;	break;	}
			case 13:{curr_ind = temp_curr_ind;	break;	}
			case 14:{curr_ind = temp_curr_ind + num_buckets * num_buckets;break;	}
			case 15:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets;	break;	}
			case 16:{curr_ind = temp_curr_ind + num_buckets;break;	}
			case 17:{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets;	break;	}
			case 18:{curr_ind = temp_curr_ind - num_buckets * num_buckets - num_buckets + 1;	break;	}
			case 19:{curr_ind = temp_curr_ind - num_buckets + 1;	break;	}
			case 20:{curr_ind = temp_curr_ind + num_buckets * num_buckets - num_buckets + 1;	break;	}
			case 21:{curr_ind = temp_curr_ind - num_buckets * num_buckets + 1;break;	}
			case 22:{curr_ind = temp_curr_ind + 1;	break;	}
			case 23:{curr_ind = temp_curr_ind + num_buckets * num_buckets+1;	break;	}
			case 24:{curr_ind = temp_curr_ind - num_buckets * num_buckets + num_buckets + 1;	break;	}
			case 25:{curr_ind = temp_curr_ind + num_buckets + 1;break;	}
			case 26:{curr_ind = temp_curr_ind + num_buckets * num_buckets + num_buckets + 1;	break;
			}
		}
		
		////////////////////////////////////d_compute_min_distances//////////////////////////////////////////////////////////////////////
		double d_min_distance = _d_temp_distances[index];
		double x_reference;
		double y_reference;
		double z_reference;

		double xRef = dxyz[3*index+0];
		double yRef = dxyz[3*index+1];
		double zRef = dxyz[3*index+2];

		//printf("A%d\n",index);

		int index_result = -1;
		int index_of_bucket = curr_ind;

		if(index_of_bucket<0)index_of_bucket = 0;
		if(index_of_bucket >= num_buckets*num_buckets*num_buckets) index_of_bucket = num_buckets*num_buckets*num_buckets-1;

		int number_of_points_in_bucket = d_num_points_in_bucket[index_of_bucket];//tu konflikt czytania
		int first_element_index = _d_table_of_buckets[index_of_bucket];				//tu konflikt czytania

		double temp_min_dist;

		

		int endIndex = number_of_points_in_bucket;
		

		//printf("B%d  index_of_bucket = %d endIndex=%d\n",index,index_of_bucket,endIndex);

		for(int ii = 0; ii < endIndex; ii++)
		{
			int index_of_point_in_sorted_table_of_points = _d_sorted_table_of_points[first_element_index + ii];//tu konflikt czytania

			x_reference = mxyz[3*index_of_point_in_sorted_table_of_points+0];
			y_reference = mxyz[3*index_of_point_in_sorted_table_of_points+1];
			z_reference = mxyz[3*index_of_point_in_sorted_table_of_points+2];


			temp_min_dist = sqrtf((xRef - x_reference)*(xRef -x_reference) +  
								  (yRef - y_reference)*(yRef -y_reference) + 
								  (zRef - z_reference)*(zRef -z_reference)); 

			if(temp_min_dist < d_min_distance) 
			{
				d_min_distance = temp_min_dist;
				index_result = index_of_point_in_sorted_table_of_points;
			}
		}
		//printf("C%d\n",index);

		if(index_result != -1)
		{
			if(d_min_distance < _additionalDistanceCheck )
			{
				_d_NN[index] = index_result;
				_d_temp_distances[index] = d_min_distance;
				//__syncthreads();
			}
		}
	}
	
}
void cudaFindNN(double *d_mxyz, 
				double *d_dxyz,
				unsigned int *index_m, 
				unsigned int *index_d, 
				unsigned int *_d_keysReference, 
				int *_d_table_of_buckets, 
				int *d_num_points_in_bucket, 
				int *_d_NN,
				int numpoints_m, 
				int numpoints_d, 
				int _thresholdOfPointsINNER, 
				int _thresholdOfPointsOUTER, 
				int num_buckets, 
				double *_d_temp_distances, 
				double _additionalDistanceCheck)
{
	CUDA_CHECK_ERROR();
	size_t mfree,mtot;
	dim3 block_m, block_d;
	//dim3 blockToFill;
	dim3 thread_m, thread_d;
	//dim3 threadToFill;
	
	getKernelDim(block_m, thread_m , numpoints_m);
	getKernelDim(block_d, thread_d , numpoints_d);
	//getKernelDimBuckets(blockToFill, threadToFill, num_buckets);
	
	printf("block_m: %d %d %d   thread_m: %d %d %d\n",block_m.x,block_m.y,block_m.z,thread_m.x,thread_m.y,thread_m.z);
	printf("block_d: %d %d %d   thread_d: %d %d %d\n",block_d.x,block_d.y,block_d.z,thread_d.x,thread_d.y,thread_d.z);
	
	cudaMemGetInfo(&mfree,&mtot);
	printf("Now %lf MB is free from %lf MB total\n",mfree/(1024*1024.0),mtot/(1024*1024.0));
	
	CUDA_CHECK_ERROR();
	//Indexes for M
	kernel_ComputeIndexes<<<block_m, thread_m>>>(index_m, d_mxyz, numpoints_m, num_buckets);
	cudaDeviceSynchronize();
	cudaMemGetInfo(&mfree,&mtot);
	printf("Now %lf MB is free from %lf MB total\n",mfree/(1024*1024.0),mtot/(1024*1024.0));
	//kernel_initKey<<<block_m, thread_m>>>(_d_keysReference, numpoints_m);  //use sequence instead
	/*printf("--------sequence-----_d_keysReference-------\n");
	
	for(int i=0;i<numpoints_m;++i)
	{
		printf("_d_keysReference[%d] = %d\n",i,_d_keysReference[i]);
	}
	printf("---------kernel_ComputeIndexes-----------\n");
	
	for(int i=0;i<numpoints_m;++i)
	{
		printf("index_m[%d] = %d\n",i,index_m[i]);
	}printf("--------sequence-----_d_keysReference-------\n");
	
	for(int i=0;i<numpoints_m;++i)
	{
		printf("_d_keysReference[%d] = %d\n",i,_d_keysReference[i]);
	}
		*/
	CUDA_CHECK_ERROR();
	thrust::device_ptr <unsigned int> dev_ptrindex_m ( index_m );
	CUDA_CHECK_ERROR();
	thrust::device_ptr <unsigned int> dev_ptr_d_keysReference ( _d_keysReference );
	CUDA_CHECK_ERROR();
	/*printf("--------sequence-----_d_keysReference-------\n");
	
	for(int i=0;i<numpoints_m;++i)
	{
		printf("_d_keysReference[%d] = %d\n",i,_d_keysReference[i]);
	}*/
	thrust::sequence(dev_ptr_d_keysReference,dev_ptr_d_keysReference+numpoints_m);
	CUDA_CHECK_ERROR();
	
	cudaDeviceSynchronize();
	/*printf("--------sequence-----_d_keysReference-------\n");
	
	for(int i=0;i<numpoints_m;++i)
	{
		printf("_d_keysReference[%d] = %d\n",i,_d_keysReference[i]);
	}*/
	thrust::sort_by_key (dev_ptrindex_m , dev_ptrindex_m + numpoints_m, dev_ptr_d_keysReference );
	CUDA_CHECK_ERROR();
	cudaDeviceSynchronize();
	/*printf("----------sort_by_key----------\n");
	
	for(int i=0;i<numpoints_m;++i)
	{
		printf("index_m[%d] = %d\n",i,index_m[i]);
	}
	for(int i=0;i<numpoints_m;++i)
	{
		printf("_d_keysReference[%d] = %d\n",i,_d_keysReference[i]);
	}
	
	
	printf("\n\n\n\n");
	*/
	
	thrust::device_ptr <int> dev_ptr_d_table_of_buckets(_d_table_of_buckets);
	CUDA_CHECK_ERROR();
	thrust::device_ptr <int> dev_ptrd_num_points_in_bucket(d_num_points_in_bucket);
	CUDA_CHECK_ERROR();
	thrust::fill(dev_ptr_d_table_of_buckets,dev_ptr_d_table_of_buckets+num_buckets*num_buckets*num_buckets,-1);
	CUDA_CHECK_ERROR();
	thrust::fill(dev_ptrd_num_points_in_bucket,dev_ptrd_num_points_in_bucket+num_buckets*num_buckets*num_buckets,-1);
	
	//kernerl_Fill<<<blockToFill, threadToFill>>>(_d_table_of_buckets, num_buckets, -1);
	//kernerl_Fill<<<blockToFill, threadToFill>>>(d_num_points_in_bucket, num_buckets, -1);
	
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
	cudaMemGetInfo(&mfree,&mtot);
	printf("Now %lf MB is free from %lf MB total\n",mfree/(1024*1024.0),mtot/(1024*1024.0));
	/*
	printf("----------_d_table_of_buckets----------\n");
	
	for(int i=0;i<num_buckets*num_buckets*num_buckets;++i)
	{
		printf("_d_table_of_buckets[%d] = %d\n",i,_d_table_of_buckets[i]);
	}
	
	printf("----------d_num_points_in_bucket----------\n");
	
	for(int i=0;i<num_buckets*num_buckets*num_buckets;++i)
	{
		printf("d_num_points_in_bucket[%d] = %d\n",i,d_num_points_in_bucket[i]);
	}
	 */
	
	cudaDeviceSynchronize();
	kernel_CountPointsInBuckets<<<block_m, thread_m>>>(index_m, _d_table_of_buckets, d_num_points_in_bucket, numpoints_m);
	cudaDeviceSynchronize();
	CUDA_CHECK_ERROR();
		/*
	thrust::device_ptr <double> dev_ptr_d_temp_double_mem ( _d_temp_double_mem );
	thrust::device_ptr <double> dev_ptr_d_table_m_d ( _d_table_m_d );
	thrust::device_ptr <double> dev_ptr_d_table_C ( _d_table_C );
*/
	


	//Indexes for D
	kernel_ComputeIndexes<<<block_d, thread_d>>>(index_d, d_dxyz, numpoints_d, num_buckets);
	cudaDeviceSynchronize();
	
	//kernel_fill_NN_by_value<<<block_d, thread_d>>>(_d_NN, numpoints_d, -1);	
	//cudaDeviceSynchronize();
	thrust::device_ptr <int> dev_ptrd__d_NN(_d_NN);
	thrust::fill(dev_ptrd__d_NN,dev_ptrd__d_NN+numpoints_d,-1);
	
	CUDA_CHECK_ERROR();
	
	
	cudaMemGetInfo(&mfree,&mtot);
	printf("Now %lf MB is free from %lf MB total\n",mfree/(1024*1024.0),mtot/(1024*1024.0));
	
	//kernel_fill_temp_distances_by_value<<<block_d, thread_d>>>(_d_temp_distances, numpoints_d, 2.0f);	
	//cudaDeviceSynchronize();
	thrust::device_ptr <double> dev_ptrd__d_temp_distances(_d_temp_distances);
	thrust::fill(dev_ptrd__d_temp_distances,dev_ptrd__d_temp_distances+numpoints_d,2);


		CUDA_CHECK_ERROR();
/*
	int step = 10;
	int iter13 ;

	for(int kkk = 0; kkk < 27; kkk++)
	{
		if(kkk == 13)
		{
			iter13 = _thresholdOfPointsINNER/step + 1;
		}
		else
		{
			iter13 = _thresholdOfPointsOUTER/step + 1;
		}

		for(int kk = 0; kk < iter13; kk++)
		{
			kernel_FindNN<<<block_d, thread_d>>>(_d_NN, index_d, d_num_points_in_bucket, 
					_d_table_of_buckets, _d_keysReference, numpoints_d, 
					_thresholdOfPointsINNER, _thresholdOfPointsOUTER, num_buckets,
					_d_temp_distances, kkk, 
					_additionalDistanceCheck,
					d_mx, d_my, d_mz, 
					d_dx,d_dy, d_dz,kk,step);
			cudaDeviceSynchronize();
		CUDA_CHECK_ERROR();
		}
	}*/
		
		
	
	for(int kkk = 0; kkk < 27; kkk++)
	{

		kernel_FindNN_ALL<<<block_d, thread_d>>>(_d_NN, index_d, d_num_points_in_bucket, 
				_d_table_of_buckets, _d_keysReference, numpoints_d, 
				num_buckets,
				_d_temp_distances, kkk, 
				_additionalDistanceCheck,
				d_mxyz, 
				d_dxyz);
		cudaDeviceSynchronize();
		CUDA_CHECK_ERROR();
	}

	
	cudaMemGetInfo(&mfree,&mtot);
	printf("Now %lf MB is free from %lf MB total\n",mfree/(1024*1024.0),mtot/(1024*1024.0));

}

/*
__global__ void kernel_bbox(double *d_x, double *d_y, double *d_z, 
							double *bbox, unsigned int size)
{
	int index = getThreadId(blockIdx.x,blockIdx.y,blockIdx.z, threadIdx.x, threadIdx.y,threadIdx.z);
	
	//Initialize the bbox with max values
	if(index==0)
	{
		bbox[0]=DBL_MAX;	//Xmin
		bbox[1]=-DBL_MAX;	//Xmax
		
		bbox[2]=DBL_MAX;	//Ymin
		bbox[3]=-DBL_MAX;	//Ymax
		
		bbox[4]=DBL_MAX;	//Zmin
		bbox[5]=-DBL_MAX;	//Zmax
	}
	
	__syncthreads();
	
	if(index<size)
	{
		
	}
}*/


void FindBoundingBox(double *d_x, double *d_y, double *d_z, double *bbox, unsigned int size)
{
	/*
	dim3 blocks, threads;
	getKernelDim(blocks, threads , size);
	
	kernel_bbox<<<blocks,threads>>>(d_x,d_y,d_z,size);
	*/
	
	// X       Y       Z
	bbox[0]=bbox[2]=bbox[4]=DBL_MAX;	//MIN
	bbox[1]=bbox[3]=bbox[5]=-DBL_MAX;	//MAX

	for(unsigned int i=0;i<size;i++)
	{
		if(d_x[i]<bbox[0])
			bbox[0]=d_x[i];
		if(d_x[i]>bbox[1])
			bbox[1]=d_x[i];

		if(d_y[i]<bbox[2])
			bbox[2]=d_y[i];
		if(d_y[i]>bbox[3])
			bbox[3]=d_y[i];

		if(d_z[i]<bbox[4])
			bbox[4]=d_z[i];
		if(d_z[i]>bbox[5])
			bbox[5]=d_z[i];
	}		
}