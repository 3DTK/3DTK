#ifndef __CUDA_TOOLS_H__
#define __CUDA_TOOLS_H__

//#define CUDA_CHECK_ERROR(); {cudaError_t error=cudaGetLastError(); if(error){printf("CUDA error: %d (%s) in %s line %d\n",error,cudaGetErrorString(error),__FILE__,__LINE__);exit(-1);}}

void CheckCudaError();
void PrintCudaInfo();
int SetCudaDevice(int cuda_device);

#endif