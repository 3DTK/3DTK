#ifndef __CUDA_TOOLS_H__
#define __CUDA_TOOLS_H__

void CheckCudaError();
void PrintCudaInfo();
int SetCudaDevice(int cuda_device);
bool ValidCUDADevice(int device);

#endif