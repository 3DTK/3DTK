/* 
 * File:   icp_gpu_cuda.h
 * Author: shams
 *
 * Created on April 17, 2010, 6:48 PM
 */

#ifndef _ICP6DCUDA_H__
#define _ICP6DCUDA_H__

#include <dlfcn.h>

#include "CIcpGpuCuda.cuh"
#include "icp6D.h"

class icp6Dcuda : public icp6D {
private:
    CIcpGpuCuda* icp;
    double** h_idata;
    float** fHstScn;

     float max_radius; // 0.3 */
     float min_radius; //0.05 */
     int iteration; 
     int max_iteration;  // 120 Must be greater than 5 */
     int max_proctime; 
     float max_deviation; 
	int width;
	int height;

// Default resolution for Slam6D software

public:
  icp6Dcuda(icp6Dminimizer *my_icp6Dminimizer,
		  double max_dist_match = 25.0,
		  int max_num_iterations = 50,
		  bool quiet = false,
		  bool meta = false,
		  int rnd = 1,
		  bool eP = true,
		  int anim = -1,
		  double epsilonICP = 0.0000001,
		  bool use_cache = false);
  
  /**
   * Destructor (empty, but needed, because virtual)
   */
  virtual ~icp6Dcuda() {};

  int match(Scan* PreviousScan, Scan* CurrentScan);

  void initGPUicp(int width, int height, float max_rad, float min_rad, int iter, int max_iter, 
			   int max_proctime, float max_dev, const double trans[], const double trans_inv[]);
  void cleanup();

};


#endif	/* _ICP_GPU_CUDA_H */

