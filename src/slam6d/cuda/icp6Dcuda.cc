#include <dlfcn.h>

using namespace std;

#include "slam6d/cuda/icp6Dcuda.h"
#include "slam6d/cuda/CIcpGpuCuda.cuh"
#include "newmat/newmat.h"


void icp6Dcuda::initGPUicp(float max_rad, float min_rad, int iter, int max_iter, 
					  int max_proctime, float max_dev, const double trans[], const double trans_inv[])
{
  if (max_scnSize < Scan::max_points_red_size) {
    if (icp != 0) {
	 delete icp;
	 icp = 0;
    }
  }
  if (icp == 0) {
    max_scnSize = Scan::max_points_red_size;
    icp = new CIcpGpuCuda((unsigned)10, ceil((float)Scan::max_points_red_size/10.f) ,
					 (unsigned)max_num_iterations);
    icp->setMaxProcTime((double) max_proctime);
    icp->setMaxDeviation((double) max_dev);
    icp->setSearchRadius((float) max_rad, (float) min_rad, (unsigned)max_num_iterations);
  }
}

icp6Dcuda::icp6Dcuda(icp6Dminimizer *my_icp6Dminimizer, double max_dist_match, 
				 int max_num_iterations, bool quiet, bool meta, int rnd, bool eP,
				 int anim, double epsilonICP, int nns_method, bool cuda_enabled)
: icp6D(my_icp6Dminimizer, max_dist_match, 
	   max_num_iterations, quiet, meta, rnd, eP,
	   anim, epsilonICP, nns_method, cuda_enabled)
{
  icp = 0;
  max_scnSize = 0;
}

/**
 * Matches a 3D Scan against a 3D Scan
 * @param PreviousScan The scan or metascan forming the model
 * @param CurrentScan The current scan thas is to be matched
 * @return The number of iterations done in this matching run
 */
int icp6Dcuda::match(Scan* PreviousScan, Scan* CurrentScan)
{
  int mdlSize = PreviousScan->get_points_red_size(), scnSize = CurrentScan->get_points_red_size();
  double** mdl;
  float** scn;
  const double *trans;
  double trans_inv[16];
  trans = PreviousScan->getDAlign();
  M4inv(trans, trans_inv);

  initGPUicp(sqrt(max_dist_match2), sqrt(max_dist_match2), max_num_iterations,
		   max_num_iterations, INT_MAX, epsilonICP, trans, trans_inv);
  
  h_idata = icp->getModelPointer();
  fHstScn = icp->getScenePointer();
  icp->setTrans_Trans_inv(trans, trans_inv);
  icp->setSize(10, ceil((float)max(mdlSize, scnSize)/10.0f));

  double **mod_dat = PreviousScan->get_org_points_red();
  double *const*scn_dat = CurrentScan->get_points_red();
  mdl = h_idata;
  scn = fHstScn;
  cout << "model point cloud size is " << mdlSize << "\n";
  for (unsigned int i = 0; i < mdlSize; ++i) {
    mdl[i][0] = (float)mod_dat[i][0];
    mdl[i][1] = (float)mod_dat[i][1];
    mdl[i][2] = (float)mod_dat[i][2];
  }
  cout << "scene point cloud size is " << scnSize << "\n";
  for (unsigned int i = 0; i < scnSize; ++i) {
    scn[0][i] = (float)scn_dat[i][0];
    scn[1][i] = (float)scn_dat[i][1];
    scn[2][i] = (float)scn_dat[i][2];
  }

  icp->setTreePointer(const_cast<ANNkd_tree *>(PreviousScan->getANNTree()));
  float result[4][4];

  // doICP
  icp->setPointClouds();
  icp->iteration();
  Matrix* M = icp->getMatrix();

  for (int i = 0 ; i < 4 ; ++i){
    for(int j = 0 ; j < 4 ; ++j) {
	 result[i][j] = (*M)(i+1,j+1);
    }	
  }
  
  if(anim > 0){
    Matrix** mats = icp->getMatrices();
    for(int ci = 0 ; ci < max_num_iterations ; ci += anim){
	 Matrix* cur_mat = mats[ci];
	 double xf[16];
	 for(int i = 1; i < 5 ; ++i)
	   for(int j = 1; j < 5; ++j)
		xf[(i-1)+(j-1)*4] = (*cur_mat)(i,j);
	 CurrentScan->transform(xf, Scan::ICP, 0);
    }
  }
  else {
    double alignxf[16];
    for (int i = 0; i < 4; ++i)
	 for (int j = 0; j < 4; ++j){
	   alignxf[i + j * 4] = result[i][j];
	 }
     CurrentScan->transform(alignxf, Scan::ICP, 0); // write end pose
  }

  return EXIT_SUCCESS;
}

