#include <dlfcn.h>

using namespace std;

#include "icp6Dcuda.h"
#include "CIcpGpuCuda.cuh"
#include "newmat/newmat.h"


void icp6Dcuda::initGPUicp(int width, int height, float max_rad, float min_rad, int iter, int max_iter, 
					  int max_proctime, float max_dev, double trans[], double trans_inv[])
{
    icp = new CIcpGpuCuda(1, NULL, (unsigned) width, (unsigned) height, (unsigned) max_iter);
    icp->setMaxIteration((unsigned) max_iter);
    icp->setMaxProcTime((double) max_proctime);
    icp->setMaxDeviation((double) max_dev);
    icp->setSearchRadius((float) max_rad, (float) min_rad, (unsigned) iter);
    h_idata = icp->getModelPointer();
    fHstScn = icp->getScenePointer();
    icp->setTrans_Trans_inv(trans, trans_inv);
}

icp6Dcuda::icp6Dcuda(icp6Dminimizer *my_icp6Dminimizer, double max_dist_match, 
				 int max_num_iterations, bool quiet, bool meta, int rnd, bool eP,
				 int anim, double epsilonICP, bool use_cache)
: icp6D(my_icp6Dminimizer, max_dist_match, 
	   max_num_iterations, quiet, meta, rnd, eP,
	   anim, epsilonICP, use_cache)
{ }

void icp6Dcuda::cleanup()
{
  delete icp;
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
  double *trans;
  double trans_inv[16];
  trans = PreviousScan->getDAlign();
  M4inv(trans,trans_inv);

  initGPUicp(1, max(mdlSize, scnSize),
		   sqrt(max_dist_match2), sqrt(max_dist_match2), max_num_iterations,
		   max_num_iterations, INT_MAX, epsilonICP, trans, trans_inv);
  
  double **mod_dat = PreviousScan->get_org_points_red();
  const double **scn_dat = CurrentScan->get_points_red();
  getModelPointer(mdl);
  getScenePointer(scn);
  //  cout << "model point cloud size is " << mdlSize << "\n";
  for (unsigned int i = 0; i < mdlSize; ++i) {
    mdl[i][0] = (float) mod_dat[i][0];
    mdl[i][1] = (float) mod_dat[i][1];
    mdl[i][2] = (float) mod_dat[i][2];
  }
  //  cout << "scene point cloud size is " << scnSize << "\n";
  for (unsigned int i = 0; i < scnSize; ++i) {
    scn[0][i] = (float) scn_dat[i][0];
    scn[1][i] = (float) scn_dat[i][1];
    scn[2][i] = (float) scn_dat[i][2];
  }

  ANNkd_tree *mdltree = PreviousScan->getANNTree();
  setTreePointer(mdltree);
  float result[4][4];
  doICP(result);
  Matrix** mats = getMatrices();
  if(anim > 0){
    for(int ci = 0 ; ci < max_num_iterations ; ci += anim){
	 Matrix* cur_mat = mats[ci];
	 double xf[16];
	 for(int i = 1; i < 5 ; ++i)
	   for(int j = 1; j < 5; ++j)
		xf[(i-1)+(j-1)*4] = (*cur_mat)(i,j);
	 CurrentScan->transform(xf,Scan::ICP, 0);
    }
  }
  else{
    double alignxf[16];
    for (int i = 0; i < 4; ++i)
	 for (int j = 0; j < 4; ++j){
	   alignxf[i + j * 4] = result[i][j];
	 }
    CurrentScan->transform(alignxf, Scan::ICP, 0); // write end pose
  }

  cleanup();
  
  return EXIT_SUCCESS;

}

int icp6Dcuda::doICP(float mat[4][4])
{
  icp->setPointClouds();
  
  icp->iteration();
  
  Matrix* M = icp->getMatrix();
  for (int i = 0 ; i < 4 ; ++i){
    for(int j = 0 ; j < 4 ; ++j) {
	 mat[i][j] = (*M)(i+1,j+1);
    }	
  }
  
  return EXIT_SUCCESS;
}

void icp6Dcuda::getModelPointer(double** &mdl){
	mdl = h_idata;
}

void icp6Dcuda::getScenePointer(float** &scn){
	scn = fHstScn;
}

void icp6Dcuda::setTreePointer(ANNkd_tree *&tree){
    icp->setTreePointer(tree);
}
void icp6Dcuda::getTreePointer(ANNkd_tree *&tree){
    icp->getTreePointer(tree);
}
void icp6Dcuda::setMinimums(float x, float y, float z){
    icp->setMinimums(x,y,z);
}

Matrix** icp6Dcuda::getMatrices(){
    return icp->getMatrices();
}
