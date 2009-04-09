/**
 * @file 
 * @brief Implementation of 3D scan matching with ICP
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "defs.h"

#include "icp6D.h"

#ifdef WITH_OPENMP
#include <omp.h>
#endif

#include <iomanip>

/**
 * Constructor 
 *
 * @param my_icp6Dminimizer Pointer to the ICP-minimizer
 * @param max_dist_match Maximum distance to which point pairs are collected
 * @param max_num_iterations Maximum number of iterations
 * @param quiet Whether to print to the standard output
 * @param meta Match against a meta scan?
 * @param rnd Randomized point selection
 * @param eP Extrapolate odometry?
 * @param anim Animate which frames?
 * @param epsilonICP Termination criterion
 * @param use_cache Shall we used cached kd tree search
 */
icp6D::icp6D(icp6Dminimizer *my_icp6Dminimizer, double max_dist_match,
	     int max_num_iterations, bool quiet, bool meta, int rnd, bool eP,
	     int anim, double epsilonICP, bool use_cache)
{
  this->my_icp6Dminimizer = my_icp6Dminimizer;
  this->anim              = anim;
  this->use_cache         = use_cache;

  if (!quiet) {
    cout << "Maximal distance match      : " << max_dist_match << endl
	 << "Maximal number of iterations: " << max_num_iterations << endl << endl;
  }
  
  // checks
  if (max_dist_match < 0.0) {
    cerr << "ERROR [ICP6D]: first parameter (max_dist_match) has to be >= 0," << endl;
    exit(1);
  }
  if (max_num_iterations < 0) {
    cerr << "ERROR [ICP6D]: second parameter (max_num_iterations) has to be >= 0." << endl;
    exit(1);
  }
  
  this->max_dist_match2    = sqr(max_dist_match);
  this->max_num_iterations = max_num_iterations;
  this->quiet              = quiet;
  this->meta               = meta;
  this->rnd                = rnd;
  this->eP                 = eP;
  this->epsilonICP         = epsilonICP;
  
  // Set initial seed (for "real" random numbers)
  //  srand( (unsigned)time( NULL ) );
}

/**
 * Matches a 3D Scan against a 3D Scan
 * @param PreviousScan The scan or metascan forming the model
 * @param CurrentScan The current scan thas is to be matched
 * @return The number of iterations done in this matching run
 */
int icp6D::match(Scan* PreviousScan, Scan* CurrentScan)
{
  // If ICP shall not be applied, then just write
  // the identity matrix
  if (max_num_iterations == 0) {
    double id[16];
    M4identity(id);
    CurrentScan->transform(id, Scan::ICP, 0);  // write end pose
    return 0;
  }

  KDCacheItem *closest = 0;
  if (use_cache) {
    closest = Scan::initCache(PreviousScan, CurrentScan);
  }
  
  // icp main loop
  double ret = 0.0, prev_ret = 0.0, prev_prev_ret = 0.0;
  int iter = 0;
  double alignxf[16];
  for (iter = 0; iter < max_num_iterations; iter++) {

    prev_prev_ret = prev_ret;
    prev_ret = ret;

#ifdef WITH_OPENMP
   // Implementation according to the paper 
   // "The Parallel Iterative Closest Point Algorithm"
   // by Langis / Greenspan / Godin, IEEE 3DIM 2001
   omp_set_num_threads(OPENMP_NUM_THREADS);

   int max = (int)CurrentScan->get_points_red_size();
   int step = max / OPENMP_NUM_THREADS;

   vector<PtPair> pairs[OPENMP_NUM_THREADS];
   double sum[OPENMP_NUM_THREADS];
   double centroid_m[OPENMP_NUM_THREADS][3];
   double centroid_d[OPENMP_NUM_THREADS][3];
   double Si[OPENMP_NUM_THREADS][9];
   unsigned int n[OPENMP_NUM_THREADS];

   for (int i = 0; i < OPENMP_NUM_THREADS; i++) {
	sum[i] = centroid_m[i][0] = centroid_m[i][1] = centroid_m[i][2] = 0.0;
	centroid_d[i][0] = centroid_d[i][1] = centroid_d[i][2] = 0.0;
	Si[i][0] = Si[i][1] = Si[i][2] = Si[i][3] = Si[i][4] = Si[i][5] = Si[i][6] = Si[i][7] = Si[i][8] = 0.0;
	n[i] = 0;
   }

   //   for (int thread_num = 0; thread_num < OPENMP_NUM_THREADS; thread_num++) {
#pragma omp parallel 
   {
	int thread_num = omp_get_thread_num();

	if (use_cache) {
	  Scan::getPtPairsCacheParallel(pairs, closest, PreviousScan, CurrentScan,
							  thread_num, step,
							  rnd, max_dist_match2,
							  sum, centroid_m, centroid_d);
	} else {
	  Scan::getPtPairsParallel(pairs, PreviousScan, CurrentScan,
						  thread_num, step,
						  rnd, max_dist_match2,
						  sum, centroid_m, centroid_d);
	}

	n[thread_num] = (unsigned int)pairs[thread_num].size();

	for (unsigned int i = 0; i < n[thread_num]; i++) {
	  double pp[3] = {pairs[thread_num][i].p1.x - centroid_m[thread_num][0],
				   pairs[thread_num][i].p1.y - centroid_m[thread_num][1],
				   pairs[thread_num][i].p1.z - centroid_m[thread_num][2]};
	  double qq[3] = {pairs[thread_num][i].p2.x - centroid_d[thread_num][0],
				   pairs[thread_num][i].p2.y - centroid_d[thread_num][1],
				   pairs[thread_num][i].p2.z - centroid_d[thread_num][2]};
	  
	  // formula (6)
	  Si[thread_num][0] += pp[0] * qq[0];
	  Si[thread_num][1] += pp[0] * qq[1];
	  Si[thread_num][2] += pp[0] * qq[2];
	  Si[thread_num][3] += pp[1] * qq[0];
	  Si[thread_num][4] += pp[1] * qq[1];
	  Si[thread_num][5] += pp[1] * qq[2]; 
	  Si[thread_num][6] += pp[2] * qq[0];
	  Si[thread_num][7] += pp[2] * qq[1];
	  Si[thread_num][8] += pp[2] * qq[2]; 
	}
   } // end parallel

   // do we have enough point pairs?
   unsigned int pairssize = 0;
   for (int i = 0; i < OPENMP_NUM_THREADS; i++) {
	pairssize = n[i];
   }
   if (pairssize > 3) {
	ret = my_icp6Dminimizer->Point_Point_Align_Parallel(OPENMP_NUM_THREADS,
											  n, sum, centroid_d, centroid_m, Si, 
											  alignxf); 
   } else {
	break;
   }

#else

   double centroid_m[3] = {0.0, 0.0, 0.0};
   double centroid_d[3] = {0.0, 0.0, 0.0};
   vector<PtPair> pairs;
   
   //   Scan::getPtPairsSimple(&pairs, PreviousScan, CurrentScan, 0, rnd, max_dist_match2);
   if (use_cache) {
	Scan::getPtPairsCache(&pairs, closest, PreviousScan, CurrentScan, 0, rnd, max_dist_match2, centroid_m, centroid_d);
   } else {
	Scan::getPtPairs(&pairs, PreviousScan, CurrentScan, 0, rnd, max_dist_match2, centroid_m, centroid_d);
   }
   // do we have enough point pairs?
   if (pairs.size() > 3) {
	ret = my_icp6Dminimizer->Point_Point_Align(pairs, alignxf, centroid_m, centroid_d);
   } else {
	break;
   }
   
#endif

#ifdef DEBUG 
   cerr << alignxf << endl;
#endif

    if ((iter == 0 && anim != -2) || ((anim > 0) && (iter % anim == 0))) {
	 CurrentScan->transform(alignxf, Scan::ICP, 0);   // transform the current scan
    } else {
	 CurrentScan->transform(alignxf, Scan::ICP, -1);  // transform the current scan
    }
    
    if ((fabs(ret - prev_ret) < epsilonICP) && (fabs(ret - prev_prev_ret) < epsilonICP)) {
      double id[16];
      M4identity(id);
      if(anim == -2) {
        CurrentScan->transform(id, Scan::ICP, -1);  // write end pose
      } else {
        CurrentScan->transform(id, Scan::ICP, 0);  // write end pose
      }
      break;
    }
  }

  return iter;
}

/**
 * This function matches the scans only with ICP
 * 
 * @param allScans Contains all necessary scans.
 */
void icp6D::doICP(vector <Scan *> allScans)
{
  double id[16];
  M4identity(id);
  
  vector < Scan* > MetaScan;
  Scan* my_MetaScan = 0;
 
  for(unsigned int i = 0; i < allScans.size(); i++) {
    cout << i << "*" << endl;

    Scan *CurrentScan = allScans[i];
    Scan *PreviousScan = 0;
    
    if (i > 0) {
      PreviousScan = allScans[i-1];
    }
    if (!eP || i == 0) {                             // extrapolate odometry
      CurrentScan->mergeCoordinatesWithRoboterPosition();
    } else {
      CurrentScan->mergeCoordinatesWithRoboterPosition(PreviousScan);
    }

    if (i > 0) {
      if (meta) {
	   // if ((i < 220-22) && (i > 250-22)) match(allScans[0], CurrentScan);
	   // else
     //match(allScans[0], CurrentScan);
	   match(my_MetaScan, CurrentScan);
      } else {
	   // if ((i < 220-22) && (i > 250-22)) match(allScans[0], CurrentScan);
	   // else
     //match(allScans[0], CurrentScan);
	   match(PreviousScan, CurrentScan);
      }
    }
    
    // push processed scan
    if ( meta && i != allScans.size()-1 ) {
      MetaScan.push_back(CurrentScan);
      if (my_MetaScan) {
	   delete my_MetaScan;
      }
      my_MetaScan = new Scan(MetaScan, use_cache);
    }
  }
}

//geklaut aus lum6Dquat.cc
//TODO: in lum6Dquat.cc diese Methode einbauen
/**
 * This function calculates the covariances Cij and the Vector Cij*Dij for
 * all links, using the given point pairs.
 * 
 * @param numLinks Number of links in the graph and thereore in the linear system
 * @param vptpairs A vector containing a set of all point pairs in the same order as specified by gr
 * @param CD After CalculateLins3D() returns, this will contain all
 *        Cij*Dij in the same order as specified by gr
 * @param C After CalculateLins3D() returns, this will contain all Cij in the same order as specified by gr
 */
//void lum6DQuat::CalculateLinks3D(int numLinks, vPtPair **ptpairs,
//						   vector <ColumnVector >* CD , vector <Matrix>* C)
void icp6D::covarianceQuat(Scan *scan1, Scan *scan2, Matrix *C)
{
  // x,y,z       denote the coordinates of uk (Here averaged over ak and bk)
  // sx,sy,sz    are the sums of their respective coordinates of uk over each paint pair
  // xpy,xpz,ypz are the sums over x*x + y*y ,x*x + z*z and y*y + z*z respectively over each point pair
  // xy,yz,xz    are the sums over each respective multiplication 
  // dx,dy,dz    are the deltas in each coordinate of a point pair
  // ss          is the estimation of the covariance of sensing error
  double x, y, z, sx, sy, sz, xy, yz, xz, ypz, xpz, xpy, dx, dy, dz, ss, xpypz;

  // D is needed to calculate the estimation of the covariance s
  ColumnVector D(7);
  // Almost Cij*Dij
  ColumnVector MZ(7);
  // Almost the covarianve
  Matrix MM(7,7);
  // A set of point pairs
  //vPtPair *uk;
  vector <PtPair> uk;
  // A point pair
  Point ak, bk;
  // number of pairs in a set
  int m;

#ifdef WITH_OPENMP
  int thread_num = omp_get_thread_num();
#else
  int thread_num = 0;
#endif

  double dummy_centroid_m[3];
  double dummy_centroid_d[3];

  if (use_cache) {
    KDCacheItem *closest = Scan::initCache(scan1, scan2);
    Scan::getPtPairsCache(&uk, closest, scan1, scan2, thread_num, rnd, max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  } else {
    Scan::getPtPairs(&uk, scan1, scan2, thread_num, rnd, max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  }

  // for every link in the network
  //for(int i = 0; i < numLinks; i++){
    //uk = ptpairs[i];
    m = uk.size();

    MZ = 0.0;
    MM = 0.0;
    sx = sy = sz = xy = yz = xz = ypz = xpz = xpy = xpypz = ss = 0.0;
    
    if (m > 0) {
	 // for each point pair
	 for(int j = 0; j < m; j++){
        ak = uk[j].p1;
        bk = uk[j].p2;
        
        // Some temporary values
        x = (ak.x + bk.x)/2.0;
        y = (ak.y + bk.y)/2.0;
        z = (ak.z + bk.z)/2.0;
        dx = ak.x - bk.x;
        dy = ak.y - bk.y;
        dz = ak.z - bk.z;

        // Sum up all necessary values to construct MM
        sx += x;
        sy += y;
        sz += z;

        xpy += x*x + y*y;
        xpz += x*x + z*z;
        ypz += y*y + z*z;
        xpypz += x*x + y*y + z*z;

        xy += x*y;
        xz += x*z;
        yz += y*z;

        // Sum up each part of MZ
        MZ(1) += dx;
        MZ(2) += dy;
        MZ(3) += dz;
        MZ(4) += x * dx + y * dy + z * dz;
        MZ(5) += z * dy - y * dz;
        MZ(6) += x * dz - z * dx;
        MZ(7) += y * dx - x * dy;
	 }
	 // Now construct the symmetrical matrix MM
	 MM(1,1) = MM(2,2) = MM(3,3) = m;
   
	 MM(4,4) = xpypz;
	 MM(5,5) = ypz;
	 MM(6,6) = xpz;
	 MM(7,7) = xpy;
    
   MM(1,4) = MM(4,1) = sx;
	 MM(1,6) = MM(6,1) = -sz;
	 MM(1,7) = MM(7,1) = sy;
             
	 MM(2,4) = MM(4,2) = sy;
	 MM(2,5) = MM(5,2) = sz;
	 MM(2,7) = MM(7,2) = -sx;
             
	 MM(3,4) = MM(4,3) = sz;
	 MM(3,5) = MM(5,3) = -sy;
	 MM(3,6) = MM(6,3) = sx;
   
	 MM(5,6) = MM(6,5) = -xy;
	 MM(5,7) = MM(7,5) = -xz;
	 MM(6,7) = MM(7,6) = -yz;
   
	 // Calculate the pose difference estimation
	 D = MM.i() * MZ ;

	 // Again going through all point pairs to faster calculate s.
	 // This cannot be done earlier as we need D, and therefore MM and MZ to do this
	 for(int j = 0; j < m; j++){
	   ak = uk[j].p1;
	   bk = uk[j].p2;
   
	   x = (ak.x + bk.x) / 2.0;
	   y = (ak.y + bk.y) / 2.0;
	   z = (ak.z + bk.z) / 2.0;
      
	   ss += sqr(ak.x - bk.x - (D(1) + x * D(4) - z * D(6) + y * D(7)))
		  + sqr(ak.y - bk.y - (D(2) + y * D(4) + z * D(5) - x * D(7)))
		  + sqr(ak.z - bk.z - (D(3) + z * D(4) - y * D(5) + x * D(6)));
	 }

	 ss =  ss / (2*m - 3);
	 //ss = 1.0 / ss;

	 //CD->push_back(MZ * ss);
	 //C->push_back(MM * ss);
   MM = MM.i();
   *C = MM * ss;
	 
    } else {
	 
      // This case should not occur
	 ss = 0.0;
	 MM(1,1) = MM(1,2) = MM(1,3) = 0.0;
	 MM(2,1) = MM(2,2) = MM(2,3) = 0.0;
	 MM(3,1) = MM(3,2) = MM(3,3) = 0.0;
	 MZ(6) = MZ(1) = MZ(2) = MZ(7) = 0.0;
	 MZ(3) = MZ(4) = MZ(5) = 0.0;
	 //CD->push_back(MZ);
	 //C->push_back(MM);
   cerr << "Error calculating covariance matrix" << endl;
   exit(1);
	 
    }
  //}
}

//geklaut aus lum6Deuler.cc
//TODO: in lum6Deuler.cc diese Methode einbauen
/**
 * This function calculates the covariances Cij and the Vector Cij*Dij for
 * all links, using the given point pairs.
 * 
 * @param numLinks Number of links in the graph and thereore in the linear system
 * @param vptpairs A vector containing a set of all point pairs in the same order as specified by gr
 * @param CD After CalculateLins3D() returns, this will contain all
 *        Cij*Dij in the same order as specified by gr
 * @param C After CalculateLins3D() returns, this will contain all Cij in the same order as specified by gr
*/
//void lum6DEuler::CalculateLinks3D(int numLinks, vPtPair **ptpairs,
//						    vector <ColumnVector >* CD , vector <Matrix>* C)
void icp6D::covarianceEuler(Scan *scan1, Scan *scan2, Matrix *C)
{
  // x,y,z       denote the coordinates of uk (Here averaged over ak and bk)
  // sx,sy,sz    are the sums of their respective coordinates of uk over each paint pair
  // xpy,xpz,ypz are the sums over x*x + y*y ,x*x + z*z and y*y + z*z respectively over each point pair
  // xy,yz,xz    are the sums over each respective multiplication 
  // dx,dy,dz    are the deltas in each coordinate of a point pair
  // ss          is the estimation of the covariance of sensing error
  double x, y, z, sx, sy, sz, xy, yz, xz, ypz, xpz, xpy, dx, dy, dz, ss;

  // D is needed to calculate the estimation of the covariance s
  ColumnVector D(6);
  // Almost Cij*Dij
  ColumnVector MZ(6);
  // Almost the covarianve
  Matrix MM(6,6);
  // A set of point pairs
  //vPtPair *uk;
  vector <PtPair> uk;
  // A point pair
  Point ak, bk;
  // number of pairs in a set
  int m;

#ifdef WITH_OPENMP
  int thread_num = omp_get_thread_num();
#else
  int thread_num = 0;
#endif

  double dummy_centroid_m[3];
  double dummy_centroid_d[3];

  if (use_cache) {
    KDCacheItem *closest = Scan::initCache(scan1, scan2);
    Scan::getPtPairsCache(&uk, closest, scan1, scan2, thread_num, rnd, max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  } else {
    Scan::getPtPairs(&uk, scan1, scan2, thread_num, rnd, max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  }

  // for every link in the network
  //for(int i = 0; i < numLinks; i++){
    //uk = ptpairs[i];
    m = uk.size();

    MZ = 0.0;
    MM = 0.0;
    sx = sy = sz = xy = yz = xz = ypz = xpz = xpy = ss = 0.0;
    
    if (m > 0) {
	 // for each point pair
	 for(int j = 0; j < m; j++){
        ak = uk[j].p1;
        bk = uk[j].p2;
        
        // Some temporary values
        x = (ak.x + bk.x)/2.0;
        y = (ak.y + bk.y)/2.0;
        z = (ak.z + bk.z)/2.0;
        dx = ak.x - bk.x;
        dy = ak.y - bk.y;
        dz = ak.z - bk.z;

        // Sum up all necessary values to construct MM
        sx += x;
        sy += y;
        sz += z;

        xpy += x*x + y*y;
        xpz += x*x + z*z;
        ypz += y*y + z*z;

        xy += x*y;
        xz += x*z;
        yz += y*z;

        // Sum up each part of MZ
        MZ(1) += dx;
        MZ(2) += dy;
        MZ(3) += dz;
        MZ(4) += -z * dy + y * dz;
        MZ(5) += -y * dx + x * dy;
        MZ(6) += z * dx - x * dz;
	 }
	 // Now construct the symmetrical matrix MM
	 MM(1,1) = MM(2,2) = MM(3,3) = m;
	 MM(4,4) = ypz;
	 MM(5,5) = xpy;
	 MM(6,6) = xpz;
    
	 MM(1,5) = MM(5,1) = -sy;
	 MM(1,6) = MM(6,1) = sz;
	 MM(2,4) = MM(4,2) = -sz;
	 MM(2,5) = MM(5,2) = sx;
	 MM(3,4) = MM(4,3) = sy;
	 MM(3,6) = MM(6,3) = -sx;
    
	 MM(4,5) = MM(5,4) = -xz;
	 MM(4,6) = MM(6,4) = -xy;
	 MM(5,6) = MM(6,5) = -yz;

	 // Calculate the pose difference estimation
	 D = MM.i() * MZ ;

	 // Again going through all point pairs to faster calculate s.
	 // This cannot be done earlier as we need D, and therefore MM and MZ to do this
	 for(int j = 0; j < m; j++){
	   ak = uk[j].p1;
	   bk = uk[j].p2;
   
	   x = (ak.x + bk.x) / 2.0;
	   y = (ak.y + bk.y) / 2.0;
	   z = (ak.z + bk.z) / 2.0;
      
	   ss += sqr(ak.x - bk.x - (D(1) - y * D(5) + z * D(6)))
		+ sqr(ak.y - bk.y - (D(2) - z * D(4) + x * D(5)))
		+ sqr(ak.z - bk.z - (D(3) + y * D(4) - x * D(6)));
	 }

	 ss =  ss / (2*m - 3);
	 //ss = 1.0 / ss;

	 //CD->push_back(MZ * ss);
	 //C->push_back(MM * ss);
   MM = MM.i();
   *C = MM * ss;
	 
    } else {
	 
      // This case should not occur
	 ss = 0.0;
	 MM(1,1) = MM(1,2) = MM(1,3) = 0.0;
	 MM(2,1) = MM(2,2) = MM(2,3) = 0.0;
	 MM(3,1) = MM(3,2) = MM(3,3) = 0.0;
	 MZ(6) = MZ(1) = MZ(2) = 0.0;
	 MZ(3) = MZ(4) = MZ(5) = 0.0;
	 //CD->push_back(MZ);
	 //C->push_back(MM);
   cerr << "Error calculating covariance matrix" << endl;
   exit(1);
	 
    }
  //}
}

