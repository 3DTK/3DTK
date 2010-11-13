#include "slam6d/d2tree.h"
#include "slam6d/globals.icc"

#include <fstream>
using std::ofstream;

double D2Tree::voxelSize = 10.0;

D2Tree::D2Tree(double **pts, int n, double voxelSize)
{
  this->voxelSize = voxelSize;
  isleaf = false;
  closest_point = 0;

  double xmin = pts[0][0], xmax = pts[0][0];
  double ymin = pts[0][1], ymax = pts[0][1];
  double zmin = pts[0][2], zmax = pts[0][2];
  for (int i = 1; i < n; i++) {
    xmin = min(xmin, pts[i][0]);
    xmax = max(xmax, pts[i][0]);
    ymin = min(ymin, pts[i][1]);
    ymax = max(ymax, pts[i][1]);
    zmin = min(zmin, pts[i][2]);
    zmax = max(zmax, pts[i][2]); 
  }
  center[0] = 0.5 * (xmin+xmax);
  center[1] = 0.5 * (ymin+ymax);
  center[2] = 0.5 * (zmin+zmax);
  x_size = y_size = z_size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin)) + 10000;

  // calculate new buckets
  double newcenter[8][3];
  double x_sizeNew = 0.0, y_sizeNew = 0.0, z_sizeNew = 0.0;

  calcDivide(x_size, y_size, z_size, center,
		   x_sizeNew, y_sizeNew, z_sizeNew, newcenter);
  
  child = new D2Tree*[8];
  child[0] = child[1] = child[2] = child[3] = 0;
  child[4] = child[5] = child[6] = child[7] = 0;

  //  ofstream out1("Points.dat");
  
  for (int i = 0; i < n; i++) {
//      cout << "Processing Point " << i << " ("
//  	    << pts[i][0] << ","
//  	    << pts[i][1] << ","
//  	    << pts[i][2] << ")"<< endl;

// 	out1 << pts[i][0] << " "
// 		<< pts[i][1] << " "
// 		<< -1*pts[i][2] << endl;
    
    updateChilds(pts[i], child, x_sizeNew, y_sizeNew, z_sizeNew, newcenter);

//     cout << "# " << child << endl;
//     for (int j = 0; j < 8; j++) {
// 	 cout << "## " << child [j]<< endl;
// 	 if (child[j] == 0) cout << "ERROR" << endl;
//     } 
  }	 
}


D2Tree::D2Tree(double *pts, double center[3], double x_size, double y_size, double z_size,
			bool _isleaf)
{
  closest_point = 0;
  this->x_size        = x_size;
  this->y_size        = y_size;
  this->z_size        = z_size;
  this->center[0]     = center[0];
  this->center[1]     = center[1];
  this->center[2]     = center[2];
  
  if ((_isleaf) ||  // do we have to create a leaf?
	 // create also a leaf, if bucket gets too small
	 ((x_size < voxelSize) && (y_size < voxelSize) && (z_size < voxelSize))) { 

    isleaf              = true;
    closest_point       = pts;
    child               = 0;    
    
  } else {

    closest_point = 0;
    
    // calculate new buckets
    double newcenter[8][3];
    double x_sizeNew = 0.0, y_sizeNew = 0.0, z_sizeNew = 0.0;

    calcDivide(x_size, y_size, z_size, center,
			x_sizeNew, y_sizeNew, z_sizeNew, newcenter);
    
    child = new D2Tree*[8];
    child[0] = child[1] = child[2] = child[3] = 0;
    child[4] = child[5] = child[6] = child[7] = 0; 

    updateChilds(pts, child, x_sizeNew, y_sizeNew, z_sizeNew, newcenter);

//     cout << "# " << child << endl;
//     for (int j = 0; j < 8; j++) {
// 	 cout << "## " << child [j]<< endl;
// 	 if (child[j] == 0) cout << "ERROR" << endl;
//     } 
  }
}


void D2Tree::updateClosestIn(double *pts)
{
  if ((isleaf) && (x_size < voxelSize) && (y_size < voxelSize) && (z_size < voxelSize)) {
    //    cout << "updateClosestIn: leaf" << endl;
    closest_point = center; //pts;     // = center???
    return;
  } else { // no leaf
    //    cout << "!!!!!!!!!!" << isleaf << endl;
    isleaf = false;
    for (int j = 0; j < 8; j++) {
	 //	 cout << "j " << child << " " << j << endl;
	 if (child == 0) {
	   // calculate new buckets
	   double newcenter[8][3];
	   double x_sizeNew = 0.0, y_sizeNew = 0.0, z_sizeNew = 0.0;

	   calcDivide(x_size, y_size, z_size, center,
			    x_sizeNew, y_sizeNew, z_sizeNew, newcenter);


	   child = new D2Tree*[8];
	   child[0] = child[1] = child[2] = child[3] = 0;
	   child[4] = child[5] = child[6] = child[7] = 0; 

	   updateChilds(pts, child, x_sizeNew, y_sizeNew, z_sizeNew, newcenter);

// 	   for (int j = 0; j < 8; j++) {
// 		if (child[j] == 0) cout << "ERROR" << endl;
// 	   }    
	   
	   //	   cout << "+This case should not happen!" << endl;
	 } else {

// 	 cout << "!!!!!!!!!!" << isleaf << endl;
// 	 for (int l = 0; l < 8; l++) {
// 	   if (child[l] == 0) cout << "ERROR++" << endl;
// 	 }    
	 
	   if (fabs(pts[0] - child[j]->center[0]) <= x_size) {
		if (fabs(pts[1] - child[j]->center[1]) <= y_size) {
		  if (fabs(pts[2] - child[j]->center[2]) <=  z_size) {
		    // point is in child bucket make tree recursively
		    // cout << "updateClosestIn: point is in child bucket make tree recursively" << endl;
		    child[j]->updateClosestIn(pts);
		    continue;
		  }
		}
	   }
	   // point is _not_ in child bucket
		// cout << "updateClosestIn: point is _not_ in child bucket" << endl;
	   child[j]->updateClosestOut(pts);
	 }
    }
  }
}


void D2Tree::updateClosestOut(double *pts)
{
  if (isleaf) {
    if ((closest_point == 0) ||
	   (Dist2(closest_point, center) > Dist2(pts, center))) { // update closest point
	 //	 cout << "updateClosestOut: leaf " << endl;
	 closest_point = pts;    
    }
    return;
  } else { // no leaf
    for (int j = 0; j < 8; j++) {
	 if (child == 0) {
	   // calculate new buckets
	   double newcenter[8][3];
	   double x_sizeNew = 0.0, y_sizeNew = 0.0, z_sizeNew = 0.0;

	   calcDivide(x_size, y_size, z_size, center,
			    x_sizeNew, y_sizeNew, z_sizeNew, newcenter);


	   child = new D2Tree*[8];
	   child[0] = child[1] = child[2] = child[3] = 0;
	   child[4] = child[5] = child[6] = child[7] = 0; 

	   updateChilds(pts, child, x_sizeNew, y_sizeNew, z_sizeNew, newcenter);
// 	   for (int j = 0; j < 8; j++) {
// 		if (child[j] == 0) cout << "ERROR" << endl;
// 	   }    
	   
	   //	   cout << "*This case should not happen!" << endl;
	 } else {
	   //	   cout << "updateClosestOut: child " << j << endl;
	   child[j]->updateClosestOut(pts);
	 }
    }
  }
}

/**
 * Destructor
 */
D2Tree::~D2Tree()
{
  if (child) {
    for(int i = 0; i < 8; i++) {
	 if (child[i] != 0) {
	   delete child[i];
	   child[i] = 0;
	 }
  }
    delete [] child;
    child = 0;
  } 
}


void D2Tree::updateChilds(double *pts, D2Tree **child,
					 double &x_sizeNew, double &y_sizeNew, double &z_sizeNew,
					 double newcenter[8][3])
{
  for (int j = 0; j < 8; j++) {
    if (fabs(pts[0] - newcenter[j][0]) <= x_sizeNew) {
	 if (fabs(pts[1] - newcenter[j][1]) <= y_sizeNew) {
	   if (fabs(pts[2] - newcenter[j][2]) <=  z_sizeNew) {
		if (child[j] == 0) {
		  // point is in child bucket make tree recursively
		  //		  cout << "*point is in child bucket make tree recursively" << endl;
		  child[j] = new D2Tree(pts, newcenter[j], x_sizeNew, y_sizeNew, z_sizeNew, false);
		  continue;
		} else {
		  // point is in child bucket and child is already present
		  //		  cout << "*point is in child bucket and child is already present" << endl;
		  child[j]->updateClosestIn(pts);
		  continue;
		}
	   }
	 }
    }
    
    // point is _not_ in child bucket
    if (child[j] == 0) {
	 // child is _not_ present make leaf node
	 //	 cout << "*child is _not_ present make leaf node" << endl;
	 child[j] =  new D2Tree(pts, newcenter[j], x_sizeNew, y_sizeNew, z_sizeNew, true);
    } else {
	 // child is present update leaf node
	 //	 cout << "*child is present update leaf node" << endl;
	 child[j]->updateClosestOut(pts);
    }
  }
//   cout << "# " << child << endl;
//   for (int j = 0; j < 8; j++) {
//     cout << "## " << child [j]<< endl;
//     if (child[j] == 0) cout << "ERROR" << endl;
//  }    
}

double *D2Tree::_FindClosest(double *p)
{
  if (isleaf) {
    return closest_point;
  } else {
    if (child != 0) {
	 for (int j = 0; j < 8; j++) {
	   if (child[j] == 0) {
		cout << "ERROR" << endl;
	   }
	   // test if point falls into successor
	   if (fabs(p[0] - child[j]->center[0]) <= x_size * 0.5) {
		if (fabs(p[1] - child[j]->center[1]) <= y_size * 0.5) {
		  if (fabs(p[2] - child[j]->center[2]) <=  z_size * 0.5) {
		    if (child[j] != 0) {
			 return child[j]->_FindClosest(p);
		    } else {
			 cout << "ERROR" << endl;
		    }
		  }
		}
	   }
	 }
    } else {
	 cout << "ERROR" << endl; 
    }
  }

  return 0;
}

double *D2Tree::FindClosest(double *p, double maxdist2, int threadNum)
{
  double *pp = 0;
  
  if (fabs(p[0] - center[0]) <= x_size * 0.5) {
    if (fabs(p[1] - center[1]) <= y_size * 0.5) {
	 if (fabs(p[2] - center[2]) <=  z_size * 0.5) {
	   //	   cout << "start search" << endl;
	   pp = _FindClosest(p);
	 }
    }
  }

  if (pp && Dist2(pp, p) > maxdist2) return 0;
  return pp;
}







ofstream out;

void D2Tree::outputTree()
{
  out.open("bboxes.dat");
  cout << "D2Tree" << endl;
  cout << "x_size " << x_size << "y_size " << y_size << "z_size " << z_size << endl;
  int boxes = 0;
  for (int j = 0; j < 8; j++) {
    if (child[j]) boxes += child[j]->_outputTree();
  }
  cout << boxes << endl;
}

int D2Tree::_outputTree()
{
  int boxes = 0;
  if (isleaf) {
    double p[8][3];
    p[0][0] = center[0] - x_size;
    p[0][1] = center[1] - y_size;
    p[0][2] = center[2] - z_size;

    p[1][0] = center[0] + x_size;
    p[1][1] = center[1] - y_size;
    p[1][2] = center[2] - z_size;
    
    p[2][0] = center[0] - x_size;
    p[2][1] = center[1] + y_size;
    p[2][2] = center[2] - z_size;
  
    p[3][0] = center[0] - x_size;
    p[3][1] = center[1] - y_size;
    p[3][2] = center[2] + z_size;
  
    p[4][0] = center[0] + x_size;
    p[4][1] = center[1] + y_size;
    p[4][2] = center[2] - z_size;

    p[5][0] = center[0] + x_size;
    p[5][1] = center[1] - y_size;
    p[5][2] = center[2] + z_size;

    p[6][0] = center[0] - x_size;
    p[6][1] = center[1] + y_size;
    p[6][2] = center[2] + z_size;

    p[7][0] = center[0] + x_size;
    p[7][1] = center[1] + y_size;
    p[7][2] = center[2] + z_size;

    out << p[0][0] << " " << p[0][1] << " " << -1*p[0][2] << endl
        << p[1][0] << " " << p[1][1] << " " << -1*p[1][2] << endl
	   << endl
        << p[1][0] << " " << p[1][1] << " " << -1*p[1][2] << endl
	   << p[4][0] << " " << p[4][1] << " " << -1*p[4][2] << endl
	   << endl
	   << p[4][0] << " " << p[4][1] << " " << -1*p[4][2] << endl
        << p[2][0] << " " << p[2][1] << " " << -1*p[2][2] << endl
	   << endl
        << p[2][0] << " " << p[2][1] << " " << -1*p[2][2] << endl
	   << p[0][0] << " " << p[0][1] << " " << -1*p[0][2] << endl
	   << endl
	   << endl
	 
	   << p[3][0] << " " << p[3][1] << " " << -1*p[3][2] << endl
        << p[5][0] << " " << p[5][1] << " " << -1*p[5][2] << endl
	   << endl
        << p[5][0] << " " << p[5][1] << " " << -1*p[5][2] << endl
	   << p[7][0] << " " << p[7][1] << " " << -1*p[7][2] << endl
	   << endl
	   << p[7][0] << " " << p[7][1] << " " << -1*-1*p[7][2] << endl
        << p[6][0] << " " << p[6][1] << " " << p[6][2] << endl
	   << endl
        << p[6][0] << " " << p[6][1] << " " << -1*p[6][2] << endl
	   << p[3][0] << " " << p[3][1] << " " << -1*p[3][2] << endl
	   << endl
	   << endl
	 
	   << p[3][0] << " " << p[3][1] << " " << -1*p[3][2] << endl
        << p[0][0] << " " << p[0][1] << " " << -1*p[0][2] << endl
	   << endl
        << p[5][0] << " " << p[5][1] << " " << -1*p[5][2] << endl
	   << p[1][0] << " " << p[1][1] << " " << -1*p[1][2] << endl
	   << endl
	   << p[7][0] << " " << p[7][1] << " " << -1*p[7][2] << endl
        << p[4][0] << " " << p[4][1] << " " << -1*p[4][2] << endl
	   << endl
        << p[6][0] << " " << p[6][1] << " " << -1*p[6][2] << endl
	   << p[2][0] << " " << p[2][1] << " " << -1*p[2][2] << endl
	   << endl
	   << endl
	 ;
//     out.close();
//     exit(1);
	 

    return 1;
  }
  for (int j = 0; j < 8; j++) {
    if (child && child[j]) {
	 boxes += child[j]->_outputTree();
    } 
  }
  return boxes;
}

