/*
 * accumulator implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "shapes/accumulator.h"
#include <math.h>
#include "slam6d/globals.icc"
#include <iostream>

#ifdef _MSC_VER
#define isnan(_X) ((_X) != (_X))
#endif

using namespace std;

double* polar2normal(double theta, double phi) {
  double * n = new double[4];
  n[0] = cos(theta)*sin(phi);
  n[1] = sin(theta)*sin(phi);
  n[2] = cos(phi);
  Normalize3(n);
  return n;
}

AccumulatorSimple::AccumulatorSimple(ConfigFileHough myCfg) {
 
  count = 0;
  myConfigFileHough = myCfg;
  accumulator = new int**[myConfigFileHough.Get_RhoNum()];

  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    accumulator[i] = new int*[myConfigFileHough.Get_PhiNum()];
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      accumulator[i][j] = new int[myConfigFileHough.Get_ThetaNum()];
      for(unsigned int k = 0; k < myConfigFileHough.Get_ThetaNum(); k++) {
        accumulator[i][j][k] = 0;
      }
    }
  }
}

AccumulatorSimple::~AccumulatorSimple() {

  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      delete[] accumulator[i][j];
    }
    delete[] accumulator[i];
    
  }
  delete[] accumulator;
  
}  

void AccumulatorSimple::printAccumulator() {
  
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_ThetaNum(); j++) {
      double phi1 = i * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999);
      double phi2 = (i+1) * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999);
      double theta1 = j * 2*M_PI / myConfigFileHough.Get_ThetaNum();
      double theta2 = (j+1) * 2*M_PI / myConfigFileHough.Get_ThetaNum();
      if(theta2 > 2*M_PI) theta2 = 2*M_PI;
      if(phi2 > M_PI) {
        phi2 = M_PI;
      }
      int rhosum = 0;
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        rhosum += accumulator[k][i][j];
      }
      cout << phi1 << " " << theta1 << " " << " " << rhosum << " 40" << endl;
      cout << phi2 << " " << theta1 << " " << " " << rhosum << " 40" << endl;
      cout << phi2 << " " << theta2 << " " << " " << rhosum << " 40" << endl;
      cout << phi1 << " " << theta2 << " " << " " << rhosum << " 40" << endl;
      cout << endl;
    }

    cout << endl;
  }
  
  /*
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_ThetaNum(); j++) {
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        if( accumulator[i][j][k] > 20 ) cout << i << " " << j << " " << k << endl;
      }
    }
  }
  */

}

void AccumulatorSimple::resetAccumulator() {
  count = 0;
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      for(unsigned int k = 0; k < myConfigFileHough.Get_ThetaNum(); k++) {
        accumulator[i][j][k] = 0;
      }
    }
  }
}

bool AccumulatorSimple::accumulate(double theta, double phi, double rho) {
  count++;
  //cout << phi << " " << theta << " " << rho << " ";
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)((rho*(double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }
  //cout << (rhoindex + 0.5) * myConfigFileHough.Get_RhoMax()/myConfigFileHough.Get_RhoNum() << endl;
  
  int phiindex = (int)(phi*((myConfigFileHough.Get_PhiNum()*0.99999999)/(M_PI)));
  int thetaindex = (int)(theta*((myConfigFileHough.Get_ThetaNum()*0.9999999)/(2*M_PI)));
  
  //thetaindex = thetaindex % ballNr[phiindex];
  accumulator[rhoindex][phiindex][thetaindex]++;
  return ((unsigned int)accumulator[rhoindex][phiindex][thetaindex] >= myConfigFileHough.Get_AccumulatorMax());
}

void AccumulatorSimple::accumulate(Point p) {
//  cout << p.x << " " << p.y << " " << p.z << endl;

  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    //TODO 0.99 vielleicht nicht gut
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999999);
  
    for(unsigned int j = 0; j < myConfigFileHough.Get_ThetaNum(); j++) {
      double theta = (j+0.5) * 2*M_PI / myConfigFileHough.Get_ThetaNum();
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
        double n[3];
        n[0] = cos(theta)*sin(phi);
        n[1] = sin(theta)*sin(phi);
        n[2] = cos(phi);
        Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
        }

      }
    }
  }

//  cout << "fertig" << endl;
  
}

double* AccumulatorSimple::accumulateRet(Point p) {
  count++;
  // rho theta phi
  double* angles = new double[3]; 
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.9999999);
  
    for(unsigned int j = 0; j < myConfigFileHough.Get_ThetaNum(); j++) {
      double theta = (j+0.5) * 2*M_PI / myConfigFileHough.Get_ThetaNum();
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
        double n[3];
        n[0] = cos(theta)*sin(phi);
        n[1] = sin(theta)*sin(phi);
        n[2] = cos(phi);
        Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
          if(((unsigned int)accumulator[k][i][j] > myConfigFileHough.Get_AccumulatorMax() && (unsigned int)accumulator[k][i][j] > count*myConfigFileHough.Get_PlaneRatio())
          || (unsigned int)accumulator[k][i][j] > myConfigFileHough.Get_AccumulatorMax()) { 
            angles[0] = rho;
            angles[1] = theta;
            angles[2] = phi;
            return angles;
          }
        }

      }
    }
  }

  angles[0] = -1;
  return angles;
  
}

int* AccumulatorSimple::accumulateAPHT(Point p) {

  int tmpInt = 0;
  // rho theta phi
  int* angles = new int[3]; 
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999);
  
    for(unsigned int j = 0; j < myConfigFileHough.Get_ThetaNum(); j++) {
      double theta = (j+0.5) * 2*M_PI / myConfigFileHough.Get_ThetaNum();
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
        double n[3];
        n[0] = cos(theta)*sin(phi);
        n[1] = sin(theta)*sin(phi);
        n[2] = cos(phi);
        Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
          if(accumulator[k][i][j] > tmpInt) {
            angles[0] = k;
            angles[1] = j;
            angles[2] = i;
            tmpInt = accumulator[k][i][j];
          }
        }

      }
    }
  }

  return angles;
  
}


double* AccumulatorSimple::getMax(double &rho, double &theta, double &phi) {
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)(rho*((double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }
  
  int phiindex = (int)(phi*((double)(myConfigFileHough.Get_PhiNum())/(M_PI)));
  int thetaindex = (int)(theta*(((double)myConfigFileHough.Get_ThetaNum())/(2*M_PI)));
  thetaindex = thetaindex % myConfigFileHough.Get_ThetaNum();

  phi = (0.5 + phiindex) * (double)(M_PI / (double)myConfigFileHough.Get_PhiNum());
  theta = (0.5 + thetaindex) * (double)(2.0 * M_PI / (double)myConfigFileHough.Get_ThetaNum());
  rho = (rhoindex + 0.5) * (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  return polar2normal(theta, phi);
}

double* AccumulatorSimple::getMax(int* cell) {
  double phi = (0.5 + cell[3]) * (double)(M_PI / (double)myConfigFileHough.Get_PhiNum());
  double theta = (0.5 + cell[2]) * (double)(2.0 * M_PI /
  (double)myConfigFileHough.Get_ThetaNum());
  double rho = (cell[1] + 0.5) *
  (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  double* polar = polar2normal(theta, phi);
  polar[3] = rho;
  return polar;
}

multiset<int*, maxcompare>* AccumulatorSimple::getMax() {
//void AccumulatorSimple::getMax() {
  multiset<int*, maxcompare>* maxlist = new multiset<int*, maxcompare>();
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      for(unsigned int k = 0; k < myConfigFileHough.Get_ThetaNum(); k++) {
        int* tmp  = new int[4];
        tmp[0] = accumulator[i][j][k];
        tmp[1] = i;
        tmp[2] = k;
        tmp[3] = j;
        maxlist->insert(tmp);
      }
    }
  }
  return maxlist;
}

void AccumulatorSimple::peakWindow(int size) {
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum() - size; i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum() - size; j++) {
      for(unsigned int k = 0; k < myConfigFileHough.Get_ThetaNum() - size; k++) {
        int max = 0;
        for(unsigned int ii = i; ii < i + size; ii++) {
          for(unsigned int ji = j; ji < j + size; ji++) {
            for(unsigned int ki = k; ki < k + size; ki++) {
              if(accumulator[ii][ji][ki] > max) {
                max = accumulator[ii][ji][ki];
              }
            }
          }
        }
        for(unsigned int ii = i; ii < i + size; ii++) {
          for(unsigned int ji = j; ji < j + size; ji++) {
            for(unsigned int ki = k; ki < k + size; ki++) {
              if(accumulator[ii][ji][ki] < max) {
                accumulator[ii][ji][ki] = 0;
              }
            }
          }
        }
      }
    }
  }
}

AccumulatorBall::AccumulatorBall(ConfigFileHough myCfg) {
  count = 0;
  int countCells = 0;
  myConfigFileHough = myCfg;

  double c = 0.0;
  int counter = 0;
  ballNr = new int[myConfigFileHough.Get_PhiNum()];
  double step = 180.0/myConfigFileHough.Get_PhiNum();

  //double MAX_A = M_PI * (2 * fabs(0 - cos(rad(90- step))) + sin(rad(90 - step)) * sin(rad(90 - step)) + 1 * 1); 

  double MAX_A = 2 * M_PI * 1.0;

  double r = sin(rad(0.0));
  double z = cos(rad(0.0));
  double r_next = 0.0;
  double z_next = 0.0;
  
  for(double phi = 0; phi < 180.0; phi+=step) {
    r_next = sin(rad(phi+step));
    z_next = cos(rad(phi+step));
  // Flaecheninhalt der aktuellen Kreisscheibe 
    //double a = (M_PI * (2.0 * 1.0 * fabs(z - z_next) + r*r + r_next*r_next));
    // Umfang des aktuellen Kreises
    double a = 2 * M_PI * (r + r_next)/2.0;
    //a = (r*r - r_next*r_next) * PI; //Flaechenvergleich

    // Berechnung der Schrittgroesse in Grad
    c = ((360.0 * (MAX_A / a)) / (myConfigFileHough.Get_ThetaNum() - 1));
    // Berechnung der Felder im Akkumulatorarray
    int size = (int)(1.0 + 360.0/c);
    //if(true) {
    ballNr[counter] = size;
    //} else {
    //  ballNr[counter] = myConfigFileHough.Get_ThetaNum();
    //}
   
    //cout << counter << endl;
    r = r_next;
    z = z_next;
    counter++;
  }

  
  accumulator = new int**[myConfigFileHough.Get_RhoNum()];

  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    accumulator[i] = new int*[myConfigFileHough.Get_PhiNum()];
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      accumulator[i][j] = new int[ballNr[j]];
      for(int k = 0; k < ballNr[j]; k++) {
        accumulator[i][j][k] = 0;
        countCells++;
      }
    }
  }
  cout << "CountCells " << countCells << endl;
}

AccumulatorBall::~AccumulatorBall() {
  
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      delete[] accumulator[i][j];
    }
    delete[] accumulator[i];
    
  }
  delete[] accumulator;
  delete[] ballNr;
  
}  

void AccumulatorBall::printAccumulator() {
  int sum = 0;
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    sum += ballNr[i];
  }
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    for(int j = 0; j < ballNr[i]; j++) {
      double phi1 = i * M_PI / (myConfigFileHough.Get_PhiNum()*0.9999999999);
      double phi2 = (i+1) * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999);
      double theta1 = j * 2*M_PI / ballNr[i];
      double theta2 = (j+1) * 2*M_PI / ballNr[i];
      if(theta2 > 2*M_PI) theta2 = 2*M_PI;
      if(phi2 > M_PI) {
        phi2 = M_PI;
      }
      int rhosum = 0;
      /*
      for(int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        rhosum += accumulator[k][i][j];
      }
      */
      rhosum = accumulator[33][i][j];
      cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
      cout << phi2 << " " << theta1 << " " << rhosum << " 40" << endl;
      cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
      cout << phi1 << " " << theta2 << " " << rhosum << " 40" << endl;
      cout << endl;
    }

    //cout << endl;
  }
}

void AccumulatorBall::resetAccumulator() {

  count = 0;
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      int bNr = ballNr[j];
      for(int k = 0; k < bNr; k++) {
        accumulator[i][j][k] = 0;
      }
    }
  }
}

bool AccumulatorBall::accumulate(double theta, double phi, double rho) {
  count++;
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)((rho*(double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  } 
  
  int phiindex = (int)(phi*(((double)myConfigFileHough.Get_PhiNum()*0.9999999)/(M_PI)));
  int bNr = ballNr[phiindex];
  int thetaindex = (int)(theta*((bNr*0.9999999999)/(2*M_PI)));
  if(thetaindex >= bNr) {
    cout << "duet";
  }
  //thetaindex = thetaindex % ballNr[phiindex];
  accumulator[rhoindex][phiindex][thetaindex]++;
  return ((unsigned int)accumulator[rhoindex][phiindex][thetaindex] >= myConfigFileHough.Get_AccumulatorMax());
}

void AccumulatorBall::accumulate(Point p) {
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.999999999);
    for(int j = 0; j < ballNr[i]; j++) {
      double theta = (j+0.5) * 2*M_PI / ballNr[i];
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
      double n[3];
      n[0] = cos(theta)*sin(phi);
      n[1] = sin(theta)*sin(phi);
      n[2] = cos(phi);
      Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
        }

      }
    }
  }

}
double* AccumulatorBall::accumulateRet(Point p) {
  count++;
  // rho theta phi
  double* angles = new double[3]; 
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999);
    for(int j = 0; j < ballNr[i]; j++) {
      double theta = (j+0.5) * 2*M_PI / ballNr[i];
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
      double n[3];
      n[0] = cos(theta)*sin(phi);
      n[1] = sin(theta)*sin(phi);
      n[2] = cos(phi);
      Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
          if(
          ((unsigned int)accumulator[k][i][j] > myConfigFileHough.Get_AccumulatorMax() && 
          (unsigned int)accumulator[k][i][j] > count*myConfigFileHough.Get_PlaneRatio()) || 
          (unsigned int)accumulator[k][i][j] > 10*myConfigFileHough.Get_AccumulatorMax()) {
            angles[0] = rho;
            angles[0] = rho;
            angles[1] = theta;
            angles[2] = phi;
            return angles;
          }
        }

      }
    }
  }
  angles[0] = -1;
  return angles;
}

int* AccumulatorBall::accumulateAPHT(Point p) {

  // rho theta phi
  int tmpMax = 0;
  int* angles = new int[4]; 
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.9999999999);
    for(int j = 0; j < ballNr[i]; j++) {
      double theta = (j+0.5) * 2*M_PI / ballNr[i];
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
      double n[3];
      n[0] = cos(theta)*sin(phi);
      n[1] = sin(theta)*sin(phi);
      n[2] = cos(phi);
      Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
          if(accumulator[k][i][j] > tmpMax) {
            angles[1] = k;
            angles[2] = j;
            angles[3] = i;
            angles[0] = tmpMax = accumulator[k][i][j];
          }
        }

      }
    }
  }
  return angles;
}

double* AccumulatorBall::getMax(double &rho, double &theta, double &phi) {
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  double tmprho = rho;
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)(rho*0.99999999*((double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }
  
  int phiindex = (int)(phi*0.99999999*(((double)myConfigFileHough.Get_PhiNum())/(M_PI)));
  int bNr = ballNr[phiindex];
  int thetaindex = (int)(theta*((bNr + 1)/(2*M_PI)));
  thetaindex = thetaindex % bNr;

  phi = (0.5 + phiindex) * (double)(M_PI / (double)myConfigFileHough.Get_PhiNum());
  theta = (0.5 + thetaindex) * (double)(2.0 * M_PI / bNr);
  rho = (rhoindex + 0.5) * (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  if(fabs(rho - rho) > 0.01) cout << rho << " " << tmprho << " " << rhoindex << endl;
  return polar2normal(theta, phi);
}
double* AccumulatorBall::getMax(int* cell) {
  double phi = (0.5 + cell[3]) * (double)(M_PI / (double)myConfigFileHough.Get_PhiNum());
  int bNr = ballNr[cell[3]];
  double theta = (0.5 + cell[2]) * (double)(2.0 * M_PI / bNr);
  double rho = (cell[1] + 0.5) *
  (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  double* polar = polar2normal(theta, phi);
  if(std::isnan(polar[0])) {
    cout << cell[1] << " " << cell[2] << " " << cell[3] << endl;
    cout << theta <<" bla " << phi << endl;
  }
  polar[3] = rho;
  return polar;
}

multiset<int*, maxcompare>* AccumulatorBall::getMax() {
//void AccumulatorBall::getMax() {
  multiset<int*, maxcompare>* maxlist = new multiset<int*, maxcompare>();
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      int bNr = ballNr[j];
      for(int k = 0; k < bNr; k++) {
        int * tmp = new int[4];
        tmp[0] = accumulator[i][j][k];
        tmp[1] = i;
        tmp[2] = k;
        tmp[3] = j;
        maxlist->insert(tmp);
      }
    }
  }
  cout << maxlist->size() << endl;
  return maxlist;
}

void AccumulatorBall::peakWindow(int size) {
  //cout << "Wir haben " << sum << "Zellen!" << endl;
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum() - size; i++) {
    //cout << "Peak " << i << endl;
    for(int j = 0; j < ballNr[i] - size; j++) {
      
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum() - size; k++) {
        int max = 0;
        for(unsigned int ii = i; (ii < (i + size)) && (ii < myConfigFileHough.Get_PhiNum()); ii++) {
          for(int ji = j; (ji < (j + size)) && (ji < ballNr[ii]); ji++) {
            for(unsigned int ki = k; (ki < (k + size)) && (ki < myConfigFileHough.Get_RhoNum()); ki++) {
              if(accumulator[ki][ii][ji] > max) {
                max = accumulator[ki][ii][ji];
              }
            }
          }
        }
        for(unsigned int ii = i; ii < i + size && ii < myConfigFileHough.Get_PhiNum(); ii++) {
          for(int ji = j; ji < j + size && ji < ballNr[ii]; ji++) {
            for(unsigned int ki = k; ki < k + size && ki < myConfigFileHough.Get_RhoNum(); ki++) {
              if(accumulator[ki][ii][ji] < max) {
                accumulator[ki][ii][ji] = 0;
              }
            }
          }
        }
      }
    }
  }
}

//BLUB



AccumulatorCube::AccumulatorCube(ConfigFileHough myCfg) {
  int countCells = 0;
  count = 0;
  myConfigFileHough = myCfg;
  nrCells = myConfigFileHough.Get_ThetaNum()/4;
  accumulator = new int***[6];
  for(int i = 0; i < 6; i++) {
    accumulator[i] = new int**[nrCells];
    for(int j = 0; j < nrCells; j++) {
      accumulator[i][j] = new int*[nrCells];
      for(int k = 0; k < nrCells; k++) {
        accumulator[i][j][k] = new int[myConfigFileHough.Get_RhoNum()];
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          accumulator[i][j][k][l] = 0;
          countCells++;
        }
      }
    }
  }
  cout << "countCells " << countCells << endl;
}

AccumulatorCube::~AccumulatorCube() {
  nrCells = myConfigFileHough.Get_ThetaNum()/4;
  
  for(unsigned int i = 0; i < 6; i++) {
    for(int j = 0; j < nrCells; j++) {
      for(int k = 0; k < nrCells; k++) {
        delete[] accumulator[i][j][k];
      }
      delete[] accumulator[i][j];
    }
    delete[] accumulator[i];
    
  }
  delete[] accumulator;
  
}

void AccumulatorCube::printAccumulator2() {
  
  double **result;
  result = new double*[4];
  result[0] = new double[3]; 
  result[1] = new double[3]; 
  result[2] = new double[3]; 
  result[3] = new double[3]; 
  
  int j;
  int k;
  
  for(int i = 0; i < 6; i+=1) {
    
    for(j = 1, k = 1; j <= nrCells; j++) {

      buffer_point bptmp;
      bptmp.face = i + 1;
      bptmp.i = j;
      bptmp.j = k;

      coords_cube_for_print(bptmp, result, nrCells);
      //double* tmp = coords_cube_to_s2(bptmp, nrCells);
      double polar1[3];
      double polar2[3];
      double polar3[3];
      double polar4[3];
      //toPolar(tmp, polar1);
   
      Normalize3(result[0]);
      Normalize3(result[1]);
      Normalize3(result[2]);
      Normalize3(result[3]);

      toPolar(result[0], polar1);
      toPolar(result[1], polar2);
      toPolar(result[2], polar3);
      toPolar(result[3], polar4);
      double phi1 = polar1[0];
      double theta1 = polar1[1];
      //double phi2 = polar2[0];
      //double theta2 = polar2[1];
      //double phi3 = polar3[0];
      //double theta3 = polar3[1];
      double phi4 = polar4[0];
      double theta4 = polar4[1];

      cout << phi1 << " " << theta1 << " " <<  " 40" << endl;
      cout << phi4 << " " << theta4 << " " <<  " 40" << endl;
    //cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
    }
    for(j--; k <= nrCells; k++) {

      buffer_point bptmp;
      bptmp.face = i + 1;
      bptmp.i = j;
      bptmp.j = k;

      coords_cube_for_print(bptmp, result, nrCells);
      //double* tmp = coords_cube_to_s2(bptmp, nrCells);
      double polar1[3];
      double polar2[3];
      double polar3[3];
      double polar4[3];
      //toPolar(tmp, polar1);
   
      Normalize3(result[0]);
      Normalize3(result[1]);
      Normalize3(result[2]);
      Normalize3(result[3]);

      toPolar(result[0], polar1);
      toPolar(result[1], polar2);
      toPolar(result[2], polar3);
      toPolar(result[3], polar4);
      //double phi1 = polar1[0];
      //double theta1 = polar1[1];
      //double phi2 = polar2[0];
      //double theta2 = polar2[1];
      double phi3 = polar3[0];
      double theta3 = polar3[1];
      double phi4 = polar4[0];
      double theta4 = polar4[1];

      cout << phi4 << " " << theta4 << " " << " 40" << endl;
      cout << phi3 << " " << theta3 << " " << " 40" << endl;
    //cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
    }
    for(k--; j > 0; j--) {

      buffer_point bptmp;
      bptmp.face = i + 1;
      bptmp.i = j;
      bptmp.j = k;

      coords_cube_for_print(bptmp, result, nrCells);
      //double* tmp = coords_cube_to_s2(bptmp, nrCells);
      double polar1[3];
      double polar2[3];
      double polar3[3];
      double polar4[3];
      //toPolar(tmp, polar1);
   
      Normalize3(result[0]);
      Normalize3(result[1]);
      Normalize3(result[2]);
      Normalize3(result[3]);

      toPolar(result[0], polar1);
      toPolar(result[1], polar2);
      toPolar(result[2], polar3);
      toPolar(result[3], polar4);
      //double phi1 = polar1[0];
      //double theta1 = polar1[1];
      double phi2 = polar2[0];
      double theta2 = polar2[1];
      double phi3 = polar3[0];
      double theta3 = polar3[1];
      //double phi4 = polar4[0];
      //double theta4 = polar4[1];

      cout << phi3 << " " << theta3 << " " << " 40" << endl;
      cout << phi2 << " " << theta2 << " " << " 40" << endl;
    //cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
    }
    for(j++; k > 0; k--) {

      buffer_point bptmp;
      bptmp.face = i + 1;
      bptmp.i = j;
      bptmp.j = k;

      coords_cube_for_print(bptmp, result, nrCells);
      //double* tmp = coords_cube_to_s2(bptmp, nrCells);
      double polar1[3];
      double polar2[3];
      double polar3[3];
      double polar4[3];
      //toPolar(tmp, polar1);
   
      Normalize3(result[0]);
      Normalize3(result[1]);
      Normalize3(result[2]);
      Normalize3(result[3]);

      toPolar(result[0], polar1);
      toPolar(result[1], polar2);
      toPolar(result[2], polar3);
      toPolar(result[3], polar4);
      double phi1 = polar1[0];
      double theta1 = polar1[1];
      double phi2 = polar2[0];
      double theta2 = polar2[1];
      //double phi3 = polar3[0];
      //double theta3 = polar3[1];
      //double phi4 = polar4[0];
      //double theta4 = polar4[1];

      cout << phi2 << " " << theta2 << " " <<  " 40" << endl;
      cout << phi1 << " " << theta1 << " " <<  " 40" << endl;
    //cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
    }

    cout << 0.0 << " " << 0.0 << " " <<  " 0" << endl;
  }
  cout << 0.0 << " " << 0.0 << " " <<  " 0" << endl;
}

void AccumulatorCube::printAccumulator() {
 
  //double norm[3];
  //double polar[3];
//  double *result;
  
  double **result;
  result = new double*[4];
  result[0] = new double[3]; 
  result[1] = new double[3]; 
  result[2] = new double[3]; 
  result[3] = new double[3]; 
  
  //buffer_point bp;

  /*
  for(double o = -0.9; o < 1.0; o+=0.1) {
    for(double p = -0.9; p < 1.0; p+=0.1) {
      norm[2] = o;
      norm[1] = p;
      norm[0] = 1.0;
      toPolar(norm, polar);
      
      bp = coords_s2_to_cell(norm, nrCells);
      result = coords_cube_to_s2(bp, nrCells);
      toPolar(result, polar);
      
      cout << polar[0] << " " << polar[1] << " " << 40 << endl;
      //if(fabs(polar[1]) < 0.0001) 
      //  cout << "STOP" << "-1.0 " << p << " " << o << endl;
      delete[] result;
    }
  }
  */

 /* 
  norm[0] = 1.0;  
  norm[1] = 0.0;  
  norm[2] = 0.0; 
  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  result = coords_cube_to_s2(bp, nrCells);
  toPolar(result, polar);
  cout << polar[0] << " " << polar[1] << endl << endl << endl;
  delete[] result;

  norm[0] = -1.0;  
  norm[1] = 0.0;  
  norm[2] = 0.0;

  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  result = coords_cube_to_s2(bp, nrCells);
  toPolar(result, polar);
  cout << polar[0] << " " << polar[1] << endl;
  cout << endl << endl;
  delete[] result;

  norm[0] = 0.0;  
  norm[1] = 1.0;  
  norm[2] = 0.0; 
  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  result = coords_cube_to_s2(bp, nrCells);
  toPolar(result, polar);
  cout << polar[0] << " " << polar[1] << endl;
  cout << endl << endl;
  delete[] result;

  norm[0] = 0.0;  
  norm[1] = -1.0;  
  norm[2] = 0.0; 
  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  result = coords_cube_to_s2(bp, nrCells);
  toPolar(result, polar);
  cout << polar[0] << " " << polar[1] << endl;
  cout << endl << endl;
  delete[] result;

  norm[0] = 0.0;  
  norm[1] = 0.0;  
  norm[2] = 1.0; 
  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  result = coords_cube_to_s2(bp, nrCells);
  toPolar(result, polar);
  cout << polar[0] << " " << polar[1] << endl;
  cout << endl << endl;
  delete[] result;

  norm[0] = 0.0;  
  norm[1] = 0.0;  
  norm[2] = -1.0; 
  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  result = coords_cube_to_s2(bp, nrCells);
  toPolar(result, polar);
  cout << polar[0] << " " << polar[1] << endl;
  cout << endl << endl;
  delete[] result;
*//*
  norm[0] = 0.0;  
  norm[1] = 0.0;  
  norm[2] = -1.0; 
  toPolar(norm, polar);
  cout << polar[0] << " " << polar[1] << endl;
  bp = coords_s2_to_cell(norm, nrCells);
  
  int i = bp.face - 1;
  int j = bp.i;
  int k = bp.j;
 */
  for(int i = 0; i < 6; i+=1) {
    cout << 0.0 << " " << 0.0 << " " << 0 << " 0" << endl;
    //for(int j = 1; j j <= nrCells j+= (nrCells-1)) {
    for(int j = 1; j <= nrCells; j++) {
      //cout << endl;
      //for(int k = 1; j k <= nrCells k+= (nrCells-1)) {
      for(int k = 1; k <= nrCells; k++) {
        //cout << endl;
        int rhosum = 0;
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          rhosum += accumulator[i][j-1][k-1][l];
        }
        buffer_point bptmp;
        bptmp.face = i + 1;
        bptmp.i = j;
        bptmp.j = k;

        coords_cube_for_print(bptmp, result, nrCells);
        //double* tmp = coords_cube_to_s2(bptmp, nrCells);
        double polar1[3];
        double polar2[3];
        double polar3[3];
        double polar4[3];
        //toPolar(tmp, polar1);
       
        Normalize3(result[0]);
        Normalize3(result[1]);
        Normalize3(result[2]);
        Normalize3(result[3]);
  /*     
        cout << result[0][0] << " " << result[0][1] << " " << result[0][2] << endl;
        cout << result[1][0] << " " << result[1][1] << " " << result[1][2] << endl;
        cout << result[2][0] << " " << result[2][1] << " " << result[2][2] << endl;
        cout << result[3][0] << " " << result[3][1] << " " << result[3][2] << endl;
        cout << result[0][0] << " " << result[0][1] << " " << result[0][2] << endl;
    */   
        toPolar(result[0], polar1);
        toPolar(result[1], polar2);
        toPolar(result[2], polar3);
        toPolar(result[3], polar4);
        double phi1 = polar1[0];
        double theta1 = polar1[1];
        double phi2 = polar2[0];
        double theta2 = polar2[1];
        double phi3 = polar3[0];
        double theta3 = polar3[1];
        double phi4 = polar4[0];
        double theta4 = polar4[1];

        /*
        cout << phi - 0.02 << " " << theta - 0.02 << " " << rhosum << endl;
        cout << phi + 0.02 << " " << theta - 0.02 << " " << rhosum << endl;
        cout << phi + 0.02 << " " << theta + 0.02 << " " << rhosum << endl;
        cout << phi - 0.02 << " " << theta + 0.02 << " " << rhosum << endl;
        */

        /*
        cout << (j+1+i*nrCells) << " " << k+1 << " " << rhosum << endl;
        cout << (j+1+i*nrCells) << " " << k << " " << rhosum << endl;
        cout << (j+i*nrCells) << " " << k << " " << rhosum << endl;
        cout << (j+i*nrCells) << " " << k+1 << " " << rhosum << endl;
        */
       /* 
        cout << phi1 << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << phi2 << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << phi1 << " " << theta2 << " " << rhosum << " 40" << endl;
       */
       
      if(phi1 < M_PI/nrCells && fabs(phi1-phi2) < 0.001 && fabs(phi1-phi3) < 0.001 && fabs(phi1 - phi4) < 0.001) { 
      //if(phi1 < M_PI/nrCells && phi2 < M_PI/nrCells && phi3 < M_PI/nrCells && phi4 < M_PI/nrCells) {
        cout << phi1 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
        cout << 0.0 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
        cout << 0.0 << " " << 0.0 << " " << rhosum << " 40" << endl;
        cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;
        //cout << phi1 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
         
        cout << endl;
        //cout << endl; 

      } //else if(M_PI - phi1 < M_PI/nrCells && M_PI - phi2 < M_PI/nrCells && M_PI - phi3 < M_PI/nrCells && M_PI - phi4 < M_PI/nrCells){
        else if(M_PI - phi1 < M_PI/nrCells && fabs(phi1-phi2) < 0.001 && fabs(phi1-phi3) < 0.001 && fabs(phi1 - phi4) < 0.001) {
        cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;
        cout << phi2 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
        cout << M_PI << " " << 2*M_PI << " " << rhosum << " 40" << endl;
        cout << M_PI << " " << 0.0 << " " << rhosum << " 40" << endl;
        //cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;
        cout << endl;
        //<< endl; 
         
        
      } else if(fabs(phi1 - phi2) < 2.0 && fabs(theta2 - theta3) < 3.0 && fabs(theta4 - theta3) < 3.0) {
        cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
        cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
        //cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

        //cout << endl;
        cout << endl;
      } else if(fabs(phi1 - phi2) < 2.0) {
        if(fabs(theta2 - theta3) < 3.0) {
          if(theta3 < 1.0) {
            cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << 0.0 << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;

          //  cout << endl;
            cout << endl;
        
            cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

            //cout << endl;
            cout << endl;
          } else {
            cout << phi1 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << 2*M_PI << " " << rhosum << " 40" << endl;

           // cout << endl;
            cout << endl;
        
            cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << 0.0 << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << 0.0 << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

           // cout << endl;
            cout << endl;

          }
        } else if(theta3 < 1.0) {
            cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << 0.0 << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << 0.0 << " " << rhosum << " 40" << endl;

           // cout << endl;
            cout << endl;
        
            cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
          //  cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

           // cout << endl;
            cout << endl;
        } else {
            cout << phi1 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << 2*M_PI << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << 2*M_PI << " " << rhosum << " 40" << endl;

           // cout << endl;
            cout << endl;
        
            cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
            cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
            cout << phi3 << " " << 0.0 << " " << rhosum << " 40" << endl;
            cout << phi4 << " " << 0.0 << " " << rhosum << " 40" << endl;
           // cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

           // cout << endl;
            cout << endl;

        }
      } else if(phi1 < 1.0) {
        cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << 0.0 << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << 0.0 << " " << theta3 << " " << rhosum << " 40" << endl;
        cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
       // cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

       // cout << endl;
        cout << endl;

        cout << M_PI << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
        cout << M_PI << " " << theta4 << " " << rhosum << " 40" << endl;
       // cout << M_PI << " " << theta1 << " " << rhosum << " 40" << endl;

       // cout << endl;
        cout << endl;


      } else {
        cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << M_PI << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << M_PI << " " << theta3 << " " << rhosum << " 40" << endl;
        cout << phi4 << " " << theta4 << " " << rhosum << " 40" << endl;
       // cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;

       // cout << endl;
        cout << endl;

        cout << 0.0 << " " << theta1 << " " << rhosum << " 40" << endl;
        cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
        cout << phi3 << " " << theta3 << " " << rhosum << " 40" << endl;
        cout << 0.0 << " " << theta4 << " " << rhosum << " 40" << endl;
       // cout << 0.0 << " " << theta1 << " " << rhosum << " 40" << endl;

       // cout << endl;
        cout << endl;
      }
       
       // cout << endl;
       // cout << endl;
        //delete[] tmp;

      }
    }
    cout << endl;
  }
  
  for(int i = 0; i < 3; i++) {
    delete[] result[i];
  }
  delete[] result;

}

void AccumulatorCube::resetAccumulator() {
  count = 0;
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < nrCells; j++) {
      for(int k = 0; k < nrCells; k++) {
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          accumulator[i][j][k][l] = 0;
        }
      }
    }
  }
}

bool AccumulatorCube::accumulate(double theta, double phi, double rho) {
  count++;
  double n[3];
  n[0] = cos(theta)*sin(phi);
  n[1] = sin(theta)*sin(phi);
  n[2] = cos(phi);

  buffer_point bp = coords_s2_to_cell(n, nrCells);
  
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)(rho*((double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }

  accumulator[bp.face - 1][bp.i - 1][bp.j - 1][rhoindex]++;
  lastbp = bp;

  return accumulator[bp.face - 1][bp.i - 1][bp.j - 1][rhoindex] > (int)myConfigFileHough.Get_AccumulatorMax(); 
}

double* AccumulatorCube::getMax(double &rho, double &theta, double &phi) {
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)(rho*((double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }
 
  double* result;
  result = coords_cube_to_s2(lastbp, nrCells); 
  //cout << "N: " << result[0] << " " << result[1] << " " << result[2] << endl; 
  return result;
}

void AccumulatorCube::accumulate(Point p) {
  for(int i = 0; i < 6; i++) {
    for(int j = 1; j <= nrCells; j++) {
      for(int k = 1; k <= nrCells; k++) {
        buffer_point bptmp;
        bptmp.face = i + 1;
        bptmp.i = j;
        bptmp.j = k;

        double* n = coords_cube_to_s2(bptmp, nrCells);
        Normalize3(n);
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          double rho = (l + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
          double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
          if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
            accumulator[i][j-1][k-1][l]++;
          }

        }
        delete[] n;
      }
    }
  }
}

double* AccumulatorCube::accumulateRet(Point p) {
  // rho theta phi
  count++;
  double * angles = new double[3];
  for(int i = 0; i < 6; i++) {
    for(int j = 1; j <= nrCells; j++) {
      for(int k = 1; k <= nrCells; k++) {
        buffer_point bptmp;
        bptmp.face = i + 1;
        bptmp.i = j;
        bptmp.j = k;

        double* n = coords_cube_to_s2(bptmp, nrCells);
        Normalize3(n);
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          double rho = (l + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
          double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
          if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
            accumulator[i][j-1][k-1][l]++;
            if(((unsigned int)accumulator[i][j-1][k-1][l] > myConfigFileHough.Get_AccumulatorMax() 
            && (unsigned int)accumulator[i][j-1][k-1][l] >
            count*myConfigFileHough.Get_PlaneRatio()) ||
          (unsigned int)accumulator[i][j-1][k-1][l] > 10*myConfigFileHough.Get_AccumulatorMax() 
            ) {
              double polar[3];
              toPolar(n, polar);
              angles[0] = rho;
              angles[1] = polar[1];
              angles[2] = polar[0];
              return angles;
            }
          }

        }
        delete[] n;
      }
    }
  }
  angles[0] = -1.0;
  return angles;
}

int* AccumulatorCube::accumulateAPHT(Point p) {
  // rho theta phi
  int tmpMax = 0;
  int * angles = new int[4];
  for(int i = 0; i < 6; i++) {
    for(int j = 1; j <= nrCells; j++) {
      for(int k = 1; k <= nrCells; k++) {
        buffer_point bptmp;
        bptmp.face = i + 1;
        bptmp.i = j;
        bptmp.j = k;

        double* n = coords_cube_to_s2(bptmp, nrCells);
        Normalize3(n);
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          double rho = (l + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
          double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
          if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
            accumulator[i][j-1][k-1][l]++;
            if(accumulator[i][j-1][k-1][l] > tmpMax) {
              angles[0] = l;
              angles[1] = i;
              angles[2] = j;
              angles[3] = k;
              tmpMax = accumulator[i][j-1][k-1][l];
            }
          }

        }
        delete[] n;
      }
    }
  }
  return angles;
}

double* AccumulatorCube::getMax(int* cell) {
  buffer_point bp;
  bp.face = cell[2] + 1;
  bp.i = cell[3] + 1;
  bp.j = cell[4] + 1;

  
  double rho = (cell[1] + 0.5) *
  (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  double* polar = coords_cube_to_s2(bp, nrCells); 
  double* polar2 = new double[4];
  for(int i = 0; i < 3; i++) {
    polar2[i] = polar[i];
  }
  delete[] polar;
  polar2[3] = rho;
  return polar2;
}

//void AccumulatorCube::getMax() {
multiset<int*, maxcompare>* AccumulatorCube::getMax() {
  multiset<int*, maxcompare>* maxlist = new multiset<int*, maxcompare>();
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < nrCells; j++) {
      for(int k = 0; k < nrCells; k++) {
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum(); l++) {
          int* tmp = new int[5];
          tmp[0] = accumulator[i][j][k][l];
          tmp[1] = l;
          tmp[2] = i;
          tmp[3] = j;
          tmp[4] = k;
          maxlist->insert(tmp);
        }
      }
    }
  }
  return maxlist;
}

void AccumulatorCube::peakWindow(int size) {
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < nrCells - size; j++) {
      for(int k = 0; k < nrCells - size; k++) {
        for(unsigned int l = 0; l < myConfigFileHough.Get_RhoNum() - size; l++) {
          int max = 0;
          for(int ji = j; ji < j + size; ji++) {
            for(int ki = k; ki < k + size; ki++) {
              for(unsigned int li = l; li < l + size; li++) {
                if(accumulator[i][ji][ki][li] > max) {
                  max = accumulator[i][ji][ki][li];
                }
              }
            }
          }
          for(int ji = j; ji < j + size; ji++) {
            for(int ki = k; ki < k + size; ki++) {
              for(unsigned int li = l; li < l + size; li++) {
                if(accumulator[i][ji][ki][li] < max) {
                  accumulator[i][ji][ki][li] = 0;
                }
              }
            }
          }
        }
      }
    }
  }
}


/*** HERE BEGINS THE ACCUMULATORBALL IMPROVED ***/

AccumulatorBallI::AccumulatorBallI(ConfigFileHough myCfg) {
  count = 0;
  int countCells = 0;
  myConfigFileHough = myCfg;

  int counter = 0;
  ballNr = new int[myConfigFileHough.Get_PhiNum()];
  step = 180.0/myConfigFileHough.Get_PhiNum();

  //double MAX_A = M_PI * (2 * fabs(0 - cos(rad(90- step))) + sin(rad(90 - step)) * sin(rad(90 - step)) + 1 * 1); 

  double h_0 = cos(rad(90 - step));
  double MAX_A = (2.0 * M_PI * h_0) / myConfigFileHough.Get_ThetaNum();
  double h_top = MAX_A /(2.0 * M_PI);
  phi_top_rad = acos(1 - h_top);
  phi_top_deg = deg(phi_top_rad);
  step = (180.0 - 2.0 * phi_top_deg) / (double)(myConfigFileHough.Get_PhiNum() - 2.0);

  double phi;
  ballNr[counter] = 1;
  ballNr[myConfigFileHough.Get_PhiNum() - 1 - counter] = 1;
  counter++;

  for(phi = phi_top_deg; phi < 90; phi+=step) {
    double h_i = cos(rad(phi)) - cos(rad(phi + step));
    double a_i = 2 * M_PI * h_i;
    ballNr[counter] = a_i / MAX_A;
    ballNr[myConfigFileHough.Get_PhiNum() - 1 - counter] = a_i / MAX_A;
    //cout << counter << " " << ballNr[counter] << " " << a_i << " " << MAX_A << endl;
    //cout << myConfigFileHough.Get_PhiNum() - 1 - counter << " " << ballNr[myConfigFileHough.Get_PhiNum() - 1 - counter] << " " << a_i << " " << MAX_A << endl;
    counter++;
  }
  /*
  for(int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    cout << i << ": " << ballNr[i] << endl;
  }
  cout << "BallNR erzeugt" << endl; 
  */

  accumulator = new int**[myConfigFileHough.Get_RhoNum()];

  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    accumulator[i] = new int*[myConfigFileHough.Get_PhiNum()];
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      accumulator[i][j] = new int[ballNr[j]];
      for(int k = 0; k < ballNr[j]; k++) {
        accumulator[i][j][k] = 0;
        countCells++;
      }
    }
  }
  cout << "CountCells " << countCells << endl;
}

AccumulatorBallI::~AccumulatorBallI() {
  
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      delete[] accumulator[i][j];
    }
    delete[] accumulator[i];
    
  }
  delete[] accumulator;
  delete[] ballNr;
  
}  

void AccumulatorBallI::printAccumulator() {
//TODO
  int sum = 0;
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    sum += ballNr[i];
  }
  //cout << "Wir haben " << sum << "Zellen!" << endl;
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    for(int j = 0; j < ballNr[i]; j++) {
      double phi1 = i * M_PI / (myConfigFileHough.Get_PhiNum()*0.99999999);
      double phi2 = (i+1) * M_PI / (myConfigFileHough.Get_PhiNum()*0.999999999);
      double theta1 = j * 2*M_PI / ballNr[i];
      double theta2 = (j+1) * 2*M_PI / ballNr[i];
      if(theta2 > 2*M_PI) theta2 = 2*M_PI;
      if(phi2 > M_PI) {
        phi2 = M_PI;
      }
      int rhosum = 0;
      /*
      for(int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        rhosum += accumulator[k][i][j];
      }
      */
      rhosum = accumulator[33][i][j];
      cout << phi1 << " " << theta1 << " " << rhosum << " 40" << endl;
      cout << phi2 << " " << theta1 << " " << rhosum << " 40" << endl;
      cout << phi2 << " " << theta2 << " " << rhosum << " 40" << endl;
      cout << phi1 << " " << theta2 << " " << rhosum << " 40" << endl;
      cout << endl;
    }

    //cout << endl;
  }
}

void AccumulatorBallI::resetAccumulator() {

//TODO
  count = 0;
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      int bNr = ballNr[j];
      for(int k = 0; k < bNr; k++) {
        accumulator[i][j][k] = 0;
      }
    }
  }
}

bool AccumulatorBallI::accumulate(double theta, double phi, double rho) {
//TODO
  count++;
  int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex = (int)((rho*(double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }
  
  int phiindex = (int)(phi*(((double)myConfigFileHough.Get_PhiNum()*0.9999999999)/(M_PI)));
  int bNr = ballNr[phiindex];
  int thetaindex = (int)(theta*((bNr*0.9999999999)/(2*M_PI)));
  if(thetaindex >= bNr) {
    cout << "duet";
  }
  //thetaindex = thetaindex % ballNr[phiindex];
  accumulator[rhoindex][phiindex][thetaindex]++;
  return ((unsigned int)accumulator[rhoindex][phiindex][thetaindex] >= myConfigFileHough.Get_AccumulatorMax());
}

void AccumulatorBallI::accumulate(Point p) {
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = phi_top_rad + (i-0.5) * rad(step);
    for(int j = 0; j < ballNr[i]; j++) {
      double theta = (j+0.5) * 2*M_PI / ballNr[i];
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
      double n[3];
      if(i == 0) {
        n[0] = 0.0;
        n[1] = 0.0;
        n[2] = 1.0;
      } else if (i == myConfigFileHough.Get_RhoNum() - 1) {
        n[0] = 0.0;
        n[1] = 0.0;
        n[2] = -1.0;
      } else {
        n[0] = cos(theta)*sin(phi);
        n[1] = sin(theta)*sin(phi);
        n[2] = cos(phi);
        Normalize3(n);
      }
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * (double)myConfigFileHough.Get_RhoMax() / (double)myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
        }

      }
    }
  }

}
double* AccumulatorBallI::accumulateRet(Point p) {
  count++;
//TODO
  // rho theta phi
  double* angles = new double[3]; 
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.999999999);
    for(int j = 0; j < ballNr[i]; j++) {
      double theta = (j+0.5) * 2*M_PI / ballNr[i];
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
      double n[3];
      n[0] = cos(theta)*sin(phi);
      n[1] = sin(theta)*sin(phi);
      n[2] = cos(phi);
      Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
          if(((unsigned int)accumulator[k][i][j] >
          myConfigFileHough.Get_AccumulatorMax() && (unsigned
          int)accumulator[k][i][j] > count*myConfigFileHough.Get_PlaneRatio())  
          || (unsigned int)accumulator[k][i][j] > 10*myConfigFileHough.Get_AccumulatorMax() ) { 
            angles[0] = rho;
            angles[1] = theta;
            angles[2] = phi;
            return angles;
          }
        }

      }
    }
  }
  angles[0] = -1;
  return angles;
}

int* AccumulatorBallI::accumulateAPHT(Point p) {

//TODO
  // rho theta phi
  int tmpMax = 0;
  int* angles = new int[4]; 
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum(); i++) {
    double phi = (i+0.5) * M_PI / (myConfigFileHough.Get_PhiNum()*0.9999999);
    for(int j = 0; j < ballNr[i]; j++) {
      double theta = (j+0.5) * 2*M_PI / ballNr[i];
      if(theta > 2*M_PI) theta = 2*M_PI;
      if(phi > M_PI) {
        phi = M_PI;
      }
      double n[3];
      n[0] = cos(theta)*sin(phi);
      n[1] = sin(theta)*sin(phi);
      n[2] = cos(phi);
      Normalize3(n);
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum(); k++) {
        double rho = (k + 0.5) * myConfigFileHough.Get_RhoMax() / myConfigFileHough.Get_RhoNum(); 
        double distance = p.x * n[0] + p.y * n[1] + p.z * n[2];
        if(fabs(distance-rho) < myConfigFileHough.Get_MaxPointPlaneDist()) {
          accumulator[k][i][j]++;
          if(accumulator[k][i][j] > tmpMax) {
            angles[1] = k;
            angles[2] = j;
            angles[3] = i;
            angles[0] = tmpMax = accumulator[k][i][j];
          }
        }

      }
    }
  }
  return angles;
}

double* AccumulatorBallI::getMax(double &rho, double &theta, double &phi) {
//TODO was??
  
  unsigned int rhoindex = myConfigFileHough.Get_RhoNum() - 1;
  
  if(rho < myConfigFileHough.Get_RhoMax()) {
    rhoindex =
    (int)(rho*((double)myConfigFileHough.Get_RhoNum()/(double)myConfigFileHough.Get_RhoMax()));   
  }
  
  unsigned int phiindex = (int)(phi*(((double)myConfigFileHough.Get_PhiNum())/(M_PI)));
  if(phiindex == myConfigFileHough.Get_PhiNum()) phiindex--;
  int bNr = ballNr[phiindex];
  int thetaindex = (int)(theta*((bNr + 1)/(2*M_PI)));
  thetaindex = thetaindex % bNr;

  phi = (0.5 + phiindex) * (double)(M_PI / (double)myConfigFileHough.Get_PhiNum());
  theta = (0.5 + thetaindex) * (double)(2.0 * M_PI / bNr);
  rho = (rhoindex + 0.5) * (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  return polar2normal(theta, phi);
}
double* AccumulatorBallI::getMax(int* cell) {
  double * polar;
  if(cell[3] == 0) {  
            polar = new double[4];
            polar[0] = 0.0;
            polar[1] = 0.0;
            polar[2] = 1.0;
  } else if((unsigned int)cell[3] == myConfigFileHough.Get_PhiNum() - 1) {
            polar = new double[4];
            polar[0] = 0.0;
            polar[1] = 0.0;
            polar[2] = -1.0;
  } else {
            double phi = phi_top_rad + (-0.5 + cell[3]) * rad(step);
            double bNr = ballNr[cell[3]];
            double theta = (0.5 + cell[2]) * (double)(2.0 * M_PI / bNr);
            polar = polar2normal(theta, phi);
  }
  
  double rho = (cell[1] + 0.5) *
  (double)myConfigFileHough.Get_RhoMax()/(double)myConfigFileHough.Get_RhoNum();
  if(std::isnan(polar[0])) {
    cout << cell[1] << " " << cell[2] << " " << cell[3] << endl;
    //cout << theta <<" bla " << phi << endl;
  }
  polar[3] = rho;
  return polar;
}

multiset<int*, maxcompare>* AccumulatorBallI::getMax() {
//void AccumulatorBallI::getMax() {
  multiset<int*, maxcompare>* maxlist = new multiset<int*, maxcompare>();
  //for(unsigned int i = 0; i < 1; i++) {
  for(unsigned int i = 0; i < myConfigFileHough.Get_RhoNum(); i++) {
    for(unsigned int j = 0; j < myConfigFileHough.Get_PhiNum(); j++) {
      int bNr = ballNr[j];
      for(int k = 0; k < bNr; k++) {
        int * tmp = new int[4];
        tmp[0] = accumulator[i][j][k];
        tmp[1] = i;
        tmp[2] = k;
        tmp[3] = j;
        maxlist->insert(tmp);
        double * polar;
        double phi;
        double theta = 0.0;
        if(tmp[3] == 0) {  
            polar = new double[4];
            polar[0] = 0.0;
            polar[1] = 0.0;
            polar[2] = 1.0;
            phi = 0;
        } else if((unsigned int)tmp[3] == myConfigFileHough.Get_PhiNum() - 1) {
            polar = new double[4];
            polar[0] = 0.0;
            polar[1] = 0.0;
            polar[2] = -1.0;
            phi = M_PI;
        } else {
            phi = phi_top_rad + (-0.5 + tmp[3]) * rad(step);
            int bNr = ballNr[tmp[3]];
            theta = (0.5 + tmp[2]) * (double)(2.0 * M_PI / (double)bNr);
            polar = polar2normal(theta, phi);
        }
        //cout << polar[0] << " " << polar[1] << " " << polar[2] << " " << phi << " " << theta << endl;
      }
    }
  }
  cout << "Maxlist Size:" << maxlist->size() << endl;
  return maxlist;
}
void AccumulatorBallI::peakWindow(int size) {
  //cout << "Wir haben " << sum << "Zellen!" << endl;
  for(unsigned int i = 0; i < myConfigFileHough.Get_PhiNum() - size; i++) {
    //cout << "Peak " << i << endl;
    for(int j = 0; j < ballNr[i] - size; j++) {
      
      for(unsigned int k = 0; k < myConfigFileHough.Get_RhoNum() - size; k++) {
        int max = 0;
        for(unsigned int ii = i; (ii < (i + size)) && (ii < myConfigFileHough.Get_PhiNum()); ii++) {
          for(int ji = j; (ji < (j + size)) && (ji < ballNr[ii]); ji++) {
            for(unsigned int ki = k; (ki < (k + size)) && (ki < myConfigFileHough.Get_RhoNum()); ki++) {
              if(accumulator[ki][ii][ji] > max) {
                max = accumulator[ki][ii][ji];
              }
            }
          }
        }
        for(unsigned int ii = i; ii < i + size && ii < myConfigFileHough.Get_PhiNum(); ii++) {
          for(int ji = j; ji < j + size && ji < ballNr[ii]; ji++) {
            for(unsigned int ki = k; ki < k + size && ki < myConfigFileHough.Get_RhoNum(); ki++) {
              if(accumulator[ki][ii][ji] < max) {
                accumulator[ki][ii][ji] = 0;
              }
            }
          }
        }
      }
    }
  }
}

