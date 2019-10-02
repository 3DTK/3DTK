#ifndef __TIMEMAP_H__
#define __TIMEMAP_H__

#include <iomanip>
#include <vector>
#include <sstream>
#include <iostream>

#include <ros/ros.h>

#include <map>


class timeMap {
  public:
    timeMap(std::vector<double> *timestamps) : firstTimestamp(0), lastTimestamp(0) {
        if (timestamps != NULL && timestamps[0].size() > 0) {
            firstTimestamp = timestamps[0][0];
            lastTimestamp = timestamps[0][timestamps[0].size() - 1];
        }
    }

  public:
    virtual ros::Time toRos(double riegltime) = 0;
    virtual void setOffset(double tdiff) = 0;

  public:
    double firstTimestamp;
    double lastTimestamp;
};

/**
 * simplest driftless model
 *
 *
 */
class timeMapSimple : public timeMap {
  protected:
    double a, b;
    double offset;

  public:
    timeMapSimple(std::vector<double> *timestamps) : timeMap(timestamps) {
      double riegltime = timestamps[0][0];
      double rost = timestamps[1][0];

      a = 1.0;
      b = rost - riegltime;
      offset = 0.0;
    }

    ros::Time toRos(double riegltime) {
      return ros::Time(a * riegltime + b + offset);
    }

    void setOffset(double tdiff) {
      offset = tdiff;
    }
};

/**
 * simple model with linear drift
 */
class timeMapLinear : public timeMapSimple {

  public:
    timeMapLinear(std::vector<double> *timestamps ) : timeMapSimple(timestamps) {
//      double rosstart = timestamps[1][0];

      int n = timestamps[0].size();

      double sum1, sum2, xy, xx;
      sum1 = sum2 = xx = xy = 0;

      for (int i = 0; i < n; i++) {
        sum1 += timestamps[0][i];
        sum2 += timestamps[1][i];

        xx += timestamps[0][i]*timestamps[0][i];
        xy += timestamps[0][i]*timestamps[1][i];
      }

      double C1 = sum1/(double)n; // centroid

      a = (xy - sum2 * C1) / (xx - n * C1*C1);
      a = 1.0;
      b = sum2/((double)n) - a*C1;
      b = b;// + rosstart;

//      cerr << std::setprecision(15) << endl;
//      cerr << "LINEAR PARAMS ARE " << a  << " " << (b - rosstart + timestamps[0][0]) << endl;
    }
};

/**
 * simple driftless model as estimated by Olsen
 */
class timeMapLinearOlsen : public timeMapSimple {

  public:
    timeMapLinearOlsen(std::vector<double> *timestamps) : timeMapSimple(timestamps) {
      int n = timestamps[0].size();

      double max = timestamps[0][0] - timestamps[1][0];
      double diff;
      for (int i = 0; i < n; i++) {
        diff = timestamps[0][i] - timestamps[1][i];
        if (max < diff) {
          max = diff;
        }
      }
      a = 1.0;
      b = -max;
//      cerr << "OLSEN OFFSET IS " << timestamps[1][0] + max - timestamps[0][0]<< endl;
    }
};



/**
 * simplest driftless model
 *
 *
 */
class timeMapOlsen : public timeMap {
  double offset;

  std::vector<double> P;    //riegl
  std::vector<double> Q;    //ros
  std::vector<double> A;    //offset

  double maxp, minp;
  double lastp;
  int lasti;
  double maxdiff;

  double alpha1;
  double alpha2;

  inline double maxdrift(double delta) {
//    return 0;
    if (delta > 0) {
      return alpha2*delta/(1-alpha2);
    } else if(delta < 0) {
      return alpha1*delta/(1+alpha1);
    } else {
      return 0.000;
    }
  }

  inline void find(double riegltime) {
    lasti = ((riegltime-minp) / (maxp - minp) * P.size() );
    if ( lasti >= (int)P.size() )  lasti = P.size() -1;
    if ( lasti < 0) lasti = 0;

//    cout << "FIND!!!! " << lasti << endl;
  }


  public:
    timeMapOlsen(std::vector<double> *timestamps, double alpha=0.000138888888) : timeMap(timestamps) {
      offset = 0.0;
      alpha1 = alpha2 = alpha;

      unsigned int n = timestamps[0].size();


      P.resize(n);
      Q.resize(n);
      A.resize(n);

      maxp = minp = timestamps[0][0];
      lasti = 0;
      maxdiff = timestamps[0][1] - timestamps[0][0];


      for (unsigned int i = 0; i < n; i++) {
        P[i] = timestamps[0][i];
        Q[i] = timestamps[1][i];
        A[i] = 0.0;

        if (minp > timestamps[0][i]) { minp = timestamps[0][i]; }
        if (maxp < timestamps[0][i]) { maxp = timestamps[0][i]; }
        if (i > 0 && maxdiff < timestamps[0][i] - timestamps[0][i-1]) {
          maxdiff = timestamps[0][i] - timestamps[0][i-1];
        }

      }
//      maxdiff = 2* (maxp-minp)/P.size();

      double p = P[0];
      double q = Q[0];

      for (unsigned int i = 0; i < n; i++) {

        if (  (P[i] - Q[i] - maxdrift(0)) >= (p - q - maxdrift(P[i]-p)) ) {
          p = P[i]; q = Q[i];
          A[i] = P[i] - Q[i] - maxdrift(0);
        } else {
          A[i] = (p - q - maxdrift(P[i]-p));
        }
      }

      for (int i = n-1; i >= 0; i--) {
        if (  (P[i] - Q[i] - maxdrift(0)) >= (p - q - maxdrift(P[i]-p)) ) {
          p = P[i]; q = Q[i];
          if (A[i] < P[i] - Q[i] -maxdrift(0)) {
            A[i] = P[i] - Q[i] - maxdrift(0);
          }
        } else {
          if ( A[i] < (p - q - maxdrift(P[i]-p)) ) {
            A[i] = (p - q - maxdrift(P[i]-p));
          }
        }
      }

//      cout << std::setprecision(15) << endl;
      for (unsigned int i = 0; i < n; i++) {
        A[i] = P[i] - A[i];
//        cout << P[i]-P[0] << " " << Q[i]-Q[0] << " " << A[i]-Q[0] << endl;
//        cout << P[i] << " " << Q[i] << " " << A[i] << endl;
      }


    }

    ros::Time toRos(double riegltime) {
      if ( fabs(riegltime - P[lasti]) > maxdiff) {
        find(riegltime);
      }

      if (P[lasti] > riegltime) {  // search backwards
        while (true) {
          if (lasti == 0) return ros::Time( A[lasti]  + offset );

          if (P[lasti] <= riegltime) break;

          lasti--;
        }
      } else {                  // default case
        while (true) {
          if (lasti == (int)(P.size()-1)) return ros::Time( A[lasti]  + offset );

          if (P[lasti +1] > riegltime) break;

          lasti++;
        }
      }

      double ros2   = A[lasti+1];
      double riegl2 = P[lasti+1];
      double ros1   = A[lasti];
      double riegl1 = P[lasti];

      // P[lasti+1] > riegltime, P[lasti] <= riegltime
//cout << "A " << lasti << " " << P[lasti] << " " << A[lasti] <<  " " << riegltime << " | ";
//cout << (riegltime - riegl1) << " / " << (riegl2 - riegl1) << " * " << (ros2-ros1) << " + " <<ros1 << " ||| ";
      return ros::Time(((riegltime - riegl1)/ (riegl2 - riegl1)) * (ros2-ros1) + ros1 + offset);
    }

    void setOffset(double tdiff) {
      offset = tdiff;
    }
};


#endif
