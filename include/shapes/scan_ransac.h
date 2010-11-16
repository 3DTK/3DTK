// =====================================================================================
// 
//       Filename:  scan_ransac.h
// 
//    Description:  
// 
//        Version:  1.0
//        Created:  09/27/2010 01:21:49 PM
//       Revision:  none
//       Compiler:  g++
// 
//         Author:  Jan Elseberg (), jelseber@uos.de
//        Company:  Universitaet Osnabrueck
// 
// =====================================================================================

#ifndef __SCAN_RANSAC_H__
#define __SCAN_RANSAC_H__


#include <pthread.h>
#include "numbers.h"

void *dummy(void *);

class NumberDetector {
  public:
    vector<const double *> *scan_points;

  NumberDetector(vector<const double *> *points, const char *number_topic, bool _visualize) : visualize(_visualize) {
    //number_publisher = n.advertise<sickday::StampedNumbers>(number_topic,  10);
    //marker_publisher = n.advertise<visualization_msgs::MarkerArray>("Marker_array", 10);

    scan_points = points;
/*
    sn.set_numbers_size(10);
    sn.set_board_p_size(10);
    sn.set_char_p_size(10);
    sn.set_angle_size(10);
*/
    for (unsigned int i = 0; i < 10; i++) {
      detectedNumbers[i] = numberConsensus(i);
      /*
      sn.numbers[i].header.frame_id = "/map";
      sn.board_p[i] = 0;
      sn.char_p[i] = 0;
      sn.angle[i] = 0;
      */
    }

    running = true;
    ransac_thread = new pthread_t();
    pthread_create( ransac_thread, NULL, dummy , (void*)this);
  }

  ~NumberDetector() {
    running = false;
    pthread_join(*ransac_thread, NULL);

  }

  void RANSAC(vector<const double *> &scan_points);
  private:
  bool FindNumber(vector<double *> &points, double plane[4]); 
  void printNumbers(); 

  pthread_t *ransac_thread;
  numberConsensus detectedNumbers[10];
//  sickday::StampedNumbers sn;

  bool visualize;
  bool running;
};





#endif
