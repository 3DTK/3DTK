// =====================================================================================
// 
//       Filename:  numbers.h
// 
//    Description:  
// 
//        Version:  1.0
//        Created:  09/24/2010 02:03:48 PM
//       Revision:  none
//       Compiler:  g++
// 
//         Author:  Jan Elseberg (), jelseber@uos.de
//        Company:  Universitaet Osnabrueck
// 
// =====================================================================================

#ifndef __NUMBERS_H_
#define __NUMBERS_H_

class numberState {
public:
  numberState() : number(0), char_p(0), board_p(0) {
    for (int i = 0; i < 3; i++) {
      position[i] = 0.0; 
    }
  };
  numberState(int _number) : number(_number), char_p(0), board_p(0) {
    for (int i = 0; i < 3; i++) {
      position[i] = 0.0; 
    }
  };
  numberState(int _number, double _char_p, double _board_p, double _position[3]) 
    : number(_number), char_p(_char_p), board_p(_board_p)
  {
    for (int i = 0; i < 3; i++) {
      position[i] = _position[i];
    }
    for (int i = 0; i < 4; i++) {
      plane[i] = 0;
    }
  }
  numberState(int _number, double _char_p, double _board_p, double _position[3], double _plane[4]) 
    : number(_number), char_p(_char_p), board_p(_board_p)
  {
    for (int i = 0; i < 3; i++) {
      position[i] = _position[i];
    }
    for (int i = 0; i < 4; i++) {
      plane[i] = _plane[i];
    }
  }

  double dist(numberState &ns) {
    double x = ns.position[0] - position[0];
    double y = ns.position[1] - position[1];
    double z = ns.position[2] - position[2];

    return sqrt( x*x + y*y +z*z );
  }
  
  int number;
  double char_p;
  double board_p;
  double position[3];
  double plane[4];

};

inline ostream& operator<<(ostream& os, const numberState ns)
{
  os << "P(" << ns.number << ") = " << ns.char_p;
  os << "   P(" << ns.position[0] << ", " << ns.position[1] << ", " << ns.position[2] << ") = " << ns.board_p;
  return os;
}

class numberConsensus {
public:

  numberConsensus() {
    sum_board_probability = 0.0;
    sum_char_probability = 0.0;
    consensus = numberState();
  }

  numberConsensus(int number) {
    sum_board_probability = 0.0;
    sum_char_probability = 0.0;
    consensus = numberState(number);
  }
  
  void addObservation(int number, double cp, double bp, double pos[3], double plane[4]) {
    addObservation( numberState(number, cp, bp, pos, plane) );
  }
  
  bool isClose(double pos[3]) {
    return isClose( numberState(0, 0, 0, pos) );
  }
  
  bool isClose(numberState ns) {
    for (unsigned int i = 0; i < clusters.size(); i++)  {
      if (ccenters[i].dist(ns) < 100.0 ) {   // should be good enough
        return true;
      }
    }
    return false;
  }

  void addObservation(numberState ns) {
    if (ns.number != consensus.number) return;
    
//    observations.push_back(ns);
    sum_board_probability += ns.board_p;
    sum_char_probability += ns.char_p;

    if (clusters.empty()) {
      vector<numberState> vec;
      vec.push_back(ns);
      ccenters.push_back(ns);
      clusters.push_back(vec);

      updateConsensus(0);
      return;
    }
    
    for (unsigned int i = 0; i < clusters.size(); i++)  {
      if (ccenters[i].dist(ns) < 100.0 ) {   // should be good enough
        cout << "new point is close to previous cluster..." << endl;
        clusters[i].push_back(ns);
        updateConsensus(i);
        return;
      }
    }

    // new cluster
    vector<numberState> vec;
    vec.push_back(ns);
    ccenters.push_back(ns);
    clusters.push_back(vec);

    updateConsensus(clusters.size() -1);
    return;
  }

  void updateConsensus(int changed) {
    // First compute new representative for the changed cluster
    double pos[3]; pos[0] = pos[1] = pos[2] = 0.0;

    vector<numberState> changed_cluster = clusters[changed];
    for (unsigned int i = 0; i < changed_cluster.size(); i++) {
      for (int j = 0; j < 3; j++)
        pos[j] += changed_cluster[i].position[j];
    }
    for (int j = 0; j < 3; j++)
      pos[j] = pos[j] / changed_cluster.size();

    // set representative
    ccenters[changed] = numberState(consensus.number, 0, 0, pos ) ;

    if (clusters.size() == 1) {       // special case
      if (clusters[0].size() == 1) {  //
        cout << "first number found..." << endl;
        ccenters[0].char_p = clusters[0][0].char_p;
        ccenters[0].board_p = clusters[0][0].board_p;
        consensus = ccenters[0];
        return;      
      }
    }


    for (unsigned int i = 0; i < clusters.size(); i++) {
      double sum_bp = 0.0;
      double sum_cp = 0.0;
      for (unsigned int j = 0; j < clusters[i].size(); j++) {
        sum_bp += clusters[i][j].board_p;
        sum_cp += clusters[i][j].char_p;
      }
      ccenters[i].char_p = sum_cp / sum_char_probability;
      ccenters[i].board_p = sum_bp / sum_board_probability;
    }


    consensus = ccenters[0];
    for (unsigned int i = 1; i < ccenters.size(); i++) {
      if (consensus.char_p * consensus.board_p < ccenters[i].char_p * ccenters[i].board_p) {
        consensus = ccenters[i]; 
      }
    }

  }
/*
  void updateConsensus() {
    double pos[3]; pos[0] = pos[1] = pos[2] = 0.0;
    double plane[4];
    double bp = 0.0;
    double cp = 0.0;
    
    for (unsigned int i = 0; i < observations.size(); i++) {
      for (int j = 0; j < 3; j++)
        pos[j] += observations[i].board_p * observations[i].position[j];
      
      cp += observations[i].char_p;
      bp += observations[i].board_p;
    }

    for (int j = 0; j < 3; j++)
      pos[j] = pos[j] / observations.size();
    cp = cp / observations.size();
    bp = bp / observations.size();

    consensus = numberState(consensus.number, cp, bp, pos, plane);
  }
  */

  numberState getConsensus() {
    return consensus;
  }

private:
  numberState consensus;
//  vector<numberState> observations;

  vector< vector<numberState> > clusters;
  vector<numberState> ccenters;

  double sum_char_probability;
  double sum_board_probability;


};

#endif
