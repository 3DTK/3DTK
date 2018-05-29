/* Mesh model object
* Author: Xia Sun
*/
#ifndef __MESH_H__
#define __MESH_H__

class Poisson {
public:
  // public attributes
  int writeAsXYZ();
  
  // public methods
  Poisson();
  ~Poisson();
  

private:
  // private attributes

  // private methods
  void initialize();
  
};

#endif