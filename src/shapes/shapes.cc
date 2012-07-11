/*
 * shapes implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "shapes/ransac_Boctree.h"
#include "shapes/NumberRecOctree.h"
#include "shapes/ransac.h"

int main() {
  CollisionPlane<double> plane(1.0); // 1.0 cm maxdist

  Ransac(plane,0);
  return 0;
}
