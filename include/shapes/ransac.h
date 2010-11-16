#ifndef __RANSAC_H__
#define __RANSAC_H__

#include "shape.h"
#include "slam6d/scan.h"

// TODO implement some parameters to modify ransac (maybe in CollisionShape?)
template <class T>
void Ransac(CollisionShape<T> &shape, Scan *scan);


#endif
