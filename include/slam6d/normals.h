/*
 * normal definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __NORMALS_H__
#define __NORMALS_H__

#include <vector>
#include <slam6d/scan.h>

void calculateNormalsApxKNN(std::vector<Point> &normals,
                            const std::vector<Point> &points,
                            const int k,
                            const double _rPos[3],
                            const double eps = 0.0);

void calculateNormalsAdaptiveApxKNN(std::vector<Point> &normals,
                                    const std::vector<Point> &points,
                                    const int kmin,
                                    const int kmax,
                                    const double _rPos[3],
                                    const double eps = 0.0);

void calculateNormalsKNN(std::vector<Point> &normals,
                         const std::vector<Point> &points,
                         const int k,
                         const double _rPos[3] );


void calculateNormalsAdaptiveKNN(std::vector<Point> &normals,
                                 const std::vector<Point> &points,
                                 const int kmin,
						                     const int kmax,
                                 const double _rPos[3]);

void calculateNormal(std::vector<Point> temp, double *norm, double *eigen);

void flipNormals(std::vector<Point> &normals);
void flipNormalsUp(std::vector<Point> &normals);

#endif // __NORMALS_H__
