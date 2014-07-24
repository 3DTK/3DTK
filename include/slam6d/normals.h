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
using std::vector;
#include <slam6d/scan.h>
#if (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#include <opencv/highgui.h>
#else
#include <opencv2/opencv.hpp>
#endif

void calculateNormalsApxKNN(vector<Point> &normals,
                            const vector<Point> &points,
                            const int k,
                            const double _rPos[3],
                            const double eps = 0.0);

void calculateNormalsAdaptiveApxKNN(vector<Point> &normals,
                                    const vector<Point> &points,
                                    const int kmin,
                                    const int kmax,
                                    const double _rPos[3],
                                    const double eps = 0.0);

void calculateNormalsKNN(vector<Point> &normals,
                         const vector<Point> &points,
                         const int k,
                         const double _rPos[3] );


void calculateNormalsAdaptiveKNN(vector<Point> &normals,
                                 const vector<Point> &points,
                                 const int kmin,
						   const int kmax,
                                 const double _rPos[3]);

#endif // __NORMALS_H__
