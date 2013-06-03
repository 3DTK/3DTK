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

void calculateNormalsApxKNN(std::vector<Point> &normals,
                            vector<Point> &points,
                            int k,
                            const double _rPos[3],
                            double eps = 0.0);

void calculateNormalsAdaptiveApxKNN(std::vector<Point> &normals,
                                    vector<Point> &points,
                                    int kmin,
                                    int kmax,
                                    const double _rPos[3],
                                    double eps = 0.0);

void calculateNormalsKNN(std::vector<Point> &normals,
                         vector<Point> &points,
                         int k,
                         const double _rPos[3] );


void calculateNormalsAdaptiveKNN(vector<Point> &normals,
                                 vector<Point> &points,
                                 int kmin, int kmax,
                                 const double _rPos[3]);

#endif // __NORMALS_H__
