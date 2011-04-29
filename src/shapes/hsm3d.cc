/**
  * Representation of the Accumulator Cube.
  * 
  * This file contains the functions necessary to perform the transformations
  * between normal vectors and cube coordinates. The representation is taken
  * from 
  * A. Censi and S. Carpin,
  * HSM3D: Feature-Less Global 6DOF Scan-Matching in the Hough/Radon Domain,
  * In Proceedings of the IEEE International Conference on Robotics and 
  * Automation, 2009.
  *
  * The code in this file is a modified version of the code given by the authors
  * on http://purl.org/censi/2008/hsm3d
  * 
  * We gratefully appreciate the work of the authors and thank them for making
  * the code publically available.
  *
  * @author Dorit Borrmann. Institute of Computer Science, University of Osnabrueck, Germany.
  */

/* 

  AUTORIGHTS
   Copyright (c) 2008 The Regents of the University of California.
   All Rights Reserved.

   Created by Stefano Carpin
   University of California, Merced, Robotics Lab - School of Engineering

   Permission to use, copy, modify, and distribute this software and its
   documentation for educational, research and non-profit purposes, without fee,
   and without a written agreement is hereby granted, provided that the above
   copyright notice, this paragraph and the following three paragraphs appear in
   all copies.

   This software program and documentation are copyrighted by The Regents of the
   University of California. The software program and documentation are supplied
   "as is", without any accompanying services from The Regents. The Regents does
   not warrant that the operation of the program will be uninterrupted or
   error-free. The end-user understands that the program was developed for
   research purposes and is advised not to rely exclusively on the program for
   any reason.

   IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
   DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
   LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
   EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
   SUCH DAMAGE. THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY
   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED
   HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO
   OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
   MODIFICATIONS.

 */
#ifndef __HSM3D_ICC__
#define __HSM3D_ICC__

#include "shapes/accumulator.h"

bool real_compare(double x, double y) {
    return fabs(x - y) < 0.000000001;
  }

  buffer_point AccumulatorCube::coords_s2_to_cell(double* s, unsigned int width) {

    cube_point cubi;

    double s_norm =sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
    s[0] = s[0] / s_norm;
    s[1] = s[1] / s_norm;
    s[2] = s[2] / s_norm;

    double sa[3];
    sa[0] = fabs(s[0]);
    sa[1] = fabs(s[1]);
    sa[2] = fabs(s[2]);

    if ( ( sa[0] >= sa[1] ) && ( sa[0] >= sa[2] ) ) {
      cubi.face = s[0] > 0 ? 1 : 4;
      cubi.u = s[1] / s[0];
      cubi.v = s[2] / s[0];
    }

      else if ( ( sa[1] >= sa[0] ) && ( sa[1] >= sa[2] ) ) {
      cubi.face = s[1] > 0 ? 2 : 5;
      cubi.u = s[0] / s[1];
      cubi.v = s[2] / s[1];
    }

    else if ( ( sa[2] >= sa[1] ) && ( sa[2] >= sa[0] ) ) {
      cubi.face = s[2] > 0 ? 3 : 6;
      cubi.u = s[0] / s[2];
      cubi.v = s[1] / s[2];
    } else {
      cubi.face = 0;
      cubi.u = 0;
      cubi.v = 0;
    }

    buffer_point retval;

    double u = (cubi.u + 1)/2;
    double v = (cubi.v + 1)/2;

    unsigned int n = width;

    retval.face = cubi.face;
    
    retval.i = real_compare(u,1) ? n : 1 + floor(u*n);
    retval.j = real_compare(v,1) ? n : 1 + floor(v*n);

    return retval;

  }

  double* AccumulatorCube::coords_cube_to_s2(buffer_point src, unsigned int width) {

    cube_point retval;
    retval.face = src.face;
    retval.u = (src.i - 0.5 ) / (static_cast<double>(width));
    retval.u = retval.u*2 - 1;

    retval.v = (src.j - 0.5 ) / (static_cast<double>(width));
    retval.v = retval.v*2 - 1;

    double *result = new double[3];

    switch ( retval.face ) {
      case 1:
        result[0] = 1;
        result[1] = retval.u;
        result[2] = retval.v;
        break;
      case 4:
        result[0] = -1;
        result[1] = -retval.u;
        result[2] = -retval.v;
        break;
      case 2:
        result[0] = retval.u;
        result[1] = 1;
        result[2] = retval.v;
        break;
      case 5:
        result[0] = -retval.u;
        result[1] = -1;
        result[2] = -retval.v;
        break;
      case 3:
        result[0] = retval.u;
        result[1] = retval.v;
        result[2] = 1;
        break;
      case 6:
        result[0] = -retval.u;
        result[1] = -retval.v;
        result[2] = -1;
        break;
    }

    double norm = sqrt(result[0]*result[0] + result[1]*result[1] + result[2]*result[2]);
    result[0] /= norm;
    result[1] /= norm;
    result[2] /= norm;

    return result;
  }

  void AccumulatorCube::coords_cube_for_print(buffer_point src, double** result, unsigned int width) {
    /*
    assert((src.face >=1 ) && (src.face <=6 ));
    assert((src.i >= 1 ) && ( src.i <= width));
    assert((src.j >= 1 ) && ( src.j <= width));
    */

    cube_point retval1;
    cube_point retval2;
    cube_point retval3;
    cube_point retval4;
    retval1.face = src.face;
    retval2.face = src.face;
    retval3.face = src.face;
    retval4.face = src.face;

    retval1.u = (src.i - 1.0 ) / (static_cast<double>(width));
    retval2.u = (src.i - 1.0 ) / (static_cast<double>(width));
    retval1.u = retval1.u*2.0 - 1.0;
    retval2.u = retval2.u*2.0 - 1.0;

    retval1.v = (src.j - 1.0 ) / (static_cast<double>(width));
    retval2.v = (src.j) / (static_cast<double>(width));
    retval1.v = retval1.v*2 - 1;
    retval2.v = retval2.v*2 - 1;

    retval3.u = (src.i) / (static_cast<double>(width));
    retval4.u = (src.i) / (static_cast<double>(width));
    retval3.u = retval3.u*2.0 - 1.0;
    retval4.u = retval4.u*2.0 - 1.0;

    retval3.v = (src.j) / (static_cast<double>(width));
    retval4.v = (src.j - 1.0) / (static_cast<double>(width));
    retval3.v = retval3.v*2 - 1;
    retval4.v = retval4.v*2 - 1;
    /*
    assert((src.face >=1 ) && (src.face <=6 ));
    assert((src.u >= -1 ) && ( src.u <= 1));
    assert((src.v >= -1 ) && ( src.v <= 1));
    */

   // double *result = new double[2][3];

    switch ( retval1.face ) {
      case 1:
        result[0][0] = 1;
        result[0][1] = retval1.u;
        result[0][2] =  retval1.v;
        result[1][0] = 1;
        result[1][1] = retval2.u;
        result[1][2] =  retval2.v;
        result[2][0] = 1;
        result[2][1] = retval3.u;
        result[2][2] =  retval3.v;
        result[3][0] = 1;
        result[3][1] = retval4.u;
        result[3][2] =  retval4.v;
        break;
      case 4:
        result[0][0] = -1;
        result[0][1] = -retval1.u;
        result[0][2] = -retval1.v;
        result[1][0] = -1;
        result[1][1] = -retval2.u;
        result[1][2] = -retval2.v;
        result[2][0] = -1;
        result[2][1] = -retval3.u;
        result[2][2] = -retval3.v;
        result[3][0] = -1;
        result[3][1] = -retval4.u;
        result[3][2] = -retval4.v;
        break;
      case 2:
        result[0][0] = retval1.u;
        result[0][1] = 1;
        result[0][2] = retval1.v;
        result[1][0] = retval2.u;
        result[1][1] = 1;
        result[1][2] = retval2.v;
        result[2][0] = retval3.u;
        result[2][1] = 1;
        result[2][2] = retval3.v;
        result[3][0] = retval4.u;
        result[3][1] = 1;
        result[3][2] = retval4.v;
        break;
      case 5:
        result[0][0] = -retval1.u;
        result[0][1] = -1;
        result[0][2] = -retval1.v;
        result[1][0] = -retval2.u;
        result[1][1] = -1;
        result[1][2] = -retval2.v;
        result[2][0] = -retval3.u;
        result[2][1] = -1;
        result[2][2] = -retval3.v;
        result[3][0] = -retval4.u;
        result[3][1] = -1;
        result[3][2] = -retval4.v;
        break;
      case 3:
        result[0][0] = retval1.u;
        result[0][1] = retval1.v;
        result[0][2] = 1;
        result[1][0] = retval2.u;
        result[1][1] = retval2.v;
        result[1][2] = 1;
        result[2][0] = retval3.u;
        result[2][1] = retval3.v;
        result[2][2] = 1;
        result[3][0] = retval4.u;
        result[3][1] = retval4.v;
        result[3][2] = 1;
        break;
      case 6:
        result[0][0] = -retval1.u;
        result[0][1] = -retval1.v;
        result[0][2] = -1;
        result[1][0] = -retval2.u;
        result[1][1] = -retval2.v;
        result[1][2] = -1;
        result[2][0] = -retval3.u;
        result[2][1] = -retval3.v;
        result[2][2] = -1;
        result[3][0] = -retval4.u;
        result[3][1] = -retval4.v;
        result[3][2] = -1;
        break;
    }

  }

#endif
