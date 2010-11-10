/**
  * @file Representation of the Accumulator Cube.
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

#ifndef __HSM3D_H__
#define __HSM3D_H__
#include <math.h>
/*!
  \brief A point on the face of the cube with edge size 2
  and centered in 0,0,0
 */
struct cube_point {
  /*! Face of the cube. Must be between 1 and 6 */
  unsigned short int face;
  /*! First coordinate on the face. Must be between -1 and 1*/
  double u;
  /*! Second coordinate on the face. Must be between -1 and 1*/
  double v;
};

/*! 
  \brief Buffer representing a discretization of cube_point elements
  as patches on faces

  Discretizes the  the cube centered with edge size 2
  and centered in 0,0,0 using the same number of patches on each face.
  Resolution is not stored in the class.
 */
struct buffer_point {
  /*! Face of the cube. Must be between 1 and 6 */
  unsigned short int face;
  /*! First patch index */
  unsigned int i;
  /*! Second path index */
  unsigned int j;
};

bool real_compare(double x, double y); 

#endif
