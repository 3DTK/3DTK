/* (C) 2013-2016, The Regents of The University of Michigan
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address
above.

   GNU LGPL
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA */

#ifndef _IMAGE_U8X3_H
#define _IMAGE_U8X3_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct image_u8x3 image_u8x3_t;
struct image_u8x3
{
    const int width, height;
    const int stride; // bytes per line

    uint8_t *const buf; // const pointer, not buf
};

/////////////////////////////////////
// IMPORTANT NOTE ON BYTE ORDER
//
// Format conversion routines will (unless otherwise specified) assume
// R, G, B, ordering of bytes. This is consistent with GTK, PNM, etc.
//
/////////////////////////////////////

// Create or load an image. returns NULL on failure
image_u8x3_t *image_u8x3_create(unsigned int width, unsigned int height);
image_u8x3_t *image_u8x3_create_alignment(unsigned int width, unsigned int height, unsigned int alignment);
image_u8x3_t *image_u8x3_create_from_pnm(const char *path);

image_u8x3_t *image_u8x3_copy(const image_u8x3_t *in);

void image_u8x3_destroy(image_u8x3_t *im);
void image_u8x3_draw_line(image_u8x3_t *im, float x0, float y0, float x1, float y1, int v[3], int width);

int image_u8x3_write_pnm(const image_u8x3_t *im, const char *path);

#ifdef __cplusplus
}
#endif

#endif
