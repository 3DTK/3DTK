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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pnm.h"
#include "image_u8x3.h"

// least common multiple of 64 (sandy bridge cache line) and 48 (stride needed
// for 16byte-wide RGB processing). (It's possible that 48 would be enough).
#define DEFAULT_ALIGNMENT 192

image_u8x3_t *image_u8x3_create(unsigned int width, unsigned int height)
{
    return image_u8x3_create_alignment(width, height, DEFAULT_ALIGNMENT);
}

image_u8x3_t *image_u8x3_create_alignment(unsigned int width, unsigned int height, unsigned int alignment)
{
    int stride = 3*width;

    if ((stride % alignment) != 0)
        stride += alignment - (stride % alignment);

    uint8_t *buf = calloc(height*stride, sizeof(uint8_t));

    // const initializer
    image_u8x3_t tmp = { .width = width, .height = height, .stride = stride, .buf = buf };

    image_u8x3_t *im = calloc(1, sizeof(image_u8x3_t));
    memcpy(im, &tmp, sizeof(image_u8x3_t));
    return im;
}

image_u8x3_t *image_u8x3_copy(const image_u8x3_t *in)
{
    uint8_t *buf = malloc(in->height*in->stride*sizeof(uint8_t));
    memcpy(buf, in->buf, in->height*in->stride*sizeof(uint8_t));

    // const initializer
    image_u8x3_t tmp = { .width = in->width, .height = in->height, .stride = in->stride, .buf = buf };

    image_u8x3_t *copy = calloc(1, sizeof(image_u8x3_t));
    memcpy(copy, &tmp, sizeof(image_u8x3_t));
    return copy;
}

void image_u8x3_destroy(image_u8x3_t *im)
{
    if (!im)
        return;

    free(im->buf);
    free(im);
}

void image_u8x3_draw_line(image_u8x3_t *im, float x0, float y0, float x1, float y1, int v[3], int width)
{
    double dist = sqrtf((y1-y0)*(y1-y0) + (x1-x0)*(x1-x0));
    double delta = 0.5 / dist;

    // terrible line drawing code
    for (float f = 0; f <= 1; f += delta) {
        int x = ((int) (x0*f + x1*(1-f)));
        int y = ((int) (y0*f + y1*(1-f)));

        if (x < 0 || y < 0 || x >= im->width || y >= im->height)
            continue;

        int idx = y*im->stride + 3*x;
        im->buf[idx] = v[0];
        im->buf[idx+1] = v[1];
        im->buf[idx+2] = v[2];

        int r = y*im->stride + 3*(x+1);
        int d = (y+1)*im->stride + 3*x;
        int rd = (y+1)*im->stride + 3*(x+1);
        
        if (width > 1) {
            im->buf[r] = v[0];
            im->buf[r+1] = v[1];
            im->buf[r+2] = v[2];

            im->buf[d] = v[0];
            im->buf[d+1] = v[1];
            im->buf[d+2] = v[2];

            im->buf[rd] = v[0];
            im->buf[rd+1] = v[1];
            im->buf[rd+2] = v[2];
        }
    }
}

////////////////////////////////////////////////////////////
// PNM file i/o

// Create an RGB image from PNM
image_u8x3_t *image_u8x3_create_from_pnm(const char *path)
{
    pnm_t *pnm = pnm_create_from_file(path);
    if (pnm == NULL)
        return NULL;

    image_u8x3_t *im = NULL;

    switch (pnm->format) {
        case PNM_FORMAT_GRAY: {
            im = image_u8x3_create(pnm->width, pnm->height);

            for (int y = 0; y < im->height; y++) {
                for (int x = 0; x < im->width; x++) {
                    uint8_t gray = pnm->buf[y*im->width + x];
                    im->buf[y*im->stride + x*3 + 0] = gray;
                    im->buf[y*im->stride + x*3 + 1] = gray;
                    im->buf[y*im->stride + x*3 + 2] = gray;
                }
            }

            break;
        }

        case PNM_FORMAT_RGB: {
            im = image_u8x3_create(pnm->width, pnm->height);

            for (int y = 0; y < im->height; y++) {
                for (int x = 0; x < im->width; x++) {
                    uint8_t r = pnm->buf[y*im->width*3 + 3*x];
                    uint8_t g = pnm->buf[y*im->width*3 + 3*x+1];
                    uint8_t b = pnm->buf[y*im->width*3 + 3*x+2];

                    im->buf[y*im->stride + x*3 + 0] = r;
                    im->buf[y*im->stride + x*3 + 1] = g;
                    im->buf[y*im->stride + x*3 + 2] = b;
                }
            }

            break;
        }
    }

    pnm_destroy(pnm);
    return im;
}

int image_u8x3_write_pnm(const image_u8x3_t *im, const char *path)
{
    FILE *f = fopen(path, "wb");
    int res = 0;

    if (f == NULL) {
        res = -1;
        goto finish;
    }

    // Only outputs to RGB
    fprintf(f, "P6\n%d %d\n255\n", im->width, im->height);
    int linesz = im->width * 3;
    for (int y = 0; y < im->height; y++) {
        if (linesz != fwrite(&im->buf[y*im->stride], 1, linesz, f)) {
            res = -1;
            goto finish;
        }
    }

finish:
    if (f != NULL)
        fclose(f);

    return res;
}

