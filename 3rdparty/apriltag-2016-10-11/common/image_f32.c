/* (C) 2013-2016, The Regents of The University of Michigan
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address
above.

   BSD
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "image_f32.h"
#include "math_util.h"

image_f32_t *image_f32_create(int width, int height)
{
    image_f32_t *fim = (image_f32_t*) calloc(1, sizeof(image_f32_t));

    fim->width = width;
    fim->height = height;
    fim->stride = width; // XXX do better alignment

    fim->buf = calloc(fim->height * fim->stride, sizeof(float));

    return fim;
}

// scales by 1/255u
image_f32_t *image_f32_create_from_u8(const image_u8_t *im)
{
    image_f32_t *fim = image_f32_create(im->width, im->height);

    for (int y = 0; y < fim->height; y++)
        for (int x = 0; x < fim->width; x++)
            fim->buf[y*fim->stride + x] = im->buf[y*im->stride + x] / 255.0f;

    return fim;
}

void image_f32_destroy(image_f32_t *im)
{
    free(im->buf);
    free(im);
}

void image_f32_normalize(image_f32_t *im)
{
    float maxval = -INFINITY;
    float minval = INFINITY;

    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            maxval = max(maxval, im->buf[y*im->width+x]);
            minval = min(maxval, im->buf[y*im->width+x]);
        }
    }

    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            int idx = y*im->width+x;
            im->buf[idx] = (im->buf[idx] - minval) / (maxval-minval);
        }
    }
}

static void convolve(const float *x, float *y, int sz, const double *k, int ksz)
{
    assert((ksz&1)==1);

    for (int i = 0; i < ksz/2 && i < sz; i++)
        y[i] = x[i];

    for (int i = 0; i < sz - ksz; i++) {
        double acc = 0.0;

        for (int j = 0; j < ksz; j++)
            acc += k[j]*x[i+j];

        y[ksz/2 + i] = acc;
    }

    for (int i = sz - ksz + ksz/2; i < sz; i++)
        y[i] = x[i];
}

void image_f32_gaussian_blur(image_f32_t *im, double sigma, int ksz)
{
    assert((ksz & 1) == 1); // ksz must be odd.

    // build the kernel.
    double k[ksz];

    // for kernel of length 5:
    // dk[0] = f(-2), dk[1] = f(-1), dk[2] = f(0), dk[3] = f(1), dk[4] = f(2)
    for (int i = 0; i < ksz; i++) {
        int x = -ksz/2 + i;
        double v = exp(-.5*sq(x / sigma));
        k[i] = v;
    }

    // normalize
    double acc = 0;
    for (int i = 0; i < ksz; i++)
        acc += k[i];

    for (int i = 0; i < ksz; i++)
        k[i] /= acc;

    for (int y = 0; y < im->height; y++) {
        float x[im->stride];
        memcpy(x, &im->buf[y*im->stride], im->stride*sizeof(float));
        convolve(x, &im->buf[y*im->stride], im->width, k, ksz);
    }

    for (int x = 0; x < im->width; x++) {
        float xb[im->height];
        float yb[im->height];

        for (int y = 0; y < im->height; y++)
            xb[y] = im->buf[y*im->stride + x];

        convolve(xb, yb, im->height, k, ksz);

        for (int y = 0; y < im->height; y++)
            im->buf[y*im->stride + x] = yb[y];
    }
}
