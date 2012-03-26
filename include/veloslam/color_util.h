#ifndef _COLOR_UTIL
#define _COLOR_UTIL

#include "math_util.h"

static int ColorTableShot[8][3]=
{
//    {255,255,255},
    {255,0,122},
    {255,255,0},
    {255,0,255},
    {0,255,255},
    {0,0,255},
    {255,122,0},
 //   {255,0,0},
    {0,255,0},
    {139,0,139}

};

static int ColorTableCircle[16][3]=
{
    {255,255,255},
	{255,122,255},
    {255,255,0},
    {255,122,0},
    {255,0,255},
    {255,0,122},
    {0,255,255},
    {0,255,122},
    {0,0,255},
    {0,0,122},
    {255,0,0},
    {122,0,0},
    {0,122,0},
    {0,255,0},
    {139,0,139},
    {139,122,139}

};

#define JET_COLORS_LUT_SIZE 1024
static float jet_colors[JET_COLORS_LUT_SIZE][3];
static int jet_colors_initialized = 0;

static inline void color_util_rand_color(float f[4],
										 double alpha,
										 double min_intensity)
{
    f[3] = alpha;

again:
    f[0] = randf();
    f[1] = randf();
    f[2] = randf();

    float v = f[0] + f[1] + f[2];

    // reject colors that are too dark
    if (v < min_intensity)
        goto again;
}

/** Given an array of colors, a palette is created that linearly interpolates through all the colors. **/
static void color_util_build_color_table(double color_palette[][3],
										  int palette_size,
										  float lut[][3],
										  int lut_size)
{
    for (int idx = 0; idx < lut_size; idx++)
	{
        double znorm = ((double) idx) / lut_size;

        int color_index = (palette_size - 1) * znorm;
        double alpha = (palette_size - 1) * znorm - color_index;

        for (int i = 0; i < 3; i++)
		{
            lut[idx][i] = color_palette[color_index][i] * (1.0 - alpha) + color_palette[color_index+1][i]*alpha;
        }
    }
}


static void init_color_table_jet()
{
    double jet[][3] = {{ 0,   0,   1 },
                       { 0,  .5,  .5 },
                       { .8, .8,   0 },
                       { 1,   0,   0 }};

    color_util_build_color_table(
		jet,
		sizeof(jet)/(sizeof(double)*3),
		jet_colors,
		JET_COLORS_LUT_SIZE);

    jet_colors_initialized = 1;
}

static inline float *color_util_jet(double v)
{
    if (!jet_colors_initialized)
        init_color_table_jet();

    v = fmax(0, v);
    v = fmin(1, v);

    int idx = (JET_COLORS_LUT_SIZE - 1) * v;
    return jet_colors[idx];
}

#endif
