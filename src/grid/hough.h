/***************************************************************************
 *                            hough.h
 ***************************************************************************/
#ifndef _hough_h
#define _hough_h

/***************************************************************************
 * INCLUDED HEADERS
 ***************************************************************************/
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <sys/types.h>
#include <ctype.h>  

extern int  sht_init(int sht_resolution);
extern int  **SHT_alloc_histogram(int x_max, int y_max);
extern void SHT_free_histogram(int **histogram, int x_max);
extern int  **SHT_init_histogram(int **histogram, double x_max, double y_max);
extern int  SHT_get_hough_lines(int nr_pts, double *x, double *y,
                                int sht_resolution, double max_rho_d,
				int **sht_histogram, int depth,
				int *nr_line_pts, double *x_line_pts,
                                double *y_line_pts,
                                double xmin, double ymin, int unsorted); 
//                                int **sht_maxima_histogram);
extern int  print_hough_lines(int sht_nr_line_pts, 
                              double *sht_x_line_pts, 
                              double *sht_y_line_pts);
extern int  SHT_get_max_distance(void);
extern void SHT_set_max_distance(int max_distance, int drange = 721);
extern int  SHT_Print_histogram(FILE *fpr, 
                                int resolution, 
                                int **histogram, 
                                int average);
extern int  SHT_calc_line(int resolution, 
                          double max_rho_d, 
                          int **histogram, 
                          int i, int j, double *m, double *b);
int SHT_show_line(int resolution, double max_rho_d, 
                  int **histogram,int i, int j, 
                  double *x0, double *y0,double *x1, double *y1);

#endif /* _hough_h */


















