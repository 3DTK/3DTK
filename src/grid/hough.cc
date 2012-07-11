/*
 * 2D hough transform for line detection implementation
 *
 * Copyright (C) Hartmut Surmann
 *
 * Released under the GPL version 3.
 *
 */


/***************************************************************************
 *                            hough.c
 ***************************************************************************
 *  PROJECT         : 3D-LASER
 *  AUTHOR          : Hartmut Surmann
 ***************************************************************************
 *  DESCRIPTION
 *  Implements some basic hough functions to detect lines in laserscans
 *  It writes a GnuPlot file in the debug case.
 *  NOTE : This is only a test program, it is'nt intended to be the 
 *         definitive evaluation tool.
 ***************************************************************************
 *  SYNTAX          : hough [depth level]
 *  ARGUMENTS
 *      depth level : default is ScanAZ which means find all lines
                    : default datafile ist test20.dat
***************************************************************************/
/* COMPILE it with (it needs test20.dat)
  gcc -o hough -g -O2 -m486 -Wall hough.c -lm -DSTANDALONE
  gcc -o hough -pg -a -O2 -m486 -Wall hough.c -lm -DSTANDALONE
*/

#include "grid/hough.h"
#include <iostream>
/***************************************************************************
 * CONSTANTS & MACROS
 ***************************************************************************/

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


/***********************FOR HOUGH TRANSFORMATION***************************/
int DATA_RANGE = 725;
int ScanAZ     = 721;
#define SKIP             0
#define EPSILON          3.1 // absoluter abstand zwischen geradengleichung und wert
//#define DIST             600 // wert zum quadrat ab diesem Wert haben wir eine Luecke
#define RAD2DEG 57.29578   // umrechnungsfaktor bogenmass grad
//#define DIST             6.9 // ab diesem Wert haben wir eine Luecke
#define MIN(a,b)         ((a)<(b)?(a):(b))
#define MAX(a,b)         ((a)>(b)?(a):(b))
#define SQR(a)           ((a)*(a))          


/*********************** HOUGH TRANSFORMATION***************************/



/***************************************************************************
 * DATA STRUCTURE DEFINITIONS
 ***************************************************************************/

/***************************************************************************
 * PUBLIC GLOBAL VARIABLES
 ***************************************************************************/
double *sin_theta = NULL, *cos_theta = NULL;
int    max_trigo_size = 0;
int    **k_index;
double alpha = M_PI / ScanAZ;
int   MAX_DISTANCE =  750; // war vorher define jetzt von aussen manipulierbar

#ifdef STANDALONE

/***************************************************************************
 * PRIVATE FUNCTION DEFINITIONS
 ***************************************************************************/
/**
 *-------------------------------------------------------------------------*
 * NAME        : AddNumberToFileName
 * DESCRIPTION
 *   Add a number to a filename. The number will be inserted at the end of 
 * the name an before the last extension or '.'
 * PARAMETERS
 *  'src'      : The original file name.
 *  'dest'     : The resulting file name.
 *  'number'   : The number to add.
 * RESULT
 *  A pointer to the buffer in which is stored the resulting file name. 
 * Always equal to 'dest'
 *-------------------------------------------------------------------------
 */
char *AddNumberToFileName(char *src, char *dest, long number)
{
  long aux;

  /* Find the last '.' */
  aux = strlen(src);
  while ( (src[aux] != '.') && (aux >= 0))
    aux--;

  /* Adds the number */
  if (aux == -1)
    sprintf(dest, "%s%03ld", src,  number);
  else
    {
      strncpy(dest, src, aux);
      sprintf(dest + aux, "%03ld%s",number, src + aux);
    }

  return dest;
}

#else //STANDALONE
extern char *AddNumberToFileName(char *src, char *dest, long number); 
//extern unsigned long GetCurrentTimeInMilliSec(void);
//extern __inline__ unsigned long long int rdtsc();
#endif


/**************************************************************************/
/***********************START HOUGH TRANSFORMATION***************************/
/**************************************************************************/


#define dist_func(x1,x2,y1,y2) ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
/**
 *
 */
double distance_func(double x1,double x2,
		     double y1, double y2)
{
  double x,y;

  x = x1 - x2;
  y = y1 - y2;
  x = x * x;
  y = y * y;

  return (sqrt(x + y));
}
/**
 * compute sin(theta) table from 0 to resolution
 */
int calc_sin_theta(int resolution, double *sin_theta)
{
  int     j;
  double  resolution_d = (double)resolution;  /* (double) form of resolution */
  double  theta;

  for (j=0; j < resolution; j++) {
    theta = M_PI * (j/(resolution_d) - 0.5);
    sin_theta[j] = sin(theta);
  }
  return 0;
}
/**
 * compute cos(theta) table from 0 to resolution
 */
int calc_cos_theta(int resolution, double *cos_theta)
{
  int     j;
  double  resolution_d = (double)resolution;  /* (double) form of resolution */
  double  theta;

  for (j=0; j < resolution; j++) {
    theta = M_PI * (j/(resolution_d) - 0.5);
    cos_theta[j] = cos(theta);
  }
  return 0;
}
/**
 *
 */
void SHT_set_max_distance(int max_distance, int drange)
{
  MAX_DISTANCE = max_distance;
  DATA_RANGE   = drange + 2;
  ScanAZ       = drange;
}
/**
 *
 */
int SHT_get_max_distance(void)
{
  return(MAX_DISTANCE);
}
/**
 * Allocate space for the histogram.
 * histogram is an array of pointers to arrays.
 */
int **SHT_alloc_histogram(int x_max, int y_max)
{
  int x; 
  int **histogram;

  histogram = (int **)calloc(x_max, sizeof(int *));
  if (histogram == NULL) {
    fprintf(stderr, "malloc failed in alloc_histogram() in sht_line.c\n");
    exit(1);
  }
  for (x = 0; x < x_max; x++) {
    histogram[x] = (int *)calloc(y_max , sizeof(int));
    if (histogram[x] == NULL) {
	 fprintf(stderr, "malloc failed in alloc_histogram() in sht_line.c\n");
	 exit(1);
    }
  }
  return(histogram);
} 
/**
 *sin_theta and cos_theta must be global
 */
int sht_init(int sht_resolution)
{
  int i;
  
  if (max_trigo_size < sht_resolution) {
    sin_theta = (double *) realloc(sin_theta,sizeof(double)*sht_resolution);
    cos_theta = (double *) realloc(cos_theta,sizeof(double)*sht_resolution);
    max_trigo_size = sht_resolution;
  }
  
  calc_sin_theta(sht_resolution, sin_theta);
  calc_cos_theta(sht_resolution, cos_theta);
  
  k_index = (int **)malloc(sizeof(int *)*DATA_RANGE);
  for (i = 0; i < DATA_RANGE; i++) {
    k_index[i] = (int *)malloc(sizeof(int)*sht_resolution);
  }
  return 0;
}
/**
 *
 */
void SHT_init_histogram(int **histogram, int x_max, int y_max)
{
  int x, y;
  for (x=0; x < x_max; x++) {
    for (y=0; y < y_max; y++) {
      histogram[x][y] = 0;
    }
  }
} 
/**
 *
 */
void SHT_free_histogram(int **histogram, int x_max)
{
  int x,i;

  for (x = 0; x < x_max; x++) {
    free(histogram[x]);
  }
  free(histogram);
  for (i = 0; i < DATA_RANGE; i++) {
    free(k_index[i]);
  }
}  
/**
 * nr_pts are the number of scan points to be used (black points)
 * resolution is the resolution used over
 * e.g. resolution = 100
 * then we produce a 100 x 100 histogram which covers the
 * relevent area of rho-theta space.
 * histogram is stored as an array of pointers to arrays.
 */
int SHT_hough_transform(int nr_pts, double *x, double *y,
			int resolution, double max_rho_d, int **histogram,
			double xmin, double ymin)
{
    //wykobi::trigonometry_tables;
  int i, j, k;
  double rho;
  int res_half = resolution >> 1;
  double mult_factor = resolution/(2.0*max_rho_d);
  /* Find Hough transform of points. */
  for (i=0; i < nr_pts; i++) {
    if ((fabs(x[i]+xmin) < MAX_DISTANCE) && (fabs(y[i]+ymin) < MAX_DISTANCE)){ 
      for (j=0; j < resolution; j++) {
	   rho = cos_theta[j]*(double )x[i] + sin_theta[j]*(double )y[i];
        k = (int)(rho * mult_factor) + res_half;
	   if ((k < 0) || (k >= resolution))  
		printf("ERROR: %d %f %f %d %f %d %f %f %f\n",
			  i,x[i],y[i],j,rho,k,max_rho_d,xmin,ymin);
	   else {
          histogram[j][k]++;
          k_index[i][j] = k;
	   }  
      }
    }
  }
  return 0;
}   
/**
 * Find average value of histogram (only consider
 * non-zero entries in histogram).
 * Use this as a model of the background noise value
 * of the histogram.
 * When the largest histogram value is less than this
 * average, then we will have removed all maxima from
 * histogram.
 */
unsigned long SHT_Find_average_value_of_historgram(int resolution, int **histogram)
{
  unsigned long sum = 0, average = 0, count = 0;
  int i,j;
  
  for (i=0; i < resolution; i++) {
    for (j=0; j < resolution; j++) {
      if (histogram[i][j] > 0) {
	   sum += histogram[i][j];
	   count++;
      }
    }
  }
  if (count != 0) average = sum / count;
  return average;			    
}
/**
 * Find max of histogram
 */
unsigned long SHT_Find_max_value_of_historgram(int resolution, int **histogram,
					       int *imax, int *jmax)
{
  register int maximum = 0;
  register int *startptr;
  register int i,j;
  
  *imax=0; *jmax = 0;
  for (i=0; i < resolution; i++)
    for (j=0, startptr = histogram[i];j < resolution; startptr++,j++) {
      if (*startptr > maximum) {
	   maximum = *startptr;
	   *imax = i; *jmax = j;
      }
    }
  return maximum;			    
}
/**
 *
 */
int SHT_Print_histogram(FILE *fpr, int resolution, int **histogram, int average)
{
  int i, j;

  fprintf(fpr,"# %d\n", resolution);
  for (i=0; i < resolution; i++) {
    for (j=0; j < resolution; j++) {
      if (histogram[i][j] > 0) fprintf(fpr,"%d %d %d\n", i, j,histogram[i][j]);
    }
    fprintf(fpr,"\n");
  }
  return 0;
  
}
/**
 * The line is in the form:
 *     y = mx + b
 * and |m| <= 1.0
 */
int SHT_calc_line(int resolution, double max_rho_d, int **histogram,
			   int i, int j, double *m, double *b)
{
  double rho;
  double resolution_d = (double)resolution;
  int    ret_val = 0;

  rho = 2.0*max_rho_d * ( (((double )j) - resolution_d/2.0) / resolution_d );
  *m = -cos_theta[i]/sin_theta[i];

  if (fabs(*m) <= 1.0) {
    *b = rho/sin_theta[i];
    ret_val = 0;
  }
  else {
    *m = 1.0/ *m;
    *b = rho/cos_theta[i];
    ret_val = 1;
  }
  return ret_val;
}
/**
 *
 */
int SHT_show_line(int resolution, double max_rho_d, int **histogram,
			   int i, int j, double *x0, double *y0,
			   double *x1, double *y1)
{
  double m, b;
  int    go_over_x = 0;
  
  go_over_x = SHT_calc_line(resolution, max_rho_d, histogram, i, j, &m, &b);

  if (!go_over_x) {
    *x0 =  -150;   *x1 = 150;
    *y0 =  m * *x0+b; *y1 = m * *x1+b;
  }
  else {
    *y0 =  0.0;    *y1 = 300;
    *x0 =  m* *y0+b; *x1 = m* *y1+b;
  }

  return 0;
}



// hier weiter harti 17.08.2000 fprint ersetzen in x bzw y_line_pts

double SHT_find_line(int nr_pts, double *x, double *y,
		     int resolution, double max_rho_d, int **histogram,
		     int ipkt, int jpkt, int *marker,
		     double length_of_a_line,
		     int *nr_line_pts, 
                     double *x_line_pts, double *y_line_pts,
		     double xmin, double ymin)
{
  double m, b;
  int    count = 0;
  double x0,y0,xa,xe,ya,ye;
  double *lx = new double[DATA_RANGE];
  double *ly = new double[DATA_RANGE];
  int *index = new int[DATA_RANGE];
  int lastindex = 0;
  double *dist = new double[DATA_RANGE];
  double maxlinedist = 0.0, linedist = 0.0;
  int    go_over_x = 0;
  int    i, j;
  
  go_over_x  = SHT_calc_line(resolution, max_rho_d, histogram, ipkt, jpkt, &m, &b);
  /* Find Hough transform of points. */
  for (i=0; i < nr_pts; i++) {
    if ((fabs(x[i]+xmin) < MAX_DISTANCE) && (fabs(y[i]+ymin) < MAX_DISTANCE)){ 
      x0 =  (m*y[i]+b+0.5); // +0.5 because int truncate
      y0 =  (m*x[i]+b+0.5); // +0.5 because int truncate
      if ( ((marker[i] == 4) || (marker[i] == 0)) && (((!go_over_x) && (fabs(y[i] -y0)< EPSILON)) ||
						      (( go_over_x) && (fabs(x[i] -x0)< EPSILON)))) {
	// now we are on the line
	// lx contains all points which are on the line
	lx[count] = x[i]; ly[count] = y[i]; index[count]=i;
	// dist contains the differents between the points on the line
	if (count > 0)
	  dist[count] = dist_func(x[i],lx[count-1],
				  y[i],ly[count-1]);
	else
	  dist[count]=0;
	if (dist[count] > 2500 ) {
	  linedist = dist_func(lx[count-1],lx[lastindex],
			       ly[count-1], ly[lastindex]);
	  if (linedist > maxlinedist) maxlinedist = linedist;
	  // is the founded line long enough
	  if (linedist > length_of_a_line) {
	    // ok show the line
	    xa = (m*ly[lastindex]+b+0.5);
	    xe = (m*ly[count-1]+b+0.5);
	    ya = (m*lx[lastindex]+b+0.5);
	    ye = (m*lx[count-1]+b+0.5);
	    x_line_pts[*nr_line_pts]     = (go_over_x?xa:lx[lastindex]);
	    y_line_pts[(*nr_line_pts)++] = (go_over_x?ly[lastindex]:ya);
	    x_line_pts[*nr_line_pts]     = (go_over_x?xe:lx[count-1]);
	    y_line_pts[(*nr_line_pts)++] = (go_over_x?ly[count-1]:ye);
	    // mark all these points
	    for (j = lastindex; j <= count-1;j++) {
	      if (marker[index[j]] == 4) marker[index[j]] = 1;
	      else marker[index[j]] = 3;
	    }
	  }
	  lastindex=count;
	}
	count++;
      }
    }
  }
  linedist = dist_func(lx[count-1] ,lx[lastindex],
		       ly[count-1], ly[lastindex]);
  if (linedist > maxlinedist) maxlinedist = linedist;
  // is the founded line long enough
  if (linedist > length_of_a_line) {
    // ok show the line
    xa = (int)(m*ly[lastindex]+b+0.5);
    xe = (int)(m*ly[count-1]+b+0.5);
    ya = (int)(m*lx[lastindex]+b+0.5);
    ye = (int)(m*lx[count-1]+b+0.5);
    x_line_pts[*nr_line_pts]     = (go_over_x?xa:lx[lastindex]);
    y_line_pts[(*nr_line_pts)++] = (go_over_x?ly[lastindex]:ya);
    x_line_pts[*nr_line_pts]     = (go_over_x?xe:lx[count-1]);
    y_line_pts[(*nr_line_pts)++] = (go_over_x?ly[count-1]:ye);
    // mark all these points
    for (j = lastindex; j <= count-1;j++) marker[index[j]] = 1;
    if (count-1 > lastindex) {
      marker[index[lastindex]] = 3; /* anfangs und endpunkte extra markieren */
      marker[index[count-1]] = 3;
    }
  }

  delete[] lx;
  delete[] ly;
  delete[] index;
  delete[] dist;  

  return maxlinedist;
}

/**
 * falls wir nicht einen orginal scan haben sondern nur wild unsotierte Punkte
 * 1cm genauigkeit 2 cm luecke wird akzeptiert
 */
double SHT_find_unsorted_line(int nr_pts, double *x, double *y,
						int resolution, double max_rho_d, int **histogram,
						int ipkt, int jpkt, int *marker,
						double length_of_a_line,
						int *nr_line_pts, 
						double *x_line_pts, double *y_line_pts,
						double xmin, double ymin)
{
  double m, b;
  int    count = 0;
  double x0,y0;
  double maxlinedist = 0.0, linedist = 0.0;
  int    go_over_x = 0;
  int    i;
  int    maxlen_index = 0, minlen_index = 0, *line_pts, *li, startpkt = 0, luecke = 0, lasti= 0;
  double len = 0.0, maxlen = 0.0, minlen = 0;
  int    ref_index = -1;
  go_over_x  = SHT_calc_line(resolution, max_rho_d, histogram, ipkt, jpkt, &m, &b);
  /* Find Hough transform of points. */
  // first calc maximal len of all possible lines
  for (i=0; i < nr_pts; i++) {
    if ((fabs(x[i]+xmin) < MAX_DISTANCE) && (fabs(y[i]+ymin) < MAX_DISTANCE)){ 
      x0 =  (m*y[i]+b+0.5); // +0.5 because int truncate
      y0 =  (m*x[i]+b+0.5); // +0.5 because int truncate
      if ( ((marker[i] == 4) || (marker[i] == 0)) && (((!go_over_x) && (fabs(y[i] -y0)< EPSILON)) ||
										    (( go_over_x) && (fabs(x[i] -x0)< EPSILON)))) {
	   if (ref_index == -1) { // init
		ref_index = i;
		maxlen = minlen = 0.0;
		maxlen_index = minlen_index = i;
		len = 0.0;
	   }
	   else {
		len = distance_func(x[i], x[ref_index],
						y[i], y[ref_index]);
		if (go_over_x) {
		  if ((y[i] - y[ref_index]) < 0 ) {
		    len *= -1.0;
		    if (len < minlen) {
			 minlen = len;
			 minlen_index = i;
		    }
		  }
		  else {
		    if (len > maxlen) {
			 maxlen = len;
			 maxlen_index = i;
		    }
		    if (len < minlen) {
			 minlen = len;
			 minlen_index = i;
		    }
		  }
		}
		else { // go over y
		  if ((x[i] - x[ref_index]) < 0 ) {
		    len *= -1.0;
		    if (len < minlen) {
			 minlen = len;
			 minlen_index = i;
		    }
		    if (len > maxlen) {
			 maxlen = len;
			 maxlen_index = i;
		    }
		  }
		  else {
		    if (len < minlen) {
			 minlen = len;
			 minlen_index = i;
		    }
		    if (len > maxlen) {
			 maxlen = len;
			 maxlen_index = i;
		    }
		  }
		}
	   } // else loop
	   count++;
	 } // on the point
    } // in distance 
  } // loop
  maxlinedist = distance_func(x[maxlen_index], x[minlen_index],
						y[maxlen_index], y[minlen_index]);
  line_pts = (int *) calloc((int)(maxlinedist+200),sizeof(int));
  li = (int *) calloc((int)(2.0*maxlinedist+200),sizeof(int));
  // now calc historgramm of the line
  for (i=0; i < nr_pts; i++) {
    if ((fabs(x[i]+xmin) < MAX_DISTANCE) && (fabs(y[i]+ymin) < MAX_DISTANCE)){ 
      x0 =  (m*y[i]+b+0.5); // +0.5 because int truncate
      y0 =  (m*x[i]+b+0.5); // +0.5 because int truncate
      if ( ((marker[i] == 4) || (marker[i] == 0)) && (((!go_over_x) && (fabs(y[i] -y0)< EPSILON)) ||
						      (( go_over_x) && (fabs(x[i] -x0)< EPSILON)))) {
	   len = distance_func(x[i], x[minlen_index],
						y[i], y[minlen_index]);
	   if (len > (maxlinedist+0.5)) {
		printf("PROBLEM %d (%f,%f) %f %f\n",i,x[i],y[i],len,maxlinedist);
		maxlinedist = len;
	   }	   line_pts[(int)(len)]++; 
	   li[(int)(len)] = i; // index fuer den Punkt
	   marker[i] = 1; // nur einfache pkt
	 }
    }
  }
  // startpunkt
  startpkt = 0;
  linedist = 0.0;
  luecke = -1;
  lasti = 0;
  // and at last calc the sub lines
  for (i = 0; i <= (int)(maxlinedist); i++) {
    //printf("%d %d %d\n",i,line_pts[i],startpkt);
    if (line_pts[i] > 0) {
	 luecke = 0;
	 lasti = i;
    }
    else luecke++;
    // startpunkt sezten
    if ( (line_pts[i] > 0) && (startpkt == 0)) {
	 x_line_pts[*nr_line_pts]     = x[li[i]];
	 y_line_pts[(*nr_line_pts)++] = y[li[i]];
	 startpkt = 1;
    }
    // endpunkt
    else if ( ((line_pts[i] == 0) && (startpkt == 1) && (luecke > 3)) || (i == (int)(maxlinedist))) {
	 x_line_pts[*nr_line_pts]     = x[li[lasti]];
	 y_line_pts[(*nr_line_pts)++] = y[li[lasti]];
	 len = distance_func(x_line_pts[*nr_line_pts - 2], x_line_pts[*nr_line_pts - 1],
					 y_line_pts[*nr_line_pts - 2], y_line_pts[*nr_line_pts - 1]);
	 if (len > linedist) linedist = len;
	 startpkt = 0;
	 luecke = -1;
    }
  }
  if (startpkt == 1) {
    x_line_pts[*nr_line_pts]     = x[li[lasti]];
    y_line_pts[(*nr_line_pts)++] = y[li[lasti]];
  }
  free(line_pts);
  free(li);
  /* 
  printf("%d points are on the line, %d lines found\n",count,*nr_line_pts/2);
  fflush(stdout);
  */
  return linedist;
}
/**
 *
 */
int SHT_remove_line(int nr_pts, double *x, double *y,
		    int resolution, double max_rho_d, int **histogram,
		    int *marker, double xmin, double ymin)
{
  int    i,j;//, k;
  int nr_removed_points = 0;
  
  for (i=0; i < nr_pts; i++) {
    if ((marker[i] == 1) || (marker[i] == 3)) {
      marker[i] += 1; // from 1 to 2 or from 3 to 4 
      if ((fabs(x[i]+xmin) < MAX_DISTANCE)&&(fabs(y[i]+ymin) < MAX_DISTANCE)){ 
        nr_removed_points++;
	   for (j = 0; j < resolution; j++) {
		histogram[j][k_index[i][j]]--;
	   }
      }
    }
  }
  return nr_removed_points;
}
/**
 * -------------------------------------------------------------------------*
 * NAME        : SHT_get_hough_lines
 * DESCRIPTION
 *  Gets the lines in a laserscan
 * PARAMETERS
 *   ---------
 * RESULT
 *   number of looped iterations
 *-------------------------------------------------------------------------
 */
int SHT_get_hough_lines(int nr_pts, double *x, double *y,
			int sht_resolution, double max_rho_d,
			int **sht_histogram, int depth,
			int *nr_line_pts, 
                        double *x_line_pts, double *y_line_pts,
			double xmin, double ymin, int unsorted)
{
#ifdef STANDALONE
  //debug
  char   filename[256], auxfilename[256];
  unsigned long prev_clock;
  FILE   *sht_fpr_histogramm;
  
  strcpy(filename,"plane.dat");
  //prev_clock = GetCurrentTimeInMilliSec();
  unsigned long sht_average;
  sht_average = SHT_Find_average_value_of_historgram(sht_resolution, sht_histogram);
#endif
  
  // sht
  unsigned long sht_maximum;
  int           sht_imax, sht_jmax;
  int           *sht_marker = new int[ScanAZ];
  double        sht_length_of_a_line;
  int           i = 0;
  double        m,b;
  
  // init marker
  for (i = 0; i < nr_pts; i++) sht_marker[i] = 0;
  *nr_line_pts = 0;
  
  // hough
  SHT_hough_transform(nr_pts, x, y, sht_resolution, max_rho_d, sht_histogram,xmin,ymin);
  sht_maximum = SHT_Find_max_value_of_historgram(sht_resolution, sht_histogram,
						 &sht_imax, &sht_jmax);
  i = 0;
  while ((sht_maximum > 0) && (i < depth)) {
    sht_length_of_a_line = -0.1; 

#ifdef STANDALONE
    printf("\n====================================================%d %f\n",i,sht_length_of_a_line);
    //printf("%d %ld\n",i,GetCurrentTimeInMilliSec()-prev_clock);
    printf("Hough Transformation beendet. Aver: %ld Max %ld, at (%d,%d)\n",
    	 sht_average, sht_maximum, sht_imax,sht_jmax);
    fflush(stdout);
    // debug Kontroll output of the planes
    AddNumberToFileName(filename, auxfilename, i);  
    sht_fpr_histogramm = fopen(auxfilename,"w");
    SHT_Print_histogram(sht_fpr_histogramm, sht_resolution, sht_histogram, sht_average);
    fclose(sht_fpr_histogramm);
#endif
    SHT_calc_line(sht_resolution, max_rho_d, sht_histogram, sht_imax, sht_jmax, &m, &b);
    if (unsorted) {
	 SHT_find_unsorted_line(nr_pts, x, y, sht_resolution, max_rho_d, sht_histogram,
									  sht_imax, sht_jmax, sht_marker, sht_length_of_a_line,
									  nr_line_pts, x_line_pts, y_line_pts, xmin,ymin);
    }
    else {
	 SHT_find_line(nr_pts, x, y, sht_resolution, max_rho_d, sht_histogram,
								 sht_imax, sht_jmax, sht_marker, sht_length_of_a_line,
								 nr_line_pts, x_line_pts, y_line_pts, xmin,ymin);
    }
    SHT_remove_line(nr_pts, x, y, sht_resolution, max_rho_d,
								sht_histogram, sht_marker,xmin,ymin);
    sht_maximum = SHT_Find_max_value_of_historgram(sht_resolution, sht_histogram,
						                     &sht_imax, &sht_jmax);
    i++;
  }
  delete[] sht_marker;
  return i;
}
/**
 * Kontroll Ausgabe
 */
int print_hough_lines(int sht_nr_line_pts, int *sht_x_line_pts, int *sht_y_line_pts)
{
  int i;
  double dist;
  static FILE *fpr = NULL;
  
  if (fpr == NULL)
    fpr = fopen("hough-lines.dat","w");
  printf("# found %d lines\n",(int)(sht_nr_line_pts*0.5));
  fprintf(fpr,"# found %d lines\n",(int)(sht_nr_line_pts*0.5));
  for (i = 0; i < sht_nr_line_pts; i+=2) {
    dist = distance_func((double)(sht_x_line_pts[i]),(double)(sht_x_line_pts[i+1]),
			 (double)(sht_y_line_pts[i]),(double)(sht_y_line_pts[i+1]));
    // 1 pkt linien nicht anzeigen
    // if (dist > 0.0) {
    if (dist > -0.1) {
      fprintf(fpr,"#%d %f\n",i/2,dist);
      fprintf(fpr,"%d %d\n",sht_x_line_pts[i],sht_y_line_pts[i]);
      fprintf(fpr,"%d %d\n\n",sht_x_line_pts[i+1],sht_y_line_pts[i+1]);
    }
  }
  //fclose(fpr);

  return 0;
}

/**************************************************************************/
/***********************END HOUGH TRANSFORMATION***************************/
/**************************************************************************/

#ifdef STANDALONE_OLD
int main(int argc, char ** argv)
{
  //debug and init, help
  FILE *fpr; // *sht_fpr_gerade
  int i, NrPts;
  char buffer[256], *ptr = buffer;
  int dist[ScanAZ], angle[ScanAZ];
  double amplitude[DATA_RANGE];
  int x[DATA_RANGE];
  int y[DATA_RANGE];
  // double h1,h2,h3,h4;
  int h1=1,h2=1,h3,h4;
  double scale_val = 1.0;

  // hough
  int    **sht_histogram; // histogramm der hough
  int    sht_resolution = 200;  // aufloesung der hough
  int    sht_nr=ScanAZ;   // ScanAz heisst alle; Anzahl der Durchlaeufe durch die Hough
  // Speicherstruktur Anzahl der Linienpunkte (d.h) Nr_Linien = sht_nr_line_pts/2,
  // x,y, 0=Anfang, 1=Ende, ....
  int    sht_nr_line_pts=0, sht_x_line_pts[2*ScanAZ], sht_y_line_pts[2*ScanAZ];
  int    xmin = 2000, xmax = -2000, ymin = 2000, ymax = -2000;
  double max_rho_d = 0.0,local_dist=0.0;

  if (argc == 2)
    sht_nr = atoi(argv[1]);
  else sht_nr = 100;
  
  // READ input files
  fpr = fopen("test20.dat","r");
  // erste zeile ueberlesen
  while ((*ptr++ = fgetc(fpr)) != 10) ;
  NrPts = atoi(&(buffer[2]));
  printf("NrPts: %d\n",NrPts);
  for (i = 0; i < NrPts; i++) {
    fscanf(fpr,"%d %d",&h3,&h4);
    angle[i] = (int)(h1);
    dist[i] = (int)h2;
    amplitude[i] = h2;
    if (i >= SKIP) {
      x[i-SKIP] = (int)(h3 * scale_val);
      y[i-SKIP] =(int)(h4 * scale_val);
      if (x[i-SKIP] > xmax) xmax =  x[i-SKIP];
      if (x[i-SKIP] < xmin) xmin =  x[i-SKIP];
      if (y[i-SKIP] > ymax) ymax =  y[i-SKIP];
      if (y[i-SKIP] < ymin) ymin =  y[i-SKIP];
      local_dist = sqrt((x[i-SKIP] - x[i-SKIP-1])* (x[i-SKIP] - x[i-SKIP-1]) +
			(y[i-SKIP] - y[i-SKIP-1])* (y[i-SKIP] - y[i-SKIP-1]));
    }
  }
  fclose(fpr);
  max_rho_d = sqrt(SQR(xmax-xmin) + SQR(ymax-ymin));
  printf("Maxima:\n%d %d %d %d %f\n",xmin,xmax,ymin,ymax,max_rho_d);
  if (xmax > MAX_DISTANCE) xmax = MAX_DISTANCE;
  if (ymax > MAX_DISTANCE) ymax = MAX_DISTANCE;
  if (xmin < -MAX_DISTANCE) xmin = -MAX_DISTANCE;
  if (ymin < -MAX_DISTANCE) ymin = -MAX_DISTANCE;
  max_rho_d = sqrt(SQR(xmax-xmin) + SQR(ymax-ymin));
  printf("New Maxima:\n%d %d %d %d %f\n",xmin,xmax,ymin,ymax,max_rho_d);
  for (i = 0; i < NrPts - 2*SKIP; i++) {
    x[i] -= xmin; 
    y[i] -= ymin;
  }
  // START with the hough
  // init block
  /* Andreas 29.9.2000
  if (max_rho_d > 500) sht_resolution = (int)(max_rho_d * 0.5); 
  else sht_resolution = (int)(max_rho_d * 0.75);
  */

  sht_resolution = (int)(max_rho_d * 0.75);
  
  printf("Hough Transformation berechnen mit Aufloesung %d\n",sht_resolution);fflush(stdout);
  sht_init(sht_resolution);
  sht_histogram = SHT_alloc_histogram(sht_resolution, sht_resolution);
  // end init


  SHT_get_hough_lines(NrPts-2*SKIP, x, y, sht_resolution, max_rho_d, sht_histogram, sht_nr,
		      &sht_nr_line_pts, sht_x_line_pts, sht_y_line_pts, xmin, ymin);

  for (i = 0; i < sht_nr_line_pts; i++) {
    sht_x_line_pts[i] = (sht_x_line_pts[i] + xmin) / scale_val; 
    sht_y_line_pts[i] = (sht_y_line_pts[i] + ymin) / scale_val;
  }


  
  // Kontroll Ausgabe
  print_hough_lines(sht_nr_line_pts, sht_x_line_pts, sht_y_line_pts);
  printf("end\n");fflush(stdout);

  // hier ist was merkwuerdig bei res = 200 stuertz der Rechner ab
  // exit block
  SHT_free_histogram(sht_histogram, sht_resolution);
  // end exit block
  
  return 0;

}
 
#endif

#ifdef STANDALONE
int main(int argc, char ** argv)
{

  int sht_resolution;
  int **sht_histogram;
  int sht_nr = 80;  // Nr
  int sht_nr_line_pts = 0,sht_nr_line = 0;
  double sht_x_line_pts[10000], sht_y_line_pts[10000] ;
  double max_rho_d;
  int result;

  int i;
  double array_x[10000], array_y[10000];   // these two arrays build
  double xmax, ymax, xmin, ymin;
  float x, y;
  int temp;

  FILE *fpr;

  xmax = ymax = xmin = ymin = 0;
  // READ input files
  fpr = fopen("points.dat","r");
  i = 0;
  while (!feof(fpr)) {
    fscanf(fpr,"%f %f",&x, &y);

    array_x[i] = (double)x;
    array_y[i] = (double)y;

    printf("%d %g %g \n",temp, array_x[i],array_y[i]);

    if (array_x[i] > xmax) xmax =  array_x[i];
    if (array_x[i] < xmin) xmin =  array_x[i];
    if (array_y[i] > ymax) ymax =  array_y[i];
    if (array_y[i] < ymin) ymin =  array_x[i];
    
    i++;
  }
  fclose(fpr);

  printf("***%f %f %f %f \n",xmax, ymax, xmin, ymin);
  printf("*%d \n",i);

  SHT_set_max_distance((int)MAX(xmax,ymax)+1,i);


  for (int j = 0; j < i; j++) {
    array_x[j] -= xmin; 
    array_y[j] -= ymin;
  }

  max_rho_d = sqrt(SQR(xmax-xmin) + SQR(ymax-ymin));
  sht_resolution = (int)(max_rho_d);
  sht_init(sht_resolution);
  sht_histogram = SHT_alloc_histogram(sht_resolution, sht_resolution);

  result = SHT_get_hough_lines(i, 
		      array_x, 
		      array_y,
		      sht_resolution,
		      max_rho_d,
		      sht_histogram, 
		      sht_nr,
		      &sht_nr_line_pts, 
		      sht_x_line_pts, 
		      sht_y_line_pts, 
		      xmin, 
		      ymin);

  printf("SHT_get_hough_lines: %d (%d)",result, sht_nr_line_pts);
  fflush(stdout);

  fpr = fopen("lines.dat","w");

  for (int j = 0; j < sht_nr_line_pts; j+=2) {
    sht_x_line_pts[j]   += xmin; 
    sht_y_line_pts[j]   += ymin;
    sht_x_line_pts[j+1] += xmin; 
    sht_y_line_pts[j+1] += ymin;

    if (SQR(sht_x_line_pts[j] - sht_x_line_pts[j+1]) + 
        SQR(sht_y_line_pts[j] - sht_y_line_pts[j+1]) > 225)
	 {
	   fprintf(fpr,"%g %g \n",sht_x_line_pts[j], sht_y_line_pts[j]);
	   fprintf(fpr,"%g %g \n\n",sht_x_line_pts[j+1], sht_y_line_pts[j+1]);
	 }
  }
  return 0;
}

#endif

 
