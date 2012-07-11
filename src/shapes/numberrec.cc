/*
 * numberrec implementation
 *
 * Copyright (C) Jan Elseberg, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#ifdef HAVE_GETTIMEOFDAY 
#include <sys/time.h>
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

extern "C" {
#include "gocr.h"
#include "pnm.h"
#include "pgm2asc.h"

#include "pcx.h"
#include "ocr0.h" 
#include "progress.h"
#include "version.h"
#include "config.h"

}


char filter[] = "0-9";
/*void my_test() {
  int multipnm=1;
  job_t job;

  while (multipnm==1) {
    job_init(&job);

    job.cfg.out_format=XML;
    job.cfg.cfilter = filter; 
    job.src.fname = "patch00000.pgm";   // TODO

    multipnm=readpgm(job.src.fname, &job.src.p, job.cfg.verbose);
 
    if (multipnm<0) break; 

    pgm2asc(&job);

  int linecounter = 0;
  const char *line;

  linecounter = 0;
  line = getTextLine(linecounter++);
  while (line) {
    //fputs(line, stdout);
    //Interpret line here

    line = getTextLine(linecounter++);
  }
  free_textlines();
    job_free(&job);
  }
  return;
}*/
#include "math.h"
#include <string>
using std::string;

#include "shapes/sickday.h"

bool gocr_recognizeNumber(string &filename, int &number, double &probability) {
  int multipnm=1;
  job_t job;
  JOB = &job;

  double probabilities[1024];   // should be plenty

  probability = 0.0;
  number = -1;

  // this while should be unnecessary
  while (multipnm == 1) {
    job_init(&job);

    job.cfg.out_format = XML;
    job.cfg.cfilter = filter; 

    //job.src.fname = filename.c_str();
    char fname[1024];
    strcpy(fname, filename.c_str() );
    job.src.fname = fname;

    multipnm = readpgm(job.src.fname, &job.src.p, job.cfg.verbose);
 
    if (multipnm<0) break; 

    pgm2asc(&job);  // recognize numbers

    int linecounter = 0;
    const char *line;
    linecounter = 0;
    line = getTextLine(linecounter++);
    while (line) {
      //fputs(line, stdout);
      //Interpret line here
      int width = 0;
      int height = 0;
      char num;
      int nr_alt_c = 0;

      int iprob = 0;

      sscanf(line, "%d %d %c %d", &width, &height, &num, &nr_alt_c);

      if (nr_alt_c > 0) {
        sscanf(line, "%d %d %c %d %d", &width, &height, &num, &nr_alt_c, &iprob);
      }

      
      // check if this if a number, and if it is wether it is large enough
//      if ( (fabs(width) < MIN_NR_WIDTH || fabs(height) < MIN_NR_HEIGHT || num < '0' || num > '9')
      if (  num < '0' || num > '9') {  
        line = getTextLine(linecounter++);
        continue;
      }
      if ( num == '1' && (fabs(width) < MIN_1_WIDTH || fabs(height) < MIN_NR_HEIGHT ||
                          fabs(width) > MAX_NR_WIDTH || fabs(height) > MAX_NR_HEIGHT) ) {            // 1 is a special case
        line = getTextLine(linecounter++);
        continue;
      }
      if (num != '1' && (fabs(width) < MIN_NR_WIDTH || fabs(height) < MIN_NR_HEIGHT ||               // other numbers should be bigger 
                          fabs(width) > MAX_NR_WIDTH || fabs(height) > MAX_NR_HEIGHT) ) {
        line = getTextLine(linecounter++);
        continue;
      }

      // num contains a number (we will disregard alternative interpretation))
      if (probability < (double)iprob / 100.0) {
      //printf("WD:  %d %d     MIN  %d %d \n", width, height,  MIN_NR_WIDTH,  MIN_NR_HEIGHT);
        probability = (double)iprob / 100.0;
        number = (int)(num - '0');
      }
      //////////////////////
      line = getTextLine(linecounter++);
    }
    free_textlines();
    job_free(&job);
  }


  if (probability > 0.0) {
    return true;
  } 
  return false;
}
