// =====================================================================================
// 
//       Filename:  test.cc
// 
//    Description:  
// 
//        Version:  1.0
//        Created:  09/26/2010 12:15:50 AM
//       Revision:  none
//       Compiler:  g++
// 
//         Author:  Jan Elseberg (), jelseber@uos.de
//        Company:  Universitaet Osnabrueck
// 
// =====================================================================================
 
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

#include "pnm.h"
#include "pgm2asc.h"
#include "pcx.h"
#include "ocr0.h" 
#include "progress.h"
#include "version.h"
#include "config.h"

char filter[] = "0-9";
void test() {
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
}
int main() {
  return 0;
}
/*
#include "gocr/pnm.h"
#include "gocr/pgm2asc.h"
#include "gocr/pcx.h"
#include "gocr/ocr0.h" 
#include "gocr/progress.h"
#include "gocr/version.h"
#include "gocr/config.h"

int main() {
  job_t job;


return 0;
}
*/
