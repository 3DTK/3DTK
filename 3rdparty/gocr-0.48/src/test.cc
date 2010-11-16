/*
This is a Optical-Character-Recognition program
Copyright (C) 2000-2009  Joerg Schulenburg

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

 see README for EMAIL-address

  sometimes I have written comments in german language, sorry for that

 This file was retrieved from pgm2asc.cc of Joerg, in order to have
 a library of the ocr-engine from Klaas Freitag
 
 */
#include "config.h"
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
#include "ocr0.h"		/* only_numbers */
#include "progress.h"
#include "version.h"

static void out_version(int v) {
  fprintf(stderr, " Optical Character Recognition --- gocr "
          version_string " " release_string "\n"
          " Copyright (C) 2001-2009 Joerg Schulenburg  GPG=1024D/53BDFBE3\n"
          " released under the GNU General Public License\n");
  /* as recommended, (c) and license should be part of the binary */
  /* no email because of SPAM, see README for contacting the author */
  if (v)
    fprintf(stderr, " use option -h for help\n");
  if (v & 2)
    exit(1);
  return;
}

static void help(void) {
  out_version(0);
  /* output is shortened to essentials, see manual page for details */
  fprintf(stderr,
	  " using: gocr [options] pnm_file_name  # use - for stdin\n"
	  " options (see gocr manual pages for more details):\n"
	  " -h, --help\n"
	  " -i name   - input image file (pnm,pgm,pbm,ppm,pcx,...)\n"
	  " -o name   - output file  (redirection of stdout)\n"
	  " -e name   - logging file (redirection of stderr)\n"
	  " -x name   - progress output to fifo (see manual)\n"
	  " -p name   - database path including final slash (default is ./db/)\n");
  fprintf(stderr, /* string length less than 509 bytes for ISO C89 */
	  " -f fmt    - output format (ISO8859_1 TeX HTML XML UTF8 ASCII)\n"
	  " -l num    - threshold grey level 0<160<=255 (0 = autodetect)\n"
	  " -d num    - dust_size (remove small clusters, -1 = autodetect)\n"
	  " -s num    - spacewidth/dots (0 = autodetect)\n"
	  " -v num    - verbose (see manual page)\n"
	  " -c string - list of chars (debugging, see manual)\n"
	  " -C string - char filter (ex. hexdigits: ""0-9A-Fx"", only ASCII)\n"
	  " -m num    - operation modes (bitpattern, see manual)\n");
  fprintf(stderr, /* string length less than 509 bytes for ISO C89 */
	  " -a num    - value of certainty (in percent, 0..100, default=95)\n"
	  " -u string - output this string for every unrecognized character\n");
  fprintf(stderr, /* string length less than 509 bytes for ISO C89 */
	  " examples:\n"
	  "\tgocr -m 4 text1.pbm                   # do layout analyzis\n"
	  "\tgocr -m 130 -p ./database/ text1.pbm  # extend database\n"
	  "\tdjpeg -pnm -gray text.jpg | gocr -    # use jpeg-file via pipe\n"
	  "\n");
  fprintf(stderr, " webpage: http://jocr.sourceforge.net/\n");
  exit(0);
}

#ifdef HAVE_GETTIMEOFDAY
/* from the glibc documentation */
static int timeval_subtract (struct timeval *result, struct timeval *x, 
    struct timeval *y) {

  /* Perform the carry for the later subtraction by updating Y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     `tv_usec' is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}
#endif

static void process_arguments(job_t *job)
{

#ifdef HAVE_PGM_H
  pnm_init(&argn, &argv);
#endif
  
job->cfg.out_format=XML; else
job->cfg.cfilter = "0-9";

job->src.fname = "patch00000.pgm";   // TODO
}

static int read_picture(job_t *job) {
  int rc=0;
  assert(job);

  if (strstr(job->src.fname, ".pcx"))
    readpcx(job->src.fname, &job->src.p, job->cfg.verbose);
  else
    rc=readpgm(job->src.fname, &job->src.p, job->cfg.verbose);
  return rc; /* 1 for multiple images, 0 else */
}

/* subject of change, we need more output for XML (ToDo) */
void print_output(job_t *job) {
  int linecounter = 0;
  const char *line;

  assert(job);
        
  linecounter = 0;
  line = getTextLine(linecounter++);
  while (line) {
    /* notice: decode() is shiftet to getTextLine since 0.38 */
    fputs(line, stdout);
    if (job->cfg.out_format==HTML) fputs("<br />",stdout);
    if (job->cfg.out_format!=XML)  fputc('\n', stdout);
    line = getTextLine(linecounter++);
  }
  free_textlines();
}

/* FIXME jb: remove JOB; */
job_t *JOB;


/* -------------------------------------------------------------
// ------   MAIN - replace this by your own aplication! 
// ------------------------------------------------------------- */
int main(int argn, char *argv[]) {
  int multipnm=1;
  job_t job;
  setvbuf(stdout, (char *) NULL, _IONBF, 0);	/* not buffered */
  
  while (multipnm==1) {

    job_init(&job);
  
//    process_arguments(&job, argn, argv);

    mark_start(&job);

    multipnm = read_picture(&job);
    /* separation of main and rest for using as lib
       this will be changed later => introduction of set_option()
       for better communication to the engine  */
    if (multipnm<0) break; /* read error */
  
    /* call main loop */
    pgm2asc(&job);

    mark_end(&job);
  
    job_free(&job);
  
  }
  
  return 0;
}



void test() {
  int multipnm=1;
  job_t job;

  while (multipnm==1) {
    job_init(&job);
  
    job->cfg.out_format=XML; 
    job->cfg.cfilter = "0-9";
    job->src.fname = "patch00000.pgm";   // TODO

    multipnm = read_picture(&job);
    /* separation of main and rest for using as lib
       this will be changed later => introduction of set_option()
       for better communication to the engine  */
    if (multipnm<0) break; /* read error */
  
    /* call main loop */
    pgm2asc(&job);

  int linecounter = 0;
  const char *line;

  assert(job);
        
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
}
