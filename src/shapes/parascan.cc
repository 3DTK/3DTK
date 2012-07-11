/*
 * parascan implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "shapes/parascan.h"


/* -------------------------------------------------------------------------- */

int  paramtr_scan_int(FILE *stream, const char *search,int defaultval)
{
  char target[256];

    if(paramtr_scan_str(stream, search, target))
      return(atoi(target));
    else
      return(defaultval);
}

/* -------------------------------------------------------------------------- */

double  paramtr_scan_double(FILE *stream, const char *search,double defaultval)
{
  char target[256];
   
    if(paramtr_scan_str(stream, search, target))
      return(atof(target));
    else
      return(defaultval);
}

/* -------------------------------------------------------------------------- */

bool paramtr_scan_str(FILE *stream, const char *search, char *target)
{
  const char  *ptr = search;
  int   c, mem;

    /* erster durchlauf */
    mem = 0;

    while(mem < 2) {
      if((c = getc(stream)) == EOF) {
	rewind(stream);
	mem++;
      }
      else {
	/* kommentarzeile uebergehen */
	if(c == COMMENT) {
	  while((c = getc(stream)) != LF );
	} /* if */

	if(*ptr) {
	  if(c == (int)*ptr) {
	    ptr++;
	  }
	  else {
	    ptr = search;
	  } /* if */
	}
	else {
	  int numberOfItemsRead = fscanf(stream, "%s", target);
          if(!numberOfItemsRead)
            {
              return false;
            }
	  return true;
	} /* if */
      } /* if */
    } /* while */

    return false;
}

/* -------------------------------------------------------------------------- */

int  paramtr_scan_echo(const char *filename, const char *search)
{
  const char  *ptr = search;
  char str[256];
  char* target;
  int   c, no;
  FILE  *stream;

    /* dateiverbindung oeffnen */
    if((stream = fopen(filename, "rt")) == NULL) {
      printf("paramtr_scan_echo() error\n");
      return 0;
    } /* if */

    no = 0;

    while((c = getc(stream)) != EOF) {
      /* kommentarzeile uebergehen */
      if(c == COMMENT) {
	while((c = getc(stream)) != LF );
      } /* if */

      if(*ptr) {
	if(c == (int)*ptr) {
          ptr++;
        }
        else {
          ptr = search;
        } /* if */
      }
      else {
        target = &(str[0]);
        while(c != LF) {
          *target = (char) c;
          target++;
          c = getc(stream);
        } /* while */
        *target = 0;
        printf("%s\n", str);
        ptr = search;
        no++;
      } /* if */
    } /* while */

    /* datei schliessen */
    fclose(stream);

  return no;
}

/* -------------------------------------------------------------------------- */
