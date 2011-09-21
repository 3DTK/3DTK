/* parascan.h */

#ifndef parascan
#define parascan

/* dependencies */
/* -------------------------------------------------------------------------- */

/* constant definitions */
/* -------------------------------------------------------------------------- */

#define LF              0x0A    /* line feed: zeilenvorschub       */
#define CR              0x20    /* carriage return: wagenruecklauf */
#define COMMENT         0x3B    /* ';' als kommentartrennzeichen   */
#define NAMELEN         80      /* maximale Laenge der Dateinamen  */

/* typedefinitions */
/* -------------------------------------------------------------------------- */

/* macros */
/* -------------------------------------------------------------------------- */

/* prototyps */
/* -------------------------------------------------------------------------- */

extern int  paramtr_scan_int(FILE *stream, const char *search,int defaultval);
extern double  paramtr_scan_double(FILE *stream, const char *search,double defaultval);
extern bool  paramtr_scan_str(FILE *stream, const char *search, char *target);
extern int  paramtr_scan_echo(const char *filename, const char *search);
/* descriptions */
/* -------------------------------------------------------------------------- */


#endif /* end of parascan.h */

