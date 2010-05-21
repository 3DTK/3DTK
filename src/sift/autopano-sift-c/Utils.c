#include "AutoPanoSift.h"

/* these are generic support routines */

#ifdef _MSC_VER

#include <stdio.h>
#include <stdarg.h>
#include <varargs.h>
#include <direct.h>

#define PATH_MAX 1024

#define getcwd _getcwd

#else

#include <unistd.h>

#endif

void Write(const char* fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);	
    fflush(stdout);
}

void WriteLine(const char* fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);	
    printf("\n");
    fflush(stdout);
}

void WriteError(const char* fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);	
    fprintf(stderr, "\n");
    fflush(stderr);
}

void FatalError(const char* fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    WriteError(fmt, ap);
    exit(-1);
}

char* FileNameToFullPath(char* filename) 
{
    char buf[PATH_MAX];
    char* cwd = getcwd(buf, sizeof(buf));
    char* path = (char*)malloc(strlen(cwd)+1+strlen(filename)+1);
    sprintf(path, "%s/%s", cwd, filename);
    return path;
}

char* FullPathToFileName(char* path) 
{
    char* filename = strrchr(path, '/');
    if (filename == NULL) filename = path;
    return strdup(filename);
}

void*** PtrMap_new(int yDim, int xDim)
{
    int* mem = (int*) malloc(sizeof(int)*2 + yDim * sizeof(void**));
    void*** self = (void***)(mem+2);
    int i;
    mem[0] = xDim;
    mem[1] = yDim;
    for(i=0; i<yDim; i++) {
	self[i] = (void**)calloc(xDim , sizeof(void*));
    }
    return self;
}

void PtrMap_delete(void*** self, void (*deletefn)(void*) ) {
    if (self) {
	int* mem = ((int*)self)-2;
	if (*mem) {
	    int xDim = mem[0];
	    int yDim = mem[1];
	    int i;
	    for(i=0; i<yDim; i++) {
		if (deletefn != NULL) {
		    void (*_deletefn)(void*) = deletefn;
		    int j;
		    for(j=0; i< xDim; j++) {
			if (self[i][j] != NULL) {
			    _deletefn(self[i][j]);
			    self[i][j] = NULL;
			}
		    }
		}
		free(self[i]);
		self[i]=NULL;
	    }
	    *mem = 0;
	    free(mem);
	}
    }
}

int** IntMap_new(int yDim, int xDim) {
    int* mem = (int*) malloc(sizeof(int)*2 + yDim * sizeof(int*));
    int** self = (int**)(mem+2);
    int i;
    mem[0] = xDim;
    mem[1] = yDim;
    for(i=0; i<yDim; i++) {
	self[i] = (int*)calloc(xDim , sizeof(int));
    }
    return self;
}

void IntMap_delete(int** self) {
    if (self) {
	int* mem = ((int*)self)-2;
	if (*mem) {
	    int xDim = mem[0];
	    int yDim = mem[1];
	    int i;
	    for(i=0; i<yDim; i++) {
		free(self[i]);
		self[i]=NULL;
	    }
	    *mem = 0;
	    free(mem);
	}
    }
}


float** FloatMap_new(int yDim, int xDim) {
    int i;
    int* mem = (int*)malloc(sizeof(int)*2 + yDim * sizeof(float*));
    float** self = (float**)(mem+2);
	if( mem ){
	  float * map = (float *)calloc(yDim, xDim * sizeof(float));
	  if( !map ){
		  free( mem );
		  FatalError("out of memory");
	  }
      mem[0] = xDim;
      mem[1] = yDim;
      for(i=0; i<yDim; i++) self[i] = map + xDim * i;
    } else FatalError("out of memory");
    return self;
}

void FloatMap_delete(float** self) {
  if (self) {
	int* mem = ((int*)self)-2;
	free(self[0]);
	free(mem);
  }
}

double** DoubleMap_new(int yDim, int xDim) {
    int* mem = (int*)malloc(sizeof(int)*2 + yDim * sizeof(double*));
    double** self = (double**)(mem+2);
    int i;
    mem[0] = xDim;
    mem[1] = yDim;
    for(i=0; i<yDim; i++) {
	self[i] = (double*)malloc(xDim * sizeof(double));
	memset(self[i], 0, xDim*sizeof(double));
	if (self[i] == NULL) { FatalError("out of memory"); }
    }
    return self;
}

void DoubleMap_delete(double** self) {
    if (self) {
	int* mem = ((int*)self)-2;
	if (*mem) {
	    int xDim = mem[0];
	    int yDim = mem[1];
	    int i;
	    for(i=0; i<yDim; i++) {
		if (self[i] != NULL) {
		    free(self[i]);
		    self[i]=NULL;
		}
	    }
	    *mem = 0;
	    free(mem);
	}
    }
}

