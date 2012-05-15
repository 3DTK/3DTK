#pragma once

#include <list>
#include <vector>
#include "veloslam/veloscan.h"
#include "veloslam/tracker.h"

#include <GL/gl.h>			/* OpenGL header file */
#include <GL/glu.h>			/* OpenGL utilities header file */

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

/** for Glut display mode */
#define RGBA 4   ///< colors for GLUT display
#define RGB 3    ///< colors for GLUT display
extern GLenum buffermode_debugView;

extern void StartShow();

void  GetCurrecntdelteMat(Scan& CurrentScan ,  double *deltaMat);

void  DrawText(float x, float y, float z, char * outputstring);
void  DrawTextRGB(float x, float y, float z, float r, float g, float b, char * outputstring);
void  DrawTextRGB(Point P, float r, float g, float b, char * outputstring);

void  Draw_Line_GL_RGB(float x1, float y1, float z1,  float x2, float y2, float z2, float r,float g,float b,float width=1.0);
void  Draw_Line_GL(float x1, float y1, float z1, float x2, float y2, float z2);
void  Draw_Line_GL_RGB(Point P1, Point P2, int width,	float r,float g,float b, bool arrow);

void  Draw_Cube_GL_RGB(float min_x, float min_y, float min_z,
	                                    float max_x, float max_y, float max_z,
										float r,float g,float b);

void  Draw_Inclined_Cube_GL_RGB(double rectangleVexPos[4][2],double min_z,double max_z,
	                                    float r,float g,float b,float width=1.0);

void  DrawPointsRGB(Point p,    float r,     float g,    float b);
void  DrawPoint(Point  p, int size , float r, float g, float b);
void  DrawPoint(Point  p, int size , float r, float g, float b, double deltaMat[16]);

int DrawAll_ScanPoints_Number(vector <Scan *> allScans,  int psize, float r, float g, float b, int n);


void glDumpWindowPPM_debugView(const char *filename, GLenum mode);

static void Reshape(int w, int h);
static void SpecialKey(int key, int x, int y);
static void Draw(void);
int Show(int frameno);



