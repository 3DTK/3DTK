#pragma once
 
#include <list>
#include <vector>
#include "veloslam/veloscan.h"
#include "veloslam/tracker.h"

void GetCurrecntdelteMat(Scan& CurrentScan ,  Scan& firstScan,  double *deltaMat);

void DrawText(float x, float y, float z, char * outputstring);
void  DrawTextRGB(float x, float y, float z, float r, float g, float b, char * outputstring);

void   Draw_Line_GL_RGB(float x1, float y1, float z1,
	                                    float x2, float y2, float z2,
										float r,float g,float b);
void  Draw_Line_GL(float x1, float y1, float z1, float x2, float y2, float z2);

void Draw_Cube_GL_RGB(float min_x, float min_y, float min_z,
	                                    float max_x, float max_y, float max_z,
										float r,float g,float b);
void Draw_Cube_GL_RGB(clusterFeature&f, float r,float g,float b);

void DrawPointsRGB(Point p,    float r,     float g,    float b);
void DrawPoint(Point  p, int size , float r, float g, float b);
void DrawPoint(Point  p, int size , float r, float g, float b, double deltaMat[16]);

void Draw_points_ZValue(int frame,  int psize);
void Draw_points_ZValue(const vector <Point>& Points,  int psize, float r, float g, float b, int type);
void Draw_points_ZValue_IN_ref(const vector <Point>& Points,  VeloScan& scanRef1,  VeloScan& scanR, int psize, float r, float g, float b, int TYPE);
void Draw_points_ZValue(VeloScan& scanRef1,  int psize, float r, float g, float b);
int DrawAll_ScanPoints_Number(vector <Scan *> allScans, int psize, float r, float g, float b, int n);

void Draw_ALL_Cells_Points_IN_ref(VeloScan& scanRef1, VeloScan& scanR,  int psize, float r, float g, float b, int type);
void Draw_ALL_Cells_Points(VeloScan& scanRef1,  int psize, float r, float g, float b, int type);
void Draw_ALL_Cells(int frame,  int psize);

void DrawObjectPoint(cluster &gluData1, int size , float r, float g, float b);
void DrawObjectPoint(cluster &gluData1, int size , float r, float g, float b, double deltaMat[16] );

void Draw_ALL_Object(int frame,  int psize);
void Draw_ALL_Object_TYPE_IN_ref(VeloScan& scanRef1, VeloScan& scanR,  int psize, float r, float g, float b, int  TYPE);
void Draw_ALL_Object_Points_TYPE_IN_ref(VeloScan& scanRef1, VeloScan& scanR,  int psize, float r, float g, float b, int  TYPE);
void Draw_ALL_Object_TYPE(VeloScan& scanRef1,  int psize, float r, float g, float b, int  TYPE);

static void Reshape(int w, int h);
static void SpecialKey(int key, int x, int y);
static void Draw(void);
int Show(int frameno);



