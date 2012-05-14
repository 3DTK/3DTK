#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <sstream>
using std::stringstream;

#include "slam6d/scan.h"
#include "slam6d/Boctree.h"
//#include "slam6d/scan_io.h"
#include "slam6d/d2tree.h"
#include "slam6d/kd.h"
//#include "slam6d/kdc.h"
#include "veloslam/veloscan.h"
#include "veloslam/trackermanager.h"
#include "veloslam/debugview.h"
#include "veloslam/velodefs.h"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

using namespace boost;
using boost::thread;
boost::mutex draw_mutex;

extern TrackerManager trackMgr;
extern  VeloScan* g_pfirstScan;

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _MSC_VER
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

#include <cstring>
using std::flush;

#include <GL/gl.h>			/* OpenGL header file */
#include <GL/glu.h>			/* OpenGL utilities header file */

#ifdef _MSC_VER
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

#include "veloslam/color_util.h"

bool DebugDrawFinished =false;
bool ICPFinished =false;
bool save_animation=false;
int anim_frame_rate;
extern int scanCount;
GLenum buffermode_debugView = GL_BACK;

boost::mutex keymutex;
boost::condition keycond;
GLenum doubleBuffer;

int g_frame=0;
float rotX = 0.0, rotY = 0.0;

int x_lbefore,y_lbefore;
int x_rbefore,y_rbefore;
int z_before1,z_before2;

bool buttonSaveLeft,  buttonSaveMiddle ,  buttonSaveRight;
float x_move,y_move,z_move;
float x_move_save,y_move_save, z_move_save;
float x_rotate,y_rotate,z_rotate;
float x_rotate_save,y_rotate_save,z_rotate_save;
float m_zoom;

float m_aspect;

//	gluLookAt(0,  0,  80,   0,   0,  0,   0,  1,  0);
float m_eyex, m_eyey,  m_eyez;
float m_centerx,  m_centery, m_centerz;
float m_upx, m_upy, m_upz;

bool  g_pause=true;

void DrawPointsRGB(Point p,    float r,     float g,    float b)
{
    GLdouble dVect1[3];
    glColor3d(r, g, b);

    dVect1[0]=p.x;
    dVect1[1]=p.y;
    dVect1[2]=p.z;

    glVertex3dv(dVect1);
}

void DrawText(float x, float y, float z, char * outputstring)
{
#ifdef _MSC_VER
	glPushMatrix();
	glColor3f(1.0f,1.0f,1.0f);
	wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, 100);

	glListBase(100);
	glRasterPos3f(x, y, z);
	glCallLists( strlen(outputstring), GL_UNSIGNED_BYTE, outputstring);

	glPopMatrix();
#endif
}

void  DrawTextRGB(float x, float y, float z, float r, float g, float b, char * outputstring)
{
#ifdef _MSC_VER
	glPushMatrix();
	glColor3f(r,g,b);
	wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, 100);

	glListBase(100);
	glRasterPos3f(x, y, z);
	glCallLists( strlen(outputstring), GL_UNSIGNED_BYTE,  outputstring);

	glPopMatrix();
#endif
}

void  DrawTextRGB(Point P, float r, float g, float b, char * outputstring)
{
#ifdef _MSC_VER
	glPushMatrix();
	glColor3f(r,g,b);
	wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, 100);

	glListBase(100);
	glRasterPos3f(P.x, P.y, P.z);
	glCallLists( strlen(outputstring), GL_UNSIGNED_BYTE,  outputstring);

	glPopMatrix();
#endif
}

//void  Draw_Cube_GL_RGB(gridClusterFeature&f,float r,float g,float b)
//{
//	    Draw_Cube_GL_RGB(f.min_x,
//							f.min_y,
//							f.min_z,
//							f.max_x,
//							f.max_y,
//							f.max_z,r,g,b);
//}

void   Draw_Line_GL_RGB(float x1, float y1, float z1,
	                                    float x2, float y2, float z2,
										float r,float g,float b,float width)
{
		GLdouble dVect1[3];
		GLdouble dVect2[3];

		glLineWidth(width);
		glBegin(GL_LINES);
		glColor3d(r, g, b);

		////////////////////////
		dVect1[0]=x1;
		dVect1[1]=y1;
		dVect1[2]=z1;
		glVertex3dv(dVect1);

		dVect2[0]=x2;
		dVect2[1]=y2;
		dVect2[2]=z2;
		glVertex3dv(dVect2);

		glEnd();
}

void   Draw_Line_GL_RGB(Point P1, Point P2, int width,	float r,float g,float b, bool arrow)
{
		GLdouble dVect1[3];
		GLdouble dVect2[3];

		glLineWidth(width);
		glBegin(GL_LINES);
		glColor3d(r, g, b);

		dVect1[0]=P1.x;
		dVect1[1]=P1.y;
		dVect1[2]=P1.z;
		glVertex3dv(dVect1);

		dVect2[0]=P2.x;
		dVect2[1]=P2.y;
		dVect2[2]=P2.z;
		glVertex3dv(dVect2);

		glEnd();

		if(arrow)
		{
				glPushMatrix();
				glTranslatef(1.0,   0.0f,   0.0f);
				glRotatef(90.0f,0.0f,1.0f,0.0f);
				glutWireCone(0.027,0.09,10,10);
				glPopMatrix();
		}

}

void  Draw_Line_GL(float x1, float y1, float z1,float x2, float y2, float z2)
{
	Draw_Line_GL_RGB(x1,y1,z1,
		                           x2,y2,z2,
								   0,1.0,0);
}

void Draw_Cube_GL_RGB(float min_x, float min_y, float min_z,
	                                    float max_x, float max_y, float max_z,
										float r,float g,float b)
{
		Draw_Line_GL_RGB(min_x,min_y,max_z,max_x,min_y,max_z,r,g,b);
		Draw_Line_GL_RGB(min_x,max_y,max_z,max_x,max_y,max_z,r,g,b);
		Draw_Line_GL_RGB(min_x,max_y,min_z,max_x,max_y,min_z,r,g,b);
		Draw_Line_GL_RGB(min_x,min_y,min_z,max_x,min_y,min_z,r,g,b);

		Draw_Line_GL_RGB(min_x,min_y,max_z,min_x,max_y,max_z,r,g,b);
		Draw_Line_GL_RGB(max_x,min_y,max_z,max_x,max_y,max_z,r,g,b);
		Draw_Line_GL_RGB(min_x,min_y,min_z,min_x,max_y,min_z,r,g,b);
		Draw_Line_GL_RGB(max_x,min_y,min_z,max_x,max_y,min_z,r,g,b);

		Draw_Line_GL_RGB(min_x,min_y,min_z,min_x,min_y,max_z,r,g,b);
		Draw_Line_GL_RGB(max_x,min_y,min_z,max_x,min_y,max_z,r,g,b);
		Draw_Line_GL_RGB(max_x,max_y,min_z,max_x,max_y,max_z,r,g,b);
		Draw_Line_GL_RGB(min_x,max_y,min_z,min_x,max_y,max_z,r,g,b);
}

void Draw_Inclined_Cube_GL_RGB(double rectangleVexPos[4][2],double min_z,double max_z,
											float r,float g,float b,float width)
{
	Draw_Line_GL_RGB(rectangleVexPos[0][0],rectangleVexPos[0][1],min_z,rectangleVexPos[1][0],rectangleVexPos[1][1],min_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[1][0],rectangleVexPos[1][1],min_z,rectangleVexPos[2][0],rectangleVexPos[2][1],min_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[2][0],rectangleVexPos[2][1],min_z,rectangleVexPos[3][0],rectangleVexPos[3][1],min_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[3][0],rectangleVexPos[3][1],min_z,rectangleVexPos[0][0],rectangleVexPos[0][1],min_z,r,g,b,width);

	Draw_Line_GL_RGB(rectangleVexPos[0][0],rectangleVexPos[0][1],max_z,rectangleVexPos[1][0],rectangleVexPos[1][1],max_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[1][0],rectangleVexPos[1][1],max_z,rectangleVexPos[2][0],rectangleVexPos[2][1],max_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[2][0],rectangleVexPos[2][1],max_z,rectangleVexPos[3][0],rectangleVexPos[3][1],max_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[3][0],rectangleVexPos[3][1],max_z,rectangleVexPos[0][0],rectangleVexPos[0][1],max_z,r,g,b,width);

	Draw_Line_GL_RGB(rectangleVexPos[0][0],rectangleVexPos[0][1],min_z,rectangleVexPos[0][0],rectangleVexPos[0][1],max_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[1][0],rectangleVexPos[1][1],min_z,rectangleVexPos[1][0],rectangleVexPos[1][1],max_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[2][0],rectangleVexPos[2][1],min_z,rectangleVexPos[2][0],rectangleVexPos[2][1],max_z,r,g,b,width);
	Draw_Line_GL_RGB(rectangleVexPos[3][0],rectangleVexPos[3][1],min_z,rectangleVexPos[3][0],rectangleVexPos[3][1],max_z,r,g,b,width);
}


void Draw_Cube_GL_RGB(clusterFeature&f, float r,float g,float b)
{
	    Draw_Cube_GL_RGB(f.min_x,
							f.min_y,
							f.min_z,
							f.max_x,
							f.max_y,
							f.max_z,r,g,b);
}

void Draw_points_ZValue(const vector <Point>& Points,  int psize, float r, float g, float b, int type)
{
	int i,j;
	float *pColor;
	int size;
	float x,y,z ;

//	const vector <Point> *pPoints;
//	pPoints = scanRef1.get_points();

	glPointSize(psize);
	glBegin(GL_POINTS);
	int n =  Points.size();
	for(j=0; j <n; j++)
	{
		Point p =Points[j];
//		if(p.point_type & type)
	    	DrawPointsRGB(p,	r, g,b);
	}

	glEnd();
}

void Draw_points_ZValue_IN_ref(const vector <Point>& Points,  VeloScan& scanRef1,  VeloScan& scanR, int psize, float r, float g, float b, int TYPE)
{
	int i,j;
	float *pColor;
	int size;
	float x,y,z ;

     Point p, q;
	 double tempMat[16], deltaMat[16];

	 M4inv( scanR.getTransMatOrg(), tempMat);
     MMult(scanRef1.get_transMat(), tempMat, deltaMat);

	//const vector <Point> *pPoints;
	//pPoints = scanRef1.get_points();

	glPointSize(psize);
	glBegin(GL_POINTS);
	int n = Points.size();
	for(j=0; j <n; j++)
	{
		Point p = Points[j];
		p.transform(deltaMat);
		//		if(p.point_type & type)
		DrawPointsRGB(p,	r, g,b);
	}

	glEnd();
}


void Draw_points_ZValue(VeloScan& scanRef1,  int psize, float r, float g, float b)
{
	int i,j;
	float *pColor;
	int size;
	float x,y,z ;

	const vector <Point> *pPoints;
	pPoints = scanRef1.get_points();

	glPointSize(psize);
	glBegin(GL_POINTS);
	int n =  (*pPoints).size();
	for(j=0; j <n; j++)
	{
		Point p = (*pPoints)[j];
		DrawPointsRGB(p,	r, g,b);
	}

	glEnd();
}

void DrawPoint(Point  p, int size , float r, float g, float b)
{
		GLdouble dVect1[3];
		glPointSize(size);
		glBegin(GL_POINTS);
		glColor3d(r, g, b);
		dVect1[0]=p.x;
		dVect1[1]=p.y;
		dVect1[2]=p.z;
		glVertex3dv(dVect1);
		glEnd();
}

void DrawPoint(Point  p, int size , float r, float g, float b, double deltaMat[16])
{
		GLdouble dVect1[3];
		glPointSize(size);
		glBegin(GL_POINTS);

		p.transform(deltaMat);
		glColor3d(r, g, b);
		dVect1[0]=p.x;
		dVect1[1]=p.y;
		dVect1[2]=p.z;
		glVertex3dv(dVect1);
		glEnd();
}

void DrawObjectPoint(cluster &gluData1, int size , float r, float g, float b)
{
	cell* pCell;
	GLdouble dVect1[3];
	glPointSize(size);
	glBegin(GL_POINTS);
	glColor3d(r , g ,b);
	for(int j=0; j<gluData1.size();++j)
	{
		pCell=gluData1[j]->pCell;
		for(int k=0; k<pCell->size();++k)
		{
			dVect1[0]=(*pCell)[k]->x;
			dVect1[1]=(*pCell)[k]->y;
			dVect1[2]=(*pCell)[k]->z;

			glVertex3dv(dVect1);
		}
	}
	glEnd();
}

void DrawObjectPoint(cluster &gluData1, int size , float r, float g, float b, double deltaMat[16] )
{
	cell* pCell;
	GLdouble dVect1[3];
	glPointSize(size);
	glBegin(GL_POINTS);
	glColor3d(r , g ,b);
	for(int j=0; j<gluData1.size();++j)
	{
		pCell=gluData1[j]->pCell;
		for(int k=0; k<pCell->size();++k)
		{
			Point pp((*pCell)[k]->x,  (*pCell)[k]->y,  (*pCell)[k]->z);
			pp.transform(deltaMat);

     		dVect1[0]=pp.x;
			dVect1[1]=pp.y;
			dVect1[2]=pp.z;

			glVertex3dv(dVect1);
		}
	}
	glEnd();
}

//  change the mat for the firstScan  coodration.
void GetCurrecntdelteMat(Scan& CurrentScan ,  Scan& firstScan,  double *deltaMat)
{
	 //Point p, q;
	double tempMat[16];
    M4inv( firstScan.getTransMatOrg(), tempMat);
    MMult(CurrentScan.get_transMat(), tempMat, deltaMat);
}

int DrawAll_ScanPoints_Number(vector <Scan *> allScans,  int psize, float r, float g, float b, int n)
{
	 int i,j,k,colorIdx;
	 for(int i =0; i <n ;i ++)
	 {
		 Scan *firstScan = (Scan *)(g_pfirstScan);
//		 Scan *firstScan = allScans[0];
		 Scan *CurrentScan = allScans[i];
		 double  deltaMat[16];

		 GetCurrecntdelteMat(*CurrentScan ,  *firstScan,  deltaMat);

		 int size =(CurrentScan->get_points())->size();
	//	 vector<Point> PP = *(CurrentScan->get_points());

		 for(j=0; j <size; j++)
		 {
			 // need more complex classification
			 Point p = (*(CurrentScan->get_points()))[j];
             if (p.type & POINT_TYPE_GROUND)  //ground
 			    DrawPoint(p,  psize , 0.8,   0.8,   0.8,  deltaMat);
             if (p.type & POINT_TYPE_STATIC_OBJECT) //static
                DrawPoint(p,  psize , 0.0,  0.0,   0.8,  deltaMat);
             if (p.type & POINT_TYPE_MOVING_OBJECT) //moving
 			    DrawPoint(p,  psize , 0.0,   0.8,   0.0,  deltaMat);
		 }
	 }
	return 0;
}

void Draw_ALL_Cells_Points_IN_ref(VeloScan& scanRef1, VeloScan& scanR,  int psize, float r, float g, float b, int type)
{
//	Pn+1 = dP ¡¤ Pn
	 Point p, q;
	 double deltaMat[16];

     GetCurrecntdelteMat(scanRef1 , scanR,  deltaMat);
	cellArray myCellArray =scanRef1.scanCellArray;
	cellFeatureArray  myCellFeatureArray = scanRef1.scanCellFeatureArray;

	for(int j=0; j <myCellArray.size(); j++)
	{
		cellColumn &column= myCellArray[j];
		for(int i=0; i<column.size(); i++)
		{
			cellFeature& cellobj= myCellFeatureArray[j][i];
			p.x= cellobj.min_x;   p.y=cellobj.min_y; p.z=cellobj.min_z;
			p.transform(deltaMat);
			q.x= cellobj.max_x;   q.y=cellobj.max_y; q.z=cellobj.max_z;
			q.transform(deltaMat);

			if( cellobj.cellType & type)
						Draw_Cube_GL_RGB(p.x,		p.y,		p.z,		q.x,   	q.y,		q.z,	r, g, b);

		}

	}
}
//	 CELL_TYPE_FOR_SLAM6D
// CELL_TYPE_ABOVE_DELTA_Y
void Draw_ALL_Cells_Points(VeloScan& scanRef1,  int psize, float r, float g, float b, int type)
{
	cellArray myCellArray =scanRef1.scanCellArray;
	cellFeatureArray  myCellFeatureArray = scanRef1.scanCellFeatureArray;

	for(int j=0; j <myCellArray.size(); j++)
	{
		cellColumn &column= myCellArray[j];

		for(int i=0; i<column.size(); i++)
		{
			cellFeature& cellobj= myCellFeatureArray[j][i];

			if( cellobj.cellType & type)
						Draw_Cube_GL_RGB(
										cellobj.min_x,
										cellobj.min_y,
										cellobj.min_z,
										cellobj.max_x,
										cellobj.max_y,
										cellobj.max_z,
										r, g, b);

		}

	}
}


void Draw_ALL_Object_TYPE_IN_ref(VeloScan& scanRef1, VeloScan& scanR,  int psize, float r, float g, float b, int  TYPE)
{

//	Pn+1 = dP ¡¤ Pn
	  Point p, q;
 	 double deltaMat[16];
     GetCurrecntdelteMat(scanRef1 , scanR,  deltaMat);

	clusterArray myclusterArray =scanRef1.scanClusterArray;
	clusterFeatureArray  myclusterFeatureArray =scanRef1.scanClusterFeatureArray;

      glLineWidth(1);
	for(int j=0; j <myclusterArray.size(); j++)
	{
			clusterFeature& clusterobj= myclusterFeatureArray[j];

			p.x= clusterobj.min_x;   p.y=clusterobj.min_y; p.z=clusterobj.min_z;
			p.transform(deltaMat);
			q.x= clusterobj.max_x;   q.y=clusterobj.max_y; q.z=clusterobj.max_z;
			q.transform(deltaMat);

	//		if(cellobj.clusterType & TYPE )
		//	if(clusterobj.clusterType & TYPE )
				//	Draw_Cube_GL_RGB(p.x,		p.y,		p.z,		q.x,   	q.y,		q.z,	r, g, b);
	}
}

void Draw_ALL_Object_Points_TYPE_IN_ref(VeloScan& scanRef1, VeloScan& scanR,  int psize, float r, float g, float b, int  TYPE)
{
//	Pn+1 = dP ¡¤ Pn
     Point p;
 	 double deltaMat[16];
     GetCurrecntdelteMat(scanRef1 , scanR,  deltaMat);

	clusterArray myclusterArray =scanRef1.scanClusterArray;
	clusterFeatureArray  myclusterFeatureArray =scanRef1.scanClusterFeatureArray;

	glPointSize(psize);
	glBegin(GL_POINTS);

	for(int j=0; j <myclusterArray.size(); j++)
	{
			clusterFeature& clusterobj= myclusterFeatureArray[j];
			cluster& cluster= myclusterArray[j];

			for(int k=0; k<cluster.size();++k)
			{
				cell* pCell=cluster[k]->pCell;
				for(int l=0; l<pCell->size();++l)
				{
					p.x=(*pCell)[l]->x;
					p.y=(*pCell)[l]->y;
					p.z=(*pCell)[l]->z;

					p.transform(deltaMat);
					DrawPointsRGB(p,	r, g, b);

				}

			}
	}
	glEnd();
}


//CLUSTER_TYPE_OBJECT
void Draw_ALL_Object_TYPE(VeloScan& scanRef1,  int psize, float r, float g, float b, int  TYPE)
{
	clusterArray myclusterArray =scanRef1.scanClusterArray;
	clusterFeatureArray  myclusterFeatureArray =scanRef1.scanClusterFeatureArray;

	for(int j=0; j <myclusterArray.size(); j++)
	{
			clusterFeature& cellobj= myclusterFeatureArray[j];

	//		if(cellobj.clusterType & TYPE )
			if(cellobj.clusterType & CLUSTER_TYPE_STATIC_OBJECT )
					Draw_Cube_GL_RGB(
									cellobj.min_x,
									cellobj.min_y,
									cellobj.min_z,
									cellobj.max_x,
									cellobj.max_y,
									cellobj.max_z,
									1, 0, 1);

			if(cellobj.clusterType & CLUSTER_TYPE_MOVING_OBJECT )
					Draw_Cube_GL_RGB(
									cellobj.min_x,
									cellobj.min_y,
									cellobj.min_z,
									cellobj.max_x,
									cellobj.max_y,
									cellobj.max_z,
									0, 1, 0);

	}
}


void glDumpWindowPPM_debugView(const char *filename, GLenum mode)
{
  int win_height, win_width;
  int i,j,k,l;                  // Counter variables
  GLubyte *buffer;              // The GL Frame Buffer
  unsigned char *ibuffer;       // The PPM Output Buffer
  ofstream fp;                  // The PPM File

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  win_height = viewport[3];
  win_width  = viewport[2];

  // Allocate memory for the the frame buffer and output buffer
  buffer = new GLubyte[win_width * win_height * RGBA];
  ibuffer = new unsigned char[win_width * win_height * RGB];

  // Read window contents from GL frame buffer with glReadPixels
  glFinish();
  glReadBuffer(buffermode_debugView);
  glReadPixels(0, 0, win_width, win_height,
                       GL_RGBA, GL_UNSIGNED_BYTE, buffer);

 //   glReadPixels(0, 0, win_width, win_height,
//                        GL_RGB, GL_UNSIGNED_BYTE, buffer);

  // Open the output file
  fp.open(filename, ios::out);

  // Write a proper P6 PPM header
  fp << "P6" << endl << "# CREATOR: 3D_Viewer by Andreas Nuechter, University of Osnabrueck"
	<< endl << win_width  << " " << win_height << " " << UCHAR_MAX << endl;

  // Loop through the frame buffer data, writing to the PPM file.  Be careful
  //   to account for the frame buffer having 4 bytes per pixel while the
  //   output file has 3 bytes per pixel
  l = 0;
  for (i = 0; i < win_height; i++)
  {     // For each row
    for (j = 0; j < win_width; j++)
    {    // For each column
   //   for (k = 0; k < RGB; k++)
       {        // For each RGB component
        //cout << (RGBA*((win_height-1-i)*win_width+j)+k) << endl;
        ibuffer[l++] = (unsigned char)   *(buffer + (RGBA*((win_height-1-i)*win_width+j)+1));
	ibuffer[l++] = (unsigned char)   *(buffer + (RGBA*((win_height-1-i)*win_width+j)+2));
        ibuffer[l++] = (unsigned char)   *(buffer + (RGBA*((win_height-1-i)*win_width+j)+0));
      }                                  // end RGB
    }                                    // end column
  }                                      // end row

  // to make a video do:
  // for f in *ppm ; do convert -quality 100 $f `basename $f ppm`jpg; done
  // mencoder "mf://*.jpg" -mf fps=10 -o test.avi -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=800
  // Write output buffer to the file */
  fp.write((const char*)ibuffer, sizeof(unsigned char) * (RGB * win_width * win_height));
  fp.close();
  fp.clear();
  delete [] buffer;
  delete [] ibuffer;
}


static void Reshape(int w, int h)
{
    glViewport(0, 0, (GLint)w, (GLint)h);

	m_aspect = (GLfloat) w / (GLfloat) h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

	gluPerspective(45.0f,
					m_aspect,
					0.0f,
					4000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

#define KEY_ESC  27
static void Key(unsigned char key, int x, int y)
{
    switch (key)
	{
      case KEY_ESC:
	    exit(0);
		break;

    //case GLUT_KEY_P:
    //    printf("next\n");
    //    g_pause != g_pause;
    //    cout << g_pause;
    //    break;

    }
}

/*
		#define GLUT_KEY_F1			1
		#define GLUT_KEY_F2			2
		#define GLUT_KEY_F3			3
		#define GLUT_KEY_F4			4
		#define GLUT_KEY_F5			5
		#define GLUT_KEY_F6			6
		#define GLUT_KEY_F7			7
		#define GLUT_KEY_F8			8
		#define GLUT_KEY_F9			9

		#define GLUT_KEY_F10			10
		#define GLUT_KEY_F11			11
		#define GLUT_KEY_F12			12

		#define GLUT_KEY_LEFT			          100
		#define GLUT_KEY_UP			              101
		#define GLUT_KEY_RIGHT			      102
		#define GLUT_KEY_DOWN			      103
		#define GLUT_KEY_PAGE_UP		      104
		#define GLUT_KEY_PAGE_DOWN	  105
		#define GLUT_KEY_HOME			      106
		#define GLUT_KEY_END			          107
		#define GLUT_KEY_INSERT			  108
*/

//GLUT_LEFT_BUTTON
//GLUT_MIDDLE_BUTTON
//GLUT_RIGHT_BUTTON

//GLUT_DOWN
//GLUT_UP

static void MouseMove(int x, int y)
{
    int mod = glutGetModifiers();
  	switch(mod)
	{

    	case GLUT_ACTIVE_CTRL :
            x_rotate += (y - z_move_save)/100;
            if (x_rotate > 360)
              x_rotate=x_rotate - 360;
            if (x_rotate < -360)
              x_rotate=x_rotate + 360;
    		cout << "Ctrl Held" << endl;
            return;

    	case GLUT_ACTIVE_SHIFT :
            y_rotate += (y - z_move_save)/100;
            if (y_rotate > 360)
              y_rotate=y_rotate - 360;
            if (y_rotate < -360)
              y_rotate=y_rotate + 360;

    		cout << "Shift Held" << endl;
            return;

    	case GLUT_ACTIVE_ALT :
            float temp = (x - x_move_save)/100;
     		z_rotate += atanf(temp);

    		cout << "Alt Held" << endl;
            return;

  }

 if(buttonSaveLeft)
  {
      x_move += (x - x_move_save)/100;
      z_move += (y - z_move_save)/100;
  }

  if(buttonSaveMiddle)
  {
     float multiplay = (y - z_move_save)/1000;
     m_zoom =m_zoom*(1+multiplay);
  }

  if(buttonSaveRight)
  {
      float multiplay = (y - z_move_save)/1000;
      m_zoom =m_zoom*(1+multiplay);
  }

   cout << "mouse move " << buttonSaveLeft << buttonSaveMiddle <<buttonSaveRight << " "
    << x  << "  "<< y <<endl;
}

static void PassiveMouseMove(int x, int y)
{

}

static void MouseRotate(int x, int y, int z)
{
    cout << "mouse Rotate "  << x  << "  "<< y << "  "<< z <<endl;

}

static void MouseKey(int button, int state, int x, int y)
{
    x_move_save=x;
    y_move_save;
    z_move_save=y;

	switch (button)
	{
	case GLUT_LEFT_BUTTON:
        if(state == GLUT_DOWN)
           buttonSaveLeft=true;
        else
           buttonSaveLeft=false;
		break;

	case GLUT_MIDDLE_BUTTON:
        if(state == GLUT_DOWN)
           buttonSaveMiddle=true;
        else
           buttonSaveMiddle=false;
		break;

	case GLUT_RIGHT_BUTTON:
        if(state == GLUT_DOWN)
           buttonSaveRight=true;
        else
           buttonSaveRight=false;
		break;
	}
}

static void SpecialKey(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_UP:
		z_move --;
		break;

	case GLUT_KEY_DOWN:
		z_move ++;
		break;

	case GLUT_KEY_LEFT:
		x_move --;
		break;

	case GLUT_KEY_RIGHT:
		x_move ++;
		break;

	case GLUT_KEY_PAGE_UP:
		m_zoom= 1.1*m_zoom;
		break;

	case GLUT_KEY_PAGE_DOWN	:
		m_zoom =m_zoom/1.1;
		break;

	case GLUT_KEY_HOME:
		m_zoom=1.5*m_zoom;
		break;

	case GLUT_KEY_END:
		m_zoom=m_zoom/1.5;
		break;

	case  GLUT_KEY_F1:
		x_rotate += 3;
		if (x_rotate > 360)
			x_rotate=x_rotate - 360;
		if (x_rotate < -360)
			x_rotate=x_rotate + 360;
		break;

	case  GLUT_KEY_F2:
		x_rotate += -3;
		if (x_rotate > 360)
			x_rotate=x_rotate - 360;
		if (x_rotate < -360)
			x_rotate=x_rotate + 360;
		break;

	case  GLUT_KEY_F3:
		y_rotate += 3;
		if (y_rotate > 360)
			y_rotate=y_rotate - 360;
		if (y_rotate < -360)
			y_rotate=y_rotate + 360;
		break;

	case  GLUT_KEY_F4	:
		y_rotate += -3;
		if (y_rotate > 360)
			y_rotate=y_rotate - 360;
		if (y_rotate < -360)
			y_rotate=y_rotate + 360;
		break;

	case  GLUT_KEY_F5:
		z_rotate += atanf(3);
		break;

	case GLUT_KEY_F6:
		z_rotate += atanf(-3);
		break;

	case GLUT_KEY_F7:
		break;

	case GLUT_KEY_F8:
		break;

	case GLUT_KEY_F11:
	    printf("paseu\n");
        g_pause != g_pause;
		break;

	case GLUT_KEY_F12:
		printf("next\n");
		keycond.notify_one();
   		break;
	}
	glutPostRedisplay();
}

static void Draw(void)
{
	DebugDrawFinished =false;
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glLoadIdentity();

	gluLookAt(m_eyex, 	m_eyey, 	 m_eyez,
		m_centerx,	m_centery,	 m_centerz,
		m_upx,		m_upy,		 m_upz);

	glRotatef(x_rotate,1,0,0);
	glRotatef(y_rotate,0,1,0);
	glRotatef(z_rotate,0,0,1);

	glTranslatef(x_move,y_move,z_move);
	glScalef(m_zoom, m_zoom, m_zoom);

	int scansize = Scan::allScans.size();
//	boost::mutex::scoped_lock lock(draw_mutex);

    Point p1, p2, p3;

	if  (scansize <5)
	{
		DebugDrawFinished =true;
		return ;
	}

    Scan *firstScan = (Scan *)g_pfirstScan;
    Scan *CurrentScan = Scan::allScans[3];

    if (CurrentScan == NULL)
 	{
		DebugDrawFinished =true;
		return ;
	}

    double  deltaMat[16];
    double  deltaMatNext[16];

    GetCurrecntdelteMat(*CurrentScan , *firstScan,  deltaMat);
    double PosTheta[3];
    double Pos[3];
    Matrix4ToEuler(deltaMat, PosTheta, Pos);

    cout << "  pose of current "
         << "  " <<  PosTheta [0]
         << "  " <<  PosTheta [1]
         << "  " <<  PosTheta [2] <<endl;

   // you need to find the direction of ego-vehicle.
//    p1.x = -4000 * cos ( PosTheta [1]) ;  p1.y= 2000;  p1.z= 4000 * sin ( PosTheta [1]);
    p1.x = 0 ;  p1.y= 15000;  p1.z= 0;
    p2.x = 0; p2.y= 0;  p2.z= 0;
    p3.x = 0; p3.y=200;  p3.z= 0;

    p1.transform(deltaMat);
    p2.transform(deltaMat);
    p3.transform(deltaMat);

    gluLookAt(p1.x,   p1.y,   p1.z,
              p2.x,   p2.y,   p2.z,
              p3.x,   p3.y,   p3.z);

    DrawAll_ScanPoints_Number(Scan::allScans, 1, 0.8, 0.8, 0.8, scansize-1);
//  Draw_ALL_Object_TYPE(*(VeloScan *)( (Scan::allScans[scansize-1])) ,1, 1, 0, 0, 1   );
    trackMgr.DrawTrackersMovtion_Long_Number_All(Scan::allScans, scansize-1);
    trackMgr.DrawEgoTrajectory();
//	trackMgr.DrawTrackersContrailAfterFilted(Scan::allScans);
	glFlush();
 	glutSwapBuffers();

    if(save_animation)
    {
       string scandirectory =".\\video\\";
       string filename = scandirectory + "animframe"
                    + to_string(scanCount,5) + ".ppm";
       cout << "write " << filename << endl;
       glDumpWindowPPM_debugView(filename.c_str(),0);
    }

	DebugDrawFinished =true;
}

int Show(int frameno)
{
    GLenum type;

	m_eyex=0,  m_eyey=0,  m_eyez=80;
	m_centerx=0,  m_centery=0, m_centerz=0;
	m_upx=0,  m_upy=1,  m_upz=0;

    buttonSaveLeft=false;
    buttonSaveMiddle=false;
    buttonSaveRight=false;

	x_move=0.0;
	y_move=0.0;
    z_move=0.0;

	x_rotate=90.0;
	y_rotate=0.0;
	z_rotate=0.0;

	m_zoom=0.01;

	x_lbefore=0,y_lbefore=0;
	x_rbefore=0,y_rbefore=0;
	z_before1=0,z_before2=0;

    type = GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE;
    glutInitDisplayMode(type);

    glutInitWindowSize(800, 600);
    glutCreateWindow("Point Cloud Viewer");

    glutReshapeFunc(Reshape);
    glutKeyboardFunc(Key);
    glutSpecialFunc(SpecialKey);
    glutMouseFunc(MouseKey);
    glutMotionFunc(MouseMove);
    glutPassiveMotionFunc(PassiveMouseMove);
    glutSpaceballRotateFunc(MouseRotate);

    g_frame = frameno;

    while(ICPFinished ==false)
    {
			 Sleep(1);
		//	 cout << "sleep" <<endl;
    }
    glutDisplayFunc(Draw);
    glutMainLoop();

    return 0;
}


void ShowWraper()
{
	Show(0);
}

void StartShow()
{
	boost::thread t(ShowWraper);
	//t.join();
}
