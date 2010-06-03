/**
 * @author	Darko Makreshanski <d.makreshanski@jacobs-university.de>
 * @commands
 *		'w'	-	move camera forward
 *		's' - 	move camera backward
 *		'a' - 	move camera left
 *		'd' - 	move camera	right
 * 		'r'	-	move camera upwards
 *		'f' - 	move camera downwards
 *		'mouseMove' - rotate camera
 *		<esc>	exit program
 */


/*
 * GL.cpp
 *
 *  Created on: Feb 21, 2010
 *      Author: darko
 */

#define OVERAL_SPEED 0.4
#define FORWARD_SPEED 1
#define SIDE_SPEED 1
#define VERTICAL_SPEED 1
#define MOUSE_SENSITIVITY 1


#include "GL.h"
#include "CoordGL.h"
#include <GL/glut.h>
#include <FTGL/ftgl.h>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <time.h>
#include "Object_gl.h"

#include <sys/types.h>
#include <unistd.h>

#include <X11/Xlib.h>
#include "X.h"

using namespace std;


list<Object_gl *> GL::objects;

int GL::w_width = 1000;
int GL::w_height = 800;
double GL::aspect = GL::w_width / GL::w_height;

double GL::fovy = 40;
double GL::near = 0.0001;
double GL::far = 1000;

CoordGL GL::pos = CoordGL(5,0,0);

float GL::theta = 0;
float GL::phi  = M_PI / 2.0;

Display* GL::dsp = NULL;
Window* GL::wnd = NULL;

clock_t GL::lastMove = 0;

FTGLPixmapFont GL::font("src/sift/opengl_framework/tahoma.ttf");

GL::GL() {
	// TODO Auto-generated constructor stub

}

GL::~GL() {
	// TODO Auto-generated destructor stub
}


void GL::addObject(Object_gl* obj)
{
	GL::objects.push_back(obj);
}

void GL::execute()
{

	std::string window_title = "opengl_object_display";

//	glutInit(&argc, argv);
	char **tcc = NULL;
	int tci = 0;
	glutInit(&tci, tcc);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(w_width, w_height);
	glutInitWindowPosition(100, 100);

	int window;



	window = glutCreateWindow(window_title.c_str());

	glClearColor(0, 0, 0, 0);

    ///////////////////////////////////////////////////////////////////////
    /*
    GL_FLAT colors the interior of a polygon with the solid color that was current when the polygon's last vertex was specified.
    GL_SMOOTH does shading calculations in the interior of the triangle, it interpolates the vertex colors and smoothly transitions
    from one to another.
    */
	glShadeModel(GL_SMOOTH);
	//Tell OpenGL to use lightning calculations.
//	glEnable(GL_LIGHTING);
//	//Enable the light source
//	glEnable(GL_LIGHT0);
	///////////////////////////////////////////////////////////////////////

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(fovy, aspect, near, far);
	glutReshapeFunc((void (*) (int, int)) &GL::changeSize);
	glutDisplayFunc((void (*) ()) &GL::display);
	glutKeyboardFunc((void (*) (unsigned char, int, int)) &GL::processKeys);
	glutPassiveMotionFunc((void (*) (int, int)) &GL::processMouse);

	GL::dsp = XOpenDisplay(NULL);
	GL::wnd = X::getWindow(GL::dsp, getpid(), window_title);

//	X::fixMouse(dsp, wnd, w_width / 2 , w_height / 2);

//	cout << GL::lastMove << endl;


	glutMainLoop();

}

void GL::getFrustum(double fovY, double aspectRatio, double front, double back, double *frustum)
{
	const double DEG2RAD = 3.14159265 / 180;

	double tangent = tan(fovY/2 * DEG2RAD);   // tangent of half fovY
	double height = front * tangent;          // half height of near plane
	double width = height * aspectRatio;      // half width of near plane

	frustum[0] = -width;
	frustum[1] = width;
	frustum[2] = -height;
	frustum[3] = height;
	frustum[4] = front;
	frustum[5] = back;
}


void GL::display()
{

	float x_look = 1 * cos(phi)*cos(theta) + pos.x;
	float y_look = 1 * sin(theta) + pos.y;
	float z_look = 1 * sin(phi)*cos(theta) + pos.z;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0, 0, 0, 0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glPushMatrix();

	if (cos(theta)>=0) gluLookAt(pos.x, pos.y, pos.z, x_look, y_look, z_look, 0, 1, 0);
	else gluLookAt(pos.x, pos.y, pos.z, x_look, y_look, z_look, 0, -1, 0);
	
//	cout << "TEST\n";
	

    for (list<Object_gl*>::iterator ci = GL::objects.begin(); ci != GL::objects.end(); ++ci) {
//		glLoadIdentity();
		glPushMatrix();
//		cout << "TEST1\n";
		(*ci)->preshow();
//		cout << "TEST2\n";
    	(*ci)->show();
//		cout << "TEST3\n";
    	(*ci)->aftershow();
		glPopMatrix();
    }

//	glPushMatrix();
//	
//	glTranslatef(5,5,0);
//	glutWireSphere(1,5,5);
//	
//	glPopMatrix();

//	glPushMatrix();
//	
//	glPushMatrix();
//	glTranslatef(1,0,0);
//	glutWireSphere(1, 5, 5);
//	glPopMatrix();
//	
//	glPopMatrix();


//	glColor3f(0.0, 0.0, 1.0);
//	glutWireSphere(0.1,10,10);
//
//	glColor3f(0.0, 1.0, 1.0);
//	glTranslatef(0,0,3);
//	glutWireSphere(0.5,10,10);

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	// Create a pixmap font from a TrueType file.

	// Set the font size and render a small text.

	GL::font.FaceSize(18);
	char buf[150];
	sprintf(buf, "Th:%f;Ph:%f;x:%f;y:%f;z:%f\n", theta, phi, pos.x, pos.y, pos.z);
//	sprintf(buf, "xl %f ; yl %f ; zl %f", x_look, y_look, z_look);
//	sprintf(buf, "xp %f ; yp %f ; zp %f", x_pos, y_pos, z_pos);
	GL::font.Render(buf);

	glutSwapBuffers();
}


void GL::changeSize(int w, int h)
{
	if(h == 0)
		h = 1;
	w_width = w;
	w_height = h;
	float ratio = 1.0* w / h;
	aspect = ratio;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	gluPerspective(fovy,aspect,near,far);
}


void GL::processKeys(unsigned char key, int x, int y)
{

	float x_look = 0.1 * cos(phi)*cos(theta);
	float y_look = 0.1 * sin(theta);
	float z_look = 0.1 * sin(phi)*cos(theta);

	CoordGL look = CoordGL(x_look, y_look, z_look);
	look = look.normalize();
	CoordGL side_look = look * CoordGL(0,1,0);
	side_look = side_look.normalize();

	look = look * FORWARD_SPEED * OVERAL_SPEED;
	side_look = side_look * SIDE_SPEED * OVERAL_SPEED;

	CoordGL vertical = CoordGL(0,1,0) * VERTICAL_SPEED * OVERAL_SPEED;

//	cout << key << endl;

	switch(key)
	{
		case 'w':
		case 'W':
			pos = pos + look;
			break;
		case 's':
		case 'S':
			pos = pos - look;
			break;
		case 'a':
		case 'A':
			pos = pos - side_look;
			break;
		case 'd':
		case 'D':
			pos = pos + side_look;
			break;
		case 'r':
		case 'R':
			pos = pos + vertical;
			break;
		case 'f':
		case 'F':
			pos = pos - vertical;
			break;
		//		case 'z': if (aa_level > 1) aa_level--; break;
		//		case 'x': if (aa_level < 10) aa_level++; break;
		case 27: exit(0) ; break;
	}

	glutPostRedisplay();
}



void GL::processMouse(int x, int y)
{
	clock_t tMove = clock();
	if (tMove - GL::lastMove < 20000) {
		if (x - w_width / 2 != 0 || y - w_height / 2 != 0) {
			phi += ((float) (x - w_width / 2)) / 200 * MOUSE_SENSITIVITY;
			theta -= ((float) (y - w_height / 2)) / 200 * MOUSE_SENSITIVITY;
			if (theta > M_PI/2) theta = M_PI/2 - 0.001;
			if (theta < -M_PI/2) theta = -M_PI/2 + 0.001;
			glutPostRedisplay();
			X::fixMouse(GL::dsp, GL::wnd, w_width / 2, w_height / 2);
		}
	} else {
		X::fixMouse(GL::dsp, GL::wnd, w_width / 2, w_height / 2);
	}
	GL::lastMove = tMove;
}
//
//std::string getexepath()
//{
//	char result[ PATH_MAX ];
//	ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
//	return std::string( result, (count > 0) ? count : 0 );
//}
