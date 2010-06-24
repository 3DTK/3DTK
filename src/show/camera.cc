/**
 * @file camera.cc
 */
#include "camera.h"
#ifdef _MSC_VER
#include <windows.h>
#endif

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "glui/glui.h"

Camera::Camera()
{
  // Constructor for the camera class
  x = y = z = angle  = 0.0f;       // initialize the variables
  cam_size = 20.0f;
  //initialize the camera color
  camcolor.r = 0.0;
  camcolor.g = 0.0;
  camcolor.b = 1.0;
  focal_length = 200.0f;
  view_width = 100.0f;

  r_x = r_y = r_z = 0.0;
  focal_x = x;
  focal_y = y;
  focal_z = z + focal_length;

}

Camera::~Camera()
{
  // Destructor for the camera class
  // do something here
}

void Camera::drawCamera(bool showbeam)
{
  // draw the Camera in the viewer window

  glPushMatrix();

  // translate the object to the original position
  glTranslated(x, y, z);
  // rotate it with the given angle
  glRotatef(angle, r_x, r_y, r_z);
  // translate the object to the origin
  glTranslated(-x, -y, -z);

  // note that when multiplying it does the opposite.
  // first it translates the object to the origin and
  // then rotates it and translates it back to the original position

  // Now draw the cube

  // set the color of the cube
  glColor3f(camcolor.r, camcolor.g, camcolor.b);

  // set the polygon mode
  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL); 

  // begin drawing
  glBegin(GL_QUADS);
    // Quad 1
     glVertex3f( x + (cam_size/2), y + (cam_size/2), z + (cam_size/2));   // v3
	glVertex3f( x + (cam_size/2), y - (cam_size/2), z + (cam_size/2));   // v2
	glVertex3f( x - (cam_size/2), y - (cam_size/2), z + (cam_size/2));   // v1
	glVertex3f( x - (cam_size/2), y + (cam_size/2), z + (cam_size/2));   // v4


   //Quad 3
	glVertex3f( x - (cam_size/2), y + (cam_size/2), z + (cam_size/2));   // v4
	glVertex3f( x + (cam_size/2), y + (cam_size/2), z + (cam_size/2));   // v3
     glVertex3f( x + (cam_size/2), y + (cam_size/2), z - (cam_size/2));   // v6
	glVertex3f( x - (cam_size/2), y + (cam_size/2), z - (cam_size/2));   // v7

	
   //Quad 2
     glVertex3f( x - (cam_size/2), y + (cam_size/2), z - (cam_size/2));   // v7 
     glVertex3f( x + (cam_size/2), y + (cam_size/2), z - (cam_size/2));   // v6   
     glVertex3f( x + (cam_size/2), y - (cam_size/2), z - (cam_size/2));   // v5
     glVertex3f( x - (cam_size/2), y - (cam_size/2), z - (cam_size/2));   // v8 
  
   //Quad 4
	glVertex3f( x - (cam_size/2), y - (cam_size/2), z - (cam_size/2));   // v8 
	glVertex3f( x + (cam_size/2), y - (cam_size/2), z - (cam_size/2));   // v5
	glVertex3f( x + (cam_size/2), y - (cam_size/2), z + (cam_size/2));   // v2
	glVertex3f( x - (cam_size/2), y - (cam_size/2), z + (cam_size/2));   // v1

	//Quad 6
     glVertex3f( x - (cam_size/2), y - (cam_size/2), z + (cam_size/2));   // v1
     glVertex3f( x - (cam_size/2), y + (cam_size/2), z + (cam_size/2));   // v4
     glVertex3f( x - (cam_size/2), y + (cam_size/2), z - (cam_size/2));   // v7
 	glVertex3f( x - (cam_size/2), y - (cam_size/2), z - (cam_size/2));   // v8    

	//Quad 5
     glVertex3f( x + (cam_size/2), y + (cam_size/2), z + (cam_size/2));   // v3
     glVertex3f( x + (cam_size/2), y + (cam_size/2), z - (cam_size/2));   // v6
     glVertex3f( x + (cam_size/2), y - (cam_size/2), z - (cam_size/2));   // v5
     glVertex3f( x + (cam_size/2), y - (cam_size/2), z + (cam_size/2));   // v2
  glEnd();

 
 // now if show beam is true then draw the beam
  if (showbeam) {
    focal_x = x;
    focal_y = y;
    focal_z = z - focal_length;

    // set the line width
    glLineWidth(1.0);
    // set the color of the beam
    glColor3f(1.0, 1.0, 0.0);
    // set the stipple style
    glLineStipple(1, 0x0C0F);
    // trun on stippled lines
    glEnable(GL_LINE_STIPPLE);
    // set the polygon mode
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE); 
    // start drawing the beam
    glBegin(GL_TRIANGLES);
    // upper face
    glVertex3f( x , y , z - (cam_size/2));                                 // c1
    glVertex3f( x + (view_width/2), y + (view_width/2), z - focal_length); // c2
    glVertex3f( x - (view_width/2), y + (view_width/2), z - focal_length); // c3

    // right face
    glVertex3f( x , y , z - (cam_size/2));                                 // c1
    glVertex3f( x - (view_width/2), y + (view_width/2), z - focal_length); // c3
    glVertex3f( x - (view_width/2), y - (view_width/2), z - focal_length); // c4

    // bottom face
    glVertex3f( x , y , z + (cam_size/2));                                 // c1
    glVertex3f( x - (view_width/2), y - (view_width/2), z - focal_length); // c4
    glVertex3f( x + (view_width/2), y - (view_width/2), z - focal_length); // c5

    // left face
    glVertex3f( x , y , z - (cam_size/2));                                 // c1
    glVertex3f( x + (view_width/2), y - (view_width/2), z - focal_length); // c5
    glVertex3f( x + (view_width/2), y + (view_width/2), z - focal_length); // c2
    glEnd();

    glPointSize(10);

    glBegin(GL_POINTS);
    glVertex3f(focal_x, focal_y, focal_z);
    glEnd();

    float mviewMatrix[16];
    float fMatrix[4];

    fMatrix[0] = focal_x - x;
    fMatrix[1] = focal_y - y;
    fMatrix[2] = focal_z - z;
    fMatrix[3] = 1;

    glGetFloatv(GL_MODELVIEW_MATRIX, mviewMatrix);

    fx = fMatrix[0] * mviewMatrix[0] + fMatrix[1] * mviewMatrix[4] + fMatrix[2] * mviewMatrix[8];
    fy = fMatrix[0] * mviewMatrix[1] + fMatrix[1] * mviewMatrix[5] + fMatrix[2] * mviewMatrix[9];
    fz = fMatrix[0] * mviewMatrix[2] + fMatrix[1] * mviewMatrix[6] + fMatrix[2] * mviewMatrix[10];
    focal_x = fx + x;// + mviewMatrix[11];
    focal_y = fy + y;// + mviewMatrix[12];
    focal_z = fz + z;// + mviewMatrix[13];

    // disable the stippled lines
    glDisable(GL_LINE_STIPPLE);
    // reset the line width
    glLineWidth(1.0);

  }
 
 // now pop the matrix
 glPopMatrix();

 // signal to repaint the screen
 glutPostRedisplay();

}


void Camera::addCamera(float newfx,
				   float newfy,
				   float newfz,
				   float newangle,
				   float newr_x,
				   float newr_y,
				   float newr_z,
				   float newx,
				   float newy,
				   float newz,
				   float newflength,
				   float newsize,
				   float newvwidth){
  //add a camera in the viewer
  x     = newx;
  y     = newy;
  z     = newz;
  angle = newangle;
  r_x = newr_x;
  r_y = newr_y;
  r_z = newr_z;
  cam_size = newsize;
  camcolor.r = 0.0;
  camcolor.g = 0.0;
  camcolor.b = 1.0;
  focal_length = newflength;
  view_width = newvwidth;
  focal_x = newfx;
  focal_y = newfy;
  focal_z = newfz;

}

void Camera::deleteCamera(){
  //delete the specified camera

}

void Camera::moveCamera(float newangle, float newx, float newy, float newz, float newsize){
  //move the camera to the new location
  x     = newx;
  y     = newy;
  z     = newz;
  angle = newangle;
  cam_size = newsize;
  
}

void Camera::setRotate(float angle1, float x_c, float y_c, float z_c){
  //rotate the camera
 
  angle = angle1;
  r_x = x_c;
  r_y = y_c;
  r_z = z_c;

  glRotatef(angle1, x_c, y_c, z_c);
  
  
}

  



