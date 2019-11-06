#include "show/callbacks_glut.h"
#include "show/show_gl.h"
#include "png.h"
#include <thread>

using namespace callbacks::glut;

bool   fullydisplayed = true;       // true if all points have been drawn to
                                    // the screen
bool   mousemoving    = false;      // true if a mouse button has been pressed
                                    // inside a window,
                                    // but has not been released
bool   keypressed     = false;      // true if a key button has been pressed
                                    // inside a window,
                                    // but has not been released
double ptstodisplay   = 100000;
double lastfps        = idealfps;   // last frame rate
int    pointmode      = -1;

bool   smallfont      = true;
bool   label          = !hide_label;

#include "show/url.h"

static GLuint urlTexture;

/**
 * Displays all data (i.e., points) that are to be displayed
 * @param mode spezification for drawing to screen or in selection mode
 */
void DrawPoints(GLenum mode, bool interruptable)
{
  long time = GetCurrentTimeInMilliSec();
  double min = 0.000000001;
  double max = 1.0;
  LevelOfDetail *= 1.0 + adaption_rate*(lastfps - idealfps)/idealfps;
  if (LevelOfDetail > max) LevelOfDetail = max;
  else if (LevelOfDetail < min) LevelOfDetail = min;

  // In case of animation
  if(frameNr != 0) {
    if (coloranim == 0) {
      cm->setMode(ScanColorManager::MODE_ANIMATION);
    }

    for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {

      // ignore scans that don't have any frames associated with them
      if((unsigned int)iterator >= MetaMatrix.size()) continue;

      // also ignore scans outside the selected range - if in advanced mode
      if (advanced_controls){
        // pay attention to offset (startScanIdx)
        if (iterator < startRangeScanIdx - startScanIdx) continue;
        if (iterator > endRangeScanIdx - startScanIdx) continue;
      }

      // set usable frame
      double* frame;
      Scan::AlgoType type;
      if((unsigned int)frameNr >= MetaMatrix[iterator].size()) {
        // use last possible frame
        frame = MetaMatrix[iterator].back();
        type = MetaAlgoType[iterator].back();
      } else {
        frame = MetaMatrix[iterator][frameNr];
        type = MetaAlgoType[iterator][frameNr];
      }
      if(type == Scan::INVALID) continue;
      cm->selectColors(type);
      glPushMatrix();
      glMultMatrixd(frame);


      glPointSize(pointsize);
        ExtractFrustum(pointsize);
        cm->selectColors(type);
        if (pointmode == 1 ) {
          octpts[iterator]->display();
        } else {
          octpts[iterator]->displayLOD(LevelOfDetail);
        }
      glPopMatrix();
    }

    setScansColored(0);

  } else {

    if (mode == GL_SELECT){
      // select points mode
      // ------------------
      GLuint name = 0;
      for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
        glPushMatrix();
        glMultMatrixd(MetaMatrix[iterator].back());

        glColor4f(1.0, 0.0, 0.0,1.0);
        glPointSize(pointsize + 2.0);
        for ( std::set<sfloat*>::iterator it = selected_points[iterator].begin();
            it != selected_points[iterator].end(); it++) {
          glLoadName(name++);
          glBegin(GL_POINTS);
          glVertex3d((*it)[0], (*it)[1], (*it)[2]);
          glEnd();
        }
        glPointSize(pointsize);

        glFlush();
        glPopMatrix();
      }

    } else {

      // draw point is normal mode
      // -------------------------

      if (interruptable) {
        glDrawBuffer (GL_FRONT);
      }
      glPointSize(pointsize);

      std::vector<int> sequence;
      calcPointSequence(sequence, current_frame);
      for(unsigned int i = 0; i < sequence.size(); i++) {
        int iterator = sequence[i];
        // ignore scans that don't have any frames associated with them
        if((unsigned int)iterator >= MetaMatrix.size()) continue;

        // also ignore scans outside the selected range - if in advanced mode
        if (advanced_controls){
          // pay attention to offset (startScanIdx)
          if (iterator < startRangeScanIdx - startScanIdx) continue;
          if (iterator > endRangeScanIdx - startScanIdx) continue;
        }

        // set usable frame
        double* frame;
        Scan::AlgoType type;
        if((unsigned int)current_frame >= MetaMatrix[iterator].size()) {
          // use last possible frame
          frame = MetaMatrix[iterator].back();
          type = MetaAlgoType[iterator].back();
        } else {
          frame = MetaMatrix[iterator][current_frame];
          type = MetaAlgoType[iterator][current_frame];
        }
        if (type == Scan::INVALID) continue;
        glPushMatrix();
        if (invert)
          // default: white points on black background
          glColor4d(1.0, 1.0, 1.0, 0.0);
        else
          // black points on white background
          glColor4d(0.0, 0.0, 0.0, 0.0);

        // glMultMatrixd(MetaMatrix[iterator].back());
        if (current_frame != (int)MetaMatrix.back().size() - 1) {
          if (coloranim == 0) {
            cm->setMode(ScanColorManager::MODE_ANIMATION);
            cm->selectColors(type);
          } else {
            setScansColored(0);
          }
        }
        glMultMatrixd(frame);

        ExtractFrustum(pointsize);
        if (pointmode == 1 ) {
          octpts[iterator]->display();
        } else if (interruptable) {
          checkForInterrupt();
          glFlush();
          glFinish();
          if (isInterrupted()) {
            glPopMatrix();
            return;
          }
          octpts[iterator]->display();
        } else {
          octpts[iterator]->displayLOD(LevelOfDetail);
        }
        if (!selected_points[iterator].empty()) {
          glColor4f(1.0, 0.0, 0.0, 1.0);
          glPointSize(pointsize + 2.0);
          glBegin(GL_POINTS);
          for ( std::set<sfloat*>::iterator it = selected_points[iterator].begin();
              it != selected_points[iterator].end(); it++) {
            glVertex3d((*it)[0], (*it)[1], (*it)[2]);
          }
          glEnd();
          glPointSize(pointsize);
        }

        glPopMatrix();
      }
    }
  }


  if (pointmode == 1 ) {
    fullydisplayed = true;
  } else {
    unsigned long td = (GetCurrentTimeInMilliSec() - time);
    if (td > 0)
      lastfps =  1000.0/td;
    else
      lastfps = 1000.0;
    fullydisplayed = false;
  }
  if (interruptable)
    fullydisplayed = true;
}


void DrawObjects(GLenum mode) {
  for (unsigned int i = 0; i < displays.size(); i++)
    displays[i]->displayAll();

}

/**
 * Draw a smooth path passing from all the camera points.
 *
 */
void DrawPath()
{

  glLineWidth(10.0);
  // draw path
  glBegin(GL_LINE_STRIP);
  for(unsigned int j = 0; j < path_vectorX.size(); j++){
    // set the color
    glColor4f(0.0, 1.0, 0.0, 1.0);
    // set the points
    glVertex3f(path_vectorX.at(j).x,path_vectorX.at(j).y,path_vectorZ.at(j).y);
  }
  glEnd();

  // draw lookat path
  glBegin(GL_LINE_STRIP);
  for(unsigned int j = 0; j < lookat_vectorX.size(); j++){
    //set the color
    glColor4d(1.0, 1.0, 0.0, 1.0);
    //set the points
    glVertex3f(lookat_vectorX.at(j).x,
               lookat_vectorX.at(j).y,
               lookat_vectorZ.at(j).y);
  }
  glEnd();

  // draw up path
  /*
  glBegin(GL_LINE_STRIP);
  for(unsigned int j = 0; j < ups_vectorX.size(); j++){
    //set the color
    glColor4d(0.0, 1.0, 0.0, 0.7);
    //set the points
    glVertex3f(ups_vectorX.at(j).x,ups_vectorX.at(j).y,ups_vectorZ.at(j).y);
  }
  glEnd();
  */
}

/**
 * Draw the camera boxes in the viewer
 *
 */
void DrawCameras(void)
{
  for (unsigned int i = 0; i < cams.size(); i++) {
    glPushMatrix();

 // TODO improve upon this primitive camera
    Point p = cams[i];
    Point l = Point::norm( lookats[i] - p );  // forward vector
    Point u = Point::norm( ups[i] - p );      // up vector
    Point r = Point::cross(l,u);        // right vector
    l = 5 * l;
    r = 5 * r;
    u = 5 * u;

    Point cube[8];
    cube[0] = p + l - r - u;
    cube[1] = p + l + r - u;
    cube[2] = p - l + r - u;
    cube[3] = p - l - r - u;
    cube[4] = p + l - r + u;
    cube[5] = p + l + r + u;
    cube[6] = p - l + r + u;
    cube[7] = p - l - r + u;

    int sides[6][4] = {{0,1,2,3}, {4,5,6,7}, {0,1,5,4},
                       {3,2,6,7}, {1,2,6,5}, {0,3,7,4}};

    if (i+1 == cam_choice) {
      glColor4f(1, 0, 1, 1);
    } else {
      glColor4f(0, 1, 0, 1);
    }
    // camera cube
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS);
      for (int j = 0; j < 6; j++) {
        if (j == 2) continue;
        for (int k = 0; k < 4; k++) {
          int index = sides[j][k];
          glVertex3d(cube[index].x, cube[index].y, cube[index].z);
        }
      }
    glEnd();

    r = 5 * r;
    u = 5 * u;

    glColor4f(1, 1, 0, 1);
    if (i+1 == cam_choice) {
      glPointSize(10);
    } else {
      glPointSize(5);
    }
    glBegin(GL_POINTS);
    glVertex3d( lookats[i].x,  lookats[i].y,  lookats[i].z);
    glEnd();

    Point fcube[8];
    fcube[0] = cube[4];
    fcube[1] = cube[5];
    fcube[2] = cube[1];
    fcube[3] = cube[0];
    fcube[4] = lookats[i] - r + u;
    fcube[5] = lookats[i] + r + u;
    fcube[6] = lookats[i] + r - u;
    fcube[7] = lookats[i] - r - u;

    glLineWidth(2.0);
    glLineStipple(1, 0x0C0F);
    glEnable(GL_LINE_STIPPLE);
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    // camera FOV
    glBegin(GL_QUADS);
      for (int j = 0; j < 6; j++) {
        for (int k = 0; k < 4; k++) {
          int index = sides[j][k];
          glVertex3d(fcube[index].x, fcube[index].y, fcube[index].z);
        }
      }
    glEnd();
    glDisable(GL_LINE_STIPPLE);

    glPopMatrix();
  }
}

void DrawScala() {
  if (factor != 1) return;

	glDisable(GL_FOG);
	if (showViewMode != 1) return;
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);

	// Save the current projection matrix
	glPushMatrix();
	// Make the current matrix the identity matrix
	glLoadIdentity();

	// Set the projection (to 2D orthographic)
	double z = 100.0;
	glOrtho(0.0,current_width,0.0,current_height,-z,z);

	double length = 200;
	double bigtick = 20;
	double smalltick = 10;

	double scala;
	int precision;
	// choose largest scala that fits on the screen, starting with 1 million
	for (int i = 10; i > -10; i--) {
		if (i >= 0 ) precision = 0;
		else precision = -i;
		scala = pow(10.0, (double)i) * 5.0; // check 50
		length = (scala * current_height ) / (2.0 * pzoom);
		if (length < 0.8 * current_height ) { break; }

		scala = pow(10.0, (double)i) * 2.0; // check 20
		length = (scala * current_height ) / (2.0 * pzoom);
		if (length < 0.8 * current_height ) { break; }

		scala = pow(10.0, (double)i) ; // check 10
		length = (scala * current_height ) / (2.0 * pzoom);
		if (length < 0.8 * current_height ) { break; }
	}
	char ymaxtext[100] = "";
	char yhalftext[100] = "";
	char yticktext[][100] = {"", "", "", ""};
	sprintf(ymaxtext, "%.*lf", precision, scala);
	if (scala < 10.0) precision += 1;
	sprintf(yhalftext, "%.*lf", precision, scala/2);
	for (int i = 1; i < 5; i++) {
		double tick = (scala / 5.0 ) * (double)i;
		sprintf(yticktext[i-1], "%.*lf", precision, tick);
	}

	int maxtext = std::max(strlen(ymaxtext), strlen(yhalftext));

	int textoffset = 5;
	int offset = textoffset + 8 * maxtext + 10;
	int yoffset = (current_height - length) / 2.0;
	int xoffset = (current_width - length) / 2.0;


	glLineWidth(1);
	glColor4d(1.0,0.0,0.0,0.4);
	glBegin(GL_LINES);
	glVertex3f(offset           , yoffset                  , z);
	glVertex3f(offset           , yoffset + length         , z);

	glVertex3f(offset           , yoffset                  , z);
	glVertex3f(offset + bigtick , yoffset                  , z); // start

	glVertex3f(offset           , yoffset + length         , z);
	glVertex3f(offset + bigtick , yoffset + length         , z); // end

	glVertex3f(offset           , yoffset + (length / 2.0) , z);
	glVertex3f(offset + bigtick , yoffset + (length / 2.0) , z); // middle

	for (int i = 1; i < 5; i++) {
		double tick = (length / 5.0 ) * (double)i;
		glVertex3f(offset             , yoffset + tick , z);
		glVertex3f(offset + smalltick , yoffset + tick , z);
	}
	glEnd();

	glBlendFunc(GL_ONE, GL_ZERO);
	glColor3f(1,1,1);
	glRasterPos3f(textoffset              , yoffset + length - 4     , z);
	_glutBitmapString(GLUT_BITMAP_8_BY_13 , ymaxtext);
	glRasterPos3f(textoffset              , yoffset + length/2.0 - 4 , z);
	_glutBitmapString(GLUT_BITMAP_8_BY_13, yhalftext);

	for (int i = 1; i < 5; i++) {
		double tick = (length / 5.0 ) * (double)i;
		glRasterPos3f(textoffset              , yoffset + tick -4 , z);
		_glutBitmapString(GLUT_BITMAP_8_BY_13 , yticktext[i-1]);
	}


	glColor4d(0.0,1.0,0.0,0.4);
	glBegin(GL_LINES);
	glVertex3f(xoffset                  , offset           , z);
	glVertex3f(xoffset + length         , offset           , z);

	glVertex3f(xoffset                  , offset           , z);
	glVertex3f(xoffset                  , offset + bigtick , z); // start

	glVertex3f(xoffset + length         , offset           , z);
	glVertex3f(xoffset + length         , offset + bigtick , z); // end

	glVertex3f(xoffset + (length / 2.0) , offset           , z);
	glVertex3f(xoffset + (length / 2.0) , offset + bigtick , z); // middle

	for (int i = 1; i < 5; i++) {
		double tick = (length / 5.0 ) * (double)i;
		glVertex3f( xoffset + tick ,offset             , z);
		glVertex3f( xoffset + tick ,offset + smalltick , z);
	}
	glEnd();

	/*
	   glBegin(GL_LINES);
	   glVertex3f(offset         , offset , z); // length
	   glVertex3f(offset+length , offset , z);
	   glEnd();
	   */

	// Restore the original projection matrix
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

}

void DrawCoordinateSystems() {
  for(int i = 0; (unsigned int)i < MetaMatrix.size(); i++){
    unsigned int j = MetaMatrix.size() - 1;
    // also ignore scans outside the selected range - if in advanced mode
    if (advanced_controls){
      // pay attention to offset (startScanIdx)
      if (i < startRangeScanIdx - startScanIdx) continue;
      if (i > endRangeScanIdx - startScanIdx) continue;
    }
    // set usable type
    Scan::AlgoType type;
    if((unsigned int)frameNr >= MetaMatrix[i].size()) {
      type = MetaAlgoType[i].back();
    } else {
      type = MetaAlgoType[i][frameNr];
    }
    if(frameNr >= 1 && frameNr < (int)MetaMatrix[i].size()) {
      if(type == Scan::INVALID) continue;
      // avoid incomplete frames in a scan
      if((unsigned int)frameNr >= MetaMatrix[i].size())
        j = MetaMatrix.size() - 1;
      else
        j = frameNr;
    } else {
      // avoid incomplete frames in a scan
      if((unsigned int)current_frame >= MetaMatrix[i].size()) {
        j = MetaMatrix.size() - 1;
      } else {
        j = current_frame;
      }
    }
    glColor3f(1.0, 0.3, 0.3);
    glLineWidth(5.0);
    glBegin(GL_LINES);

    float s = 10.0*MetaMatrix[i][j][15]*0.01/scale;
    glColor3f(1,0.0,0.0);
    glVertex3f(MetaMatrix[i][j][12], MetaMatrix[i][j][13], MetaMatrix[i][j][14]);
    glVertex3f(MetaMatrix[i][j][12] + s*MetaMatrix[i][j][0], MetaMatrix[i][j][13] + s*MetaMatrix[i][j][1], MetaMatrix[i][j][14] + s*MetaMatrix[i][j][2]);
    glColor3f(0.0,1.0,0.0);
    glVertex3f(MetaMatrix[i][j][12], MetaMatrix[i][j][13], MetaMatrix[i][j][14]);
    glVertex3f(MetaMatrix[i][j][12] + s*MetaMatrix[i][j][4], MetaMatrix[i][j][13] + s*MetaMatrix[i][j][5], MetaMatrix[i][j][14] + s*MetaMatrix[i][j][6]);
    glColor3f(0.0,0.0,1.0);
    glVertex3f(MetaMatrix[i][j][12], MetaMatrix[i][j][13], MetaMatrix[i][j][14]);
    glVertex3f(MetaMatrix[i][j][12] + s*MetaMatrix[i][j][8], MetaMatrix[i][j][13] + s*MetaMatrix[i][j][9], MetaMatrix[i][j][14] + s*MetaMatrix[i][j][10]);

    glEnd();
  }
}

void setup_camera() {

  // The camera projection is represented in OpenGL by the ModelView matrix
  glMatrixMode(GL_MODELVIEW);

  // Initialize ModelView with identity
  glLoadIdentity();

  if(showViewMode == 2) {
    gluLookAt(0, 0, rzoom, 0, 0, 0, 0, 1, 0);
    double dx = (double)(mouseNavX - mousePresX) / current_width;
    double dy = (double)(mouseNavY - mousePresY) / current_height;
    double a = sqrt(dx * dx + dy * dy);
    static double rot_mat[16] = {
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    };

    if(a != 0.0) {
      double ar = a * M_PI;
      double as = sin(ar) / a;
      double delta_quat[4] = {cos(ar), dy * as, dx * as, 0.0};

      QMult(delta_quat, now_quat, tmp_quat);
      QuatToMatrix4(tmp_quat, 0, rot_mat);
    }

    glMultMatrixd(rot_mat);

    if (!nogui && !takescreenshot)
      updateControls();

    return;
  }

  // do the model-transformation
  if (haveToUpdate == 8 && path_iterator < ups.size()) {
          gluLookAt(cams.at(path_iterator).x,
                    cams.at(path_iterator).y,
                    cams.at(path_iterator).z,
                    lookats.at(path_iterator).x,
                    lookats.at(path_iterator).y,
                    lookats.at(path_iterator).z,
                    ups.at(path_iterator).x-cams.at(path_iterator).x,
                    ups.at(path_iterator).y-cams.at(path_iterator).y,
                    ups.at(path_iterator).z-cams.at(path_iterator).z);
      } else if (haveToUpdate == 6 && path_iterator < path_vectorX.size()) {

    if(path3D) {
      double * lc = new double[3];
      double * uc = new double[3];
      double * n = new double[3];

      Point l(lookat_vectorX.at(path_iterator).x,
              lookat_vectorX.at(path_iterator).y,
              lookat_vectorZ.at(path_iterator).y);
      Point p(path_vectorX.at(path_iterator).x,
              path_vectorX.at(path_iterator).y,
              path_vectorZ.at(path_iterator).y);
      Point u(ups_vectorX.at(path_iterator).x - path_vectorX.at(path_iterator).x,
              ups_vectorX.at(path_iterator).y - path_vectorX.at(path_iterator).y,
              ups_vectorZ.at(path_iterator).y - path_vectorZ.at(path_iterator).y);

      lc[0] = l.x - p.x;
      lc[1] = l.y - p.y;
      lc[2] = l.z - p.z;

      uc[0] = u.x;
      uc[1] = u.y;
      uc[2] = u.z;

      Cross(lc,uc,n);
      Normalize3(n);
      p.x = p.x + n[0] * shifted;
      p.y = p.y + n[1] * shifted;
      p.z = p.z + n[2] * shifted;
      l.x = l.x + n[0] * shifted;
      l.y = l.y + n[1] * shifted;
      l.z = l.z + n[2] * shifted;

      delete[] lc;
      delete[] uc;
      delete[] n;

      gluLookAt(p.x,p.y,p.z,l.x,l.y,l.z,u.x,u.y,u.z);

    } else {

      gluLookAt(path_vectorX.at(path_iterator).x,
                path_vectorX.at(path_iterator).y,
                path_vectorZ.at(path_iterator).y,
                lookat_vectorX.at(path_iterator).x,
                lookat_vectorX.at(path_iterator).y,
                lookat_vectorZ.at(path_iterator).y,
                ups_vectorX.at(path_iterator).x-path_vectorX.at(path_iterator).x,
                ups_vectorX.at(path_iterator).y-path_vectorX.at(path_iterator).y,
                ups_vectorZ.at(path_iterator).y-path_vectorZ.at(path_iterator).y);
    }
  } else {
    if (cameraNavMouseMode == 1) {
      glRotated( mouseRotX, 1, 0, 0);
      glRotated( mouseRotY, 0, 1, 0);
      glRotated( mouseRotZ, 0, 0, 1);
      glGetFloatv(GL_MODELVIEW_MATRIX, view_rotate_button);
      update_view_rotate(0);
    } else {
      double t[3] = {0,0,0};
      double mat[16];
      QuatToMatrix4((const double *)quat, t, mat);
      glMultMatrixd(mat);

      glGetFloatv(GL_MODELVIEW_MATRIX, view_rotate_button);
      // rotate the camera
      double rPT[3];
      Matrix4ToEuler(mat, rPT);
      mouseRotX = deg(rPT[0]);
      mouseRotY = deg(rPT[1]);
      mouseRotZ = deg(rPT[2]);

    }
    if (!nogui && !takescreenshot)
      updateControls();

    glTranslated(X, Y, Z);       // move camera
  }

#ifdef WITH_LOGGING
  std::clog << "Position  :" << X << " " << Y << " " << Z << '\n'
            << "Quaternion:" << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << '\n'
            << "Axis/Angle:" << axis[0] << " " << axis[1] << " " << axis[2] << " " << angle << '\n'
            << "Apex angle:" << cangle << '\n' << std::endl;
#endif
}

void setup_fog() {
  if (show_fog > 0) {
    GLfloat fogColor[4];

    if (show_fog > 3) // One of the "inverted fog" options
      fogColor[0] = fogColor[1] = fogColor[2] = fogColor[3] = 1.0;
    else
      fogColor[0] = fogColor[1] = fogColor[2] = fogColor[3] = 0.0;

    // by setting the fardistance there we gain some performance

    // GL fog modes for show_fog = (1 & 4), (2 & 5), (3 & 6)
    int fogModes[3] = { GL_EXP, GL_EXP2, GL_LINEAR };
    // fogMode can then be looked up in that array, regardless of "inverted fog" (% 3)
    fogMode = fogModes[(show_fog-1) % 3];

    // By clipping the fardistance we hope to gain some performance
    switch (show_fog) {
    case 1:
      // ln(1/2^8) = -5.54517744 is the threshold at which the last color bit is gone due to fog
      fardistance = std::min(5.54517744 / fogDensity, (double)maxfardistance);
      break;
    case 2:
      fardistance = std::min(sqrt(5.54517744) / fogDensity, (double)maxfardistance);
      break;
    case 3:
      fardistance = 32000.0;
      break;
    default:
      fardistance = (double)maxfardistance;
    }

    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, fogMode);
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_DENSITY, fogDensity);
    glHint(GL_FOG_HINT, GL_FASTEST);
    glFogf(GL_FOG_START, neardistance);
    glFogf(GL_FOG_END, maxfardistance);
  } else {
    glDisable(GL_FOG);
    fardistance = maxfardistance;
  }
}

/**
 * Display function
 */
void DisplayItFunc(GLenum mode, bool interruptable)
{
  // set the clear color buffer in case of
  // both invert and non invert mode
  if (invert) {
    glClearColor(bgcolor[0], bgcolor[1], bgcolor[2], 1.0);
  } else {
    glEnable(GL_COLOR_LOGIC_OP);
    glLogicOp(GL_COPY_INVERTED);

    glClearColor(1-bgcolor[0], 1-bgcolor[1], 1-bgcolor[2], 1.0);
  }

  // clear the color and depth buffer bit
  if (!interruptable) { // single buffer mode, we need the depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  // set the polygon mode
  // XXX please clarify why we need this and what GL_FRONT_AND_BACK would do
  glPolygonMode(GL_FRONT/*_AND_BACK*/, GL_LINE);

  glPushMatrix();

  setup_camera();

  DrawScala();

  // process fog
  setup_fog();

  if (fardistance > maxfardistance) fardistance = maxfardistance;
  if ( fabs(oldfardistance - fardistance) > 0.00001 ||
       fabs(oldneardistance - neardistance) > 0.00001 ) {
    oldfardistance = fardistance;
    oldneardistance = neardistance;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    reshape(viewport[2], viewport[3]);
  }

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //
  // show the objects __after__ the model-transformation
  // for all status variables we show the appropiated thing
  // using the drawing functions
  //
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  if (show_path == 1) {
    double *pose;
    glColor4d(1.0, 0.0, 0.0, 1.0);
    glLineWidth(5);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; (unsigned int)i < MetaMatrix.size(); i++){

      // also ignore scans outside the selected range - if in advanced mode
      if (advanced_controls){
        // pay attention to offset (startScanIdx)
        if (i < startRangeScanIdx - startScanIdx) continue;
        if (i > endRangeScanIdx - startScanIdx) continue;
      }
      // set usable type
      Scan::AlgoType type;
      if((unsigned int)frameNr >= MetaMatrix[i].size()) {
        type = MetaAlgoType[i].back();
      } else {
        type = MetaAlgoType[i][frameNr];
      }
      if(frameNr >= 1 && frameNr < (int)MetaMatrix[i].size()) {
        if(type == Scan::INVALID) continue;
        // avoid incomplete frames in a scan
        if((unsigned int)frameNr >= MetaMatrix[i].size())
          pose = MetaMatrix[i].back();
        else
          pose = MetaMatrix[i][frameNr];
      } else {
        //pose = MetaMatrix[i].back();
        // avoid incomplete frames in a scan
        if((unsigned int)current_frame >= MetaMatrix[i].size())
          pose = MetaMatrix[i].back();
        else
          pose = MetaMatrix[i][current_frame];
      }
      if(showViewMode == 1) {
        glVertex3f(pose[12], 2000, pose[14]);
      } else {
        glVertex3f(pose[12], pose[13], pose[14]);
      }
    }
    glEnd();
  }

  // Draw trajectory from file
  if (trajectory.size() > 0) {
    glColor4d(1.0, 0.0, 0.0, 1.0);
    glLineWidth(5);
    glBegin(GL_LINE_STRIP);

    for (size_t i = 0; i < trajectory.size(); i++) {
      glVertex3f(trajectory.at(i)[0], trajectory.at(i)[1], trajectory.at(i)[2]);
    }

    glEnd();
  }

  // if show camera is true then draw cameras.
  if (show_cameras == 1) {
    DrawCameras();
  }

  // if show path is true the draw path.
  if (show_path == 1) {
    DrawPath();
  }
  // if show poses is true then draw coordinate axes.
  if (show_poses == 1) {
    DrawCoordinateSystems();
  }
  DrawObjects(mode);

  // if show points is true the draw points
  if (show_points == 1) DrawPoints(mode, interruptable);


  if (label) DrawUrl();

  glPopMatrix();

  if (!invert) {
    glDisable(GL_COLOR_LOGIC_OP);
  }

  // force draw the scene
  glFlush();
  glFinish();
}

void DrawUrl() {
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);

  // Save the current projection matrix
  glPushMatrix();
  // Make the current matrix the identity matrix
  glLoadIdentity();

  // Set the projection (to 2D orthographic)
  glOrtho(0,current_width,0,current_height,-1,1);

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor4d(1,1,1,0.8);

  glEnable(GL_TEXTURE_2D);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  glBindTexture(GL_TEXTURE_2D, urlTexture);

  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0); glVertex3f(0,0,1);
  glTexCoord2f(0.0, 1.0); glVertex3f(0,urlTextureHeight,1);
  glTexCoord2f(1.0, 1.0); glVertex3f(urlTextureWidth,urlTextureHeight,1);
  glTexCoord2f(1.0, 0.0); glVertex3f(urlTextureWidth,0,1);
  glEnd();

  glBlendFunc(GL_ONE, GL_ZERO);
  glColor3f(1,1,1);

  glDisable(GL_TEXTURE_2D);

  // Restore the original projection matrix
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

/**
 * Function topview. Set the screen for top view.
 */
void topView()
{
  static GLdouble save_qx, save_qy, save_qz, save_qangle;
  static GLdouble save_X, save_Y, save_Z;
  static GLdouble saveMouseRotX, saveMouseRotY, saveMouseRotZ;

  if (showViewMode != 1) // set to top view
  {
    showViewMode = 1;

    // save current pose
    save_X      = X;
    save_Y      = Y;
    save_Z      = Z;
    save_qx     = quat[0];
    save_qy     = quat[1];
    save_qz     = quat[2];
    save_qangle = quat[3];
    saveMouseRotX = mouseRotX;
    saveMouseRotY = mouseRotY;
    saveMouseRotZ = mouseRotZ;

    Y = Y - 350.0;
    Z = Z + 500.0;
    quat[0] = quat[1] = sqrt(0.5);
    quat[2] = quat[3] = 0.0;
    mouseRotX = 90;
    mouseRotY = 0;
    mouseRotZ = 0;

    haveToUpdate = 2;

  } else {

    showViewMode = 0;

    // restore old settings
    X = save_X;
    Y = save_Y;
    Z = save_Z;
    quat[0] = save_qx;
    quat[1] = save_qy;
    quat[2] = save_qz;
    quat[3] = save_qangle;
    mouseRotX = saveMouseRotX;
    mouseRotY = saveMouseRotY;
    mouseRotZ = saveMouseRotZ;

    haveToUpdate = 2;
  }
}

/**
 * Funcition rotateview. Set the screen for rotate view.
 */
void rotateView()
{
  static GLdouble save_qx, save_qy, save_qz, save_qangle;
  static GLdouble save_X, save_Y, save_Z;
  static GLdouble saveMouseRotX, saveMouseRotY, saveMouseRotZ;

  if(showViewMode != 2) {
    showViewMode = 2;

    save_X      = X;
    save_Y      = Y;
    save_Z      = Z;
    save_qx     = quat[0];
    save_qy     = quat[1];
    save_qz     = quat[2];
    save_qangle = quat[3];
    saveMouseRotX = mouseRotX;
    saveMouseRotY = mouseRotY;
    saveMouseRotZ = mouseRotZ;

    haveToUpdate = 2;
  } else {
    showViewMode = 0;

    // restore old settings
    X = save_X;
    Y = save_Y;
    Z = save_Z;
    quat[0] = save_qx;
    quat[1] = save_qy;
    quat[2] = save_qz;
    quat[3] = save_qangle;
    mouseRotX = saveMouseRotX;
    mouseRotY = saveMouseRotY;
    mouseRotZ = saveMouseRotZ;

    haveToUpdate = 2;
  }
}


//---------------------------------------------------------------------------
/**
 * This function is called when the user wants to
 * delete a camera.
 */
void callDeleteCamera(int dummy){

  // iterator for the position of camera
  // in the camera list
  std::vector<Point>::iterator position;
  std::vector<Point>::iterator positionL;
  std::vector<Point>::iterator positionU;

  // calculate the position of the camera. we are referring
  // to the selected camera
  position = cams.begin()+ (cam_choice-1);
  positionL = lookats.begin()+ (cam_choice-1);
  positionU = ups.begin()+ (cam_choice-1);

  // if no camera present then return
  if(cam_choice == 0)
    return;

  // if the list is not empty then
  if(!cams.empty()){
    // delete the camera from the position
    cams.erase(position);
    lookats.erase(positionL);
    ups.erase(positionU);
    // reset the cam_choice spinner values
  }

  updateCamera();
}


//---------------------------------------------------------------------------
/**
 * Function to reset the viewer window.
 */

void resetView(int dummy)
{
  cangle = 60.0;
  pzoom = defaultZoom;
  X = RVX;
  Y = RVY;
  Z = RVZ;
  quat[0] = Rquat[0];
  quat[1] = Rquat[1];
  quat[2] = Rquat[2];
  quat[3] = Rquat[3];
  haveToUpdate = 2;
  mouseRotX = 0;
  mouseRotY = 0;
  mouseRotZ = 0;

  resetRotationButton();
}

/**
 * Function to set the viewer window back to a previously saved state.
 */
void setView(double pos[3], double new_quat[4],
             double newMouseRotX, double newMouseRotY, double newMouseRotZ,
             double newCangle,
             int sVM, bool cNMM, double pzoom_new,
             bool s_points, bool s_path, bool s_cameras, bool s_poses, double ps, int
             sf, double fD, bool inv)
{
  X = pos[0];
  Y = pos[1];
  Z = pos[2];
  for(int i = 0; i < 4; i++) {
    quat[i] = new_quat[i];
  }
  cangle = newCangle;
  mouseRotX = newMouseRotX;
  mouseRotY = newMouseRotY;
  mouseRotZ = newMouseRotZ;
  showViewMode = sVM,
  cameraNavMouseMode = cNMM;
  pzoom = pzoom_new;
  updateViewModeControls();

  show_points = s_points;
  show_path = s_path;
  show_cameras = s_cameras;
  show_poses = s_poses;
  pointsize = ps;
  show_fog = sf;
  fogDensity = fD;
  invert = inv;

  haveToUpdate = 2;
}


/**
 * This function handles the rotation of the view
 */
void update_view_rotate(int t)
{
  double view_rotate_button_quat[4];

  // convert the rotate button matrix to quaternion
  double mat[16];
  for (int i = 0; i < 16; i++)
    mat[i] = view_rotate_button[i];
  Matrix4ToQuat(mat, view_rotate_button_quat);

  // normalize the quartenion
  QuatNormalize(view_rotate_button_quat);

  if(view_rotate_button_quat[0] == 1.0) return;
  // copy it to the global quartenion quat
  memcpy(quat, view_rotate_button_quat, sizeof(quat));
}

/**
 * This function handles the translation of view.
 */
void update_view_translation(int t)
{
  double obj_pos_button1[3];

  for (int i = 0; i < 3; i++) {
    if (fabs(obj_pos_button_old[i] - obj_pos_button[i]) > COMPARE_EPSILON) {
      obj_pos_button1[i] = obj_pos_button[i] - obj_pos_button_old[i];
      obj_pos_button_old[i] = obj_pos_button[i];
    } else obj_pos_button1[i] = 0.0;
  }

  X = X + obj_pos_button1[0] * view_rotate_button[0]
    + obj_pos_button1[1] * view_rotate_button[1]
    + obj_pos_button1[2] * view_rotate_button[2];
  Y = Y + obj_pos_button1[0] * view_rotate_button[4]
    + obj_pos_button1[1] * view_rotate_button[5]
    + obj_pos_button1[2] * view_rotate_button[6];
  Z = Z + obj_pos_button1[0] * view_rotate_button[8]
    + obj_pos_button1[1] * view_rotate_button[9]
    + obj_pos_button1[2] * view_rotate_button[10];
}


/**
 * handles the animation button
 * @param dummy not needed necessary for glui
 */
void startAnimation(int dummy)
{
  if (MetaMatrix.size() > 0) {
    if (haveToUpdate != 3) {
      haveToUpdate = 3;
    } else // stop animation
      haveToUpdate = 4;
  }
}

/**
 * calls the resetView function
 * @param dummy not needed necessary for glui
 */
void callResetView(int dummy)
{
  if (showViewMode == 1) callTopView(dummy);
  else if(showViewMode == 2) callRotateView(dummy);
  resetView(0);
}

/**
 * calls the resetView function
 * @param dummy not needed necessary for glui
 */
void invertView(int dummy)
{
  invert = !invert;
}

/**
 * calls the topView function
 * @param dummy not needed necessary for glui
 */
void callTopView(int dummy)
{
  topView();
  if (showViewMode == 1) {
    rotButton->disable();
    cangle_spinner->disable();
    pzoom_spinner->enable();
    rzoom_spinner->disable();
  } else {
    rotButton->enable();
    cangle_spinner->enable();
    pzoom_spinner->disable();
    rzoom_spinner->disable();
  }
}

/**
 * calls the rotateView function
 * @param dummy not needed necessary for glui
 */
void callRotateView(int dummy)
{
  rotateView();
  if(showViewMode == 2) {
    cangle_spinner->disable();
    pzoom_spinner->disable();
    rzoom_spinner->enable();
  } else {
    cangle_spinner->enable();
    pzoom_spinner->disable();
    rzoom_spinner->disable();
  }
}

/**
 * calls the cameraView function
 * @param dummy not needed necessary for glui
 */
void callAddCamera(int dummy)
{
  Point campos(-X, -Y, -Z);

  // calculate lookat point
  Point lookat;
  Point up(0, 0, 0);
  double tmat[16];
  for (int i = 0; i < 16; i++) tmat[i] = view_rotate_button[i];

  lookat.x = -50*tmat[2]  - X;
  lookat.y = -50*tmat[6]  - Y;
  lookat.z = -50*tmat[10] - Z;

  up.x = 50*tmat[1] -X;
  up.y = 50*tmat[5] -Y;
  up.z = 50*tmat[9] -Z;

  cams.push_back(campos);
  lookats.push_back(lookat);
  ups.push_back(up);

  updateCamera();

  // signal to repaint screen
  haveToUpdate  = 1;
}

void selectPoints(int x, int y) {
	// TODO smaller buffer ok for all OS?
#ifdef _MSC_VER
	GLuint selectBuf[BUFSIZE_WIN];
#else
  GLuint selectBuf[BUFSIZE];
#endif
  GLint hits;
  GLint viewport[4];
  if (selectOrunselect) {
    // set the matrix mode
    glMatrixMode(GL_MODELVIEW);
    // init modelview matrix
    glLoadIdentity();

    // do the model-transformation
  if (cameraNavMouseMode == 1) {
    glRotated( mouseRotX, 1, 0, 0);
    glRotated( mouseRotY, 0, 1, 0);
    glRotated( mouseRotZ, 0, 0, 1);
  } else {
    double t[3] = {0,0,0};
    double mat[16];
    QuatToMatrix4((const double *)quat, t, mat);
    glMultMatrixd(mat);

    glGetFloatv(GL_MODELVIEW_MATRIX, view_rotate_button);
    double rPT[3];
    Matrix4ToEuler(mat, rPT);
    mouseRotX = deg(rPT[0]);
    mouseRotY = deg(rPT[1]);
    mouseRotZ = deg(rPT[2]);
  }
  updateControls();
    glTranslated(X, Y, Z);       // move camera

    static sfloat *sp2 = 0;
    for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {

      // ignore scans outside the selected (currently visible) range - if in advanced mode
      if (advanced_controls){
        // pay attention to offset (startScanIdx)
        if (iterator < startRangeScanIdx - startScanIdx) continue;
        if (iterator > endRangeScanIdx - startScanIdx) continue;
      }

      glPushMatrix();
      glMultMatrixd(MetaMatrix[iterator].back());
      calcRay(x, y, 1.0, 40000.0);
      if (select_voxels) {
        octpts[iterator]->selectRay(selected_points[iterator], selection_depth);
      } else if (brush_size == 0) {
        sfloat *sp = 0;
        octpts[iterator]->selectRay(sp);
        if (sp != 0) {
          std::cout << "Selected point: "
               << sp[0] << " " << sp[1] << " " << sp[2] << std::endl;

          if (sp2 != 0) {
            std::cout << "Distance to last point: "
                 << sqrt( sqr(sp2[0] - sp[0]) +
                          sqr(sp2[1] - sp[1]) +
                          sqr(sp2[2] - sp[2])  )
                 << std::endl;
          }
          sp2 = sp;

          selected_points[iterator].insert(sp);
        }
      } else { // select multiple points with a given brushsize
        octpts[iterator]->selectRayBrushSize(selected_points[iterator],
                                             brush_size);
      }

      glPopMatrix();
    }

  } else {
    // unselect points
    glGetIntegerv(GL_VIEWPORT, viewport);

#ifdef _MSC_VER
	glSelectBuffer(BUFSIZE_WIN, selectBuf);
#else
    glSelectBuffer(BUFSIZE, selectBuf);
#endif
    (void) glRenderMode(GL_SELECT);

    glInitNames();
    glPushName(0);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    gluPickMatrix((GLdouble)x, (GLdouble)(viewport[3]-y),
                  brush_size*2, brush_size*2,
                  viewport);
    gluPerspective(cangle, aspect, neardistance, fardistance);
    glMatrixMode(GL_MODELVIEW);
    DisplayItFunc(GL_SELECT);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    hits = glRenderMode(GL_RENDER);                       // get hits
    ProcessHitsFunc(hits, selectBuf);
  }
  glPopMatrix();
  update_callback();
}


void moveCamera(double x, double y, double z,
                double rotx, double roty, double rotz) {
  interruptDrawing();
  double mat[9];

  double xr = M_PI * mouseRotX / 180;
  double yr = M_PI * mouseRotY / 180;
  double zr = M_PI * mouseRotZ / 180;
  double c1,c2,c3,s1,s2,s3;
  s1 = sin(xr);  c1 = cos(xr);
  s2 = sin(yr);  c2 = cos(yr);
  s3 = sin(zr);  c3 = cos(zr);
  mat[0] = c2*c3;
  mat[1] = -c2*s3;
  mat[2] = s2;
  mat[3] = c1*s3+c3*s1*s2;
  mat[4] = c1*c3-s1*s2*s3;
  mat[5] = -c2*s1;
  mat[6] = s1*s3-c1*c3*s2;
  mat[7] = c1*s2*s3+c3*s1;
  mat[8] = c1*c2;

  double transX, transY, transZ;
  transX = transY = transZ = 0.0;

  mouseRotX += rotx;
  mouseRotY -= roty;
  mouseRotZ -= rotz;

  if (mouseRotX < -90) mouseRotX=-90;
  else if (mouseRotX > 90) mouseRotX=90;
  if (mouseRotY > 360) mouseRotY-=360;
  else if (mouseRotY < 0) mouseRotY+=360;
  if (mouseRotZ > 360) mouseRotZ-=360;
  else if (mouseRotZ < 0) mouseRotZ+=360;

  transX += x * mat[0] + y * mat[3] + z * mat[6];
  transY += x * mat[1] + y * mat[4] + z * mat[7];
  transZ += x * mat[2] + y * mat[5] + z * mat[8];

  X += transX;
  Y += transY;
  Z += transZ;
  haveToUpdate = 1;

}

void initScreenWindow()
{
  // init display
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);

  // define the window position and size
  glutInitWindowPosition(START_X, START_Y);
  glutInitWindowSize( START_WIDTH, START_HEIGHT );

  // create window and append callback functions
  window_id = glutCreateWindow("3D_Viewer");

  glutDisplayFunc(display);
  GLUI_Master.set_glutReshapeFunc(reshape);

  GLUI_Master.set_glutMouseFunc(mouseButton);
  GLUI_Master.set_glutKeyboardFunc(keyPressed);
  glutKeyboardUpFunc(keyReleased);
  glutMotionFunc(mouseMove);
#ifdef __APPLE__
  glutWMCloseFunc(quit);
#else
  glutCloseFunc(quit);
#endif

  // glutEntryFunc ( CallBackEntryFunc);
  GLUI_Master.set_glutReshapeFunc(reshape);
  GLUI_Master.set_glutIdleFunc(idle);

  update_view_rotate(0);
  glClearColor(0.0, 0.0, 0.0, 0.0);

  load_url_texture();
}

void load_url_texture() {
  glGenTextures(1, &urlTexture);
  glBindTexture(GL_TEXTURE_2D, urlTexture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, urlTextureWidth,
		  urlTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE,
		  urlTextureData);
}

/* +++++++++-------------++++++++++++
 * NAME
 *   glDumpWindowPPM
 * DESCRIPTION
 *   writes an ppm file of the window
 *   content
 * PARAMETERS
 *   filename
 * RESULT
 *  writes the framebuffer content
 *  to a ppm file
+++++++++-------------++++++++++++ */
void glDumpWindowPPM(const char *filename, GLenum mode)
{
  int win_height, win_width;
  int i,j,k,l;                  // Counter variables
  GLubyte *buffer;              // The GL Frame Buffer
  unsigned char *ibuffer;       // The PPM Output Buffer
  std::ofstream fp;                  // The PPM File

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  win_height = viewport[3];
  win_width  = viewport[2];

  // Allocate memory for the the frame buffer and output buffer
  buffer = new GLubyte[win_width * win_height * RGBA_];
  ibuffer = new unsigned char[win_width * win_height * RGB_];

  // Read window contents from GL frame buffer with glReadPixels
  glFinish();
  glReadBuffer(buffermode);
  glReadPixels(0, 0, win_width, win_height,
                        GL_RGBA, GL_UNSIGNED_BYTE, buffer);

  // Open the output file
  fp.open(filename, std::ios::out);

  // Write a proper P6 PPM header
  fp << "P6" << std::endl
     << "# CREATOR: 3D_Viewer by Andreas Nuechter, University of Osnabrueck"
     << std::endl << win_width  << " " << win_height << " " << UCHAR_MAX << std::endl;

  // Loop through the frame buffer data, writing to the PPM file.  Be careful
  //   to account for the frame buffer having 4 bytes per pixel while the
  //   output file has 3 bytes per pixel
  l = 0;
  for (i = 0; i < win_height; i++) {     // For each row
    for (j = 0; j < win_width; j++) {    // For each column
      for (k = 0; k < RGB_; k++) {        // For each RGB component
        //cout << (RGBA_*((win_height-1-i)*win_width+j)+k) << endl;
        ibuffer[l++] = (unsigned char)
          *(buffer + (RGBA_*((win_height-1-i)*win_width+j)+k));
      }                                  // end RGB
    }                                    // end column
  }                                      // end row

  // to make a video do:
  // for f in *ppm ; do convert -quality 100 $f `basename $f ppm`jpg; done
  // mencoder "mf://*.jpg" -mf fps=10 -o test.avi -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=800
  // Write output buffer to the file */
  fp.write((const char*)ibuffer, sizeof(unsigned char) * (RGB_ * win_width * win_height));
  fp.close();
  fp.clear();
  delete [] buffer;
  delete [] ibuffer;
}

void writePngInBackground(const char *filename, unsigned char *ibuffer, int image_width, int image_height)
{
    // write contents of ibuffer to a png file
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png)
        return;

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, &info);
        return;
    }

    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        png_destroy_write_struct(&png, &info);
        return;
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info, image_width, image_height, 8 /* depth */, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
    png_write_info(png, info);
    png_set_packing(png);

    png_bytepp rows = (png_bytepp)png_malloc(png, image_height * sizeof(png_bytep));
    for (int i = 0; i < image_height; ++i)
        rows[i] = (png_bytep)(ibuffer + i * image_width * 3);

    png_write_image(png, rows);
    png_write_end(png, info);
    png_destroy_write_struct(&png, &info);

    fclose(fp);
    delete[] rows;
    delete[] ibuffer;
    delete[] filename;

	{
		std::unique_lock<std::mutex> lock(png_workers_mutex);
		png_workers -= 1;
		png_workers_cv.notify_one();
	}
}

/* +++++++++-------------++++++++++++
 * NAME
 *   glWriteImagePNG
 * DESCRIPTION
 *   writes a png file of the window
 *   content
 *   size is scale times the window size
 * PARAMETERS
 *   filename
 * RESULT
 *  writes the framebuffer content
 *  to a png file
+++++++++-------------++++++++++++ */
void glWriteImagePNG(const char *filename, int scale, GLenum mode)
{
  //if(!showTopView) {
    int m,o,k;                  // Counter variables
    // Get viewport parameters
    double left, right, top, bottom;
    double tmp = 1.0/tan(rad(cangle)/2.0);

    // Save camera parameters
    GLdouble savedMatrix[16];
    glGetDoublev(GL_PROJECTION_MATRIX,savedMatrix);
    GLdouble savedModel[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,savedModel);
    //glMatrixMode(GL_PROJECTION);
    //glPushMatrix();
    //glMatrixMode(GL_MODELVIEW);
    //glPushMatrix();
    top = 1.0/tmp;
    bottom = -top;
    right = (aspect)/tmp;
    left = -right;

    double part_height, part_width;
    if(showViewMode != 1) {
      part_width = (right - left)/(double)scale;
      part_height = (top - bottom)/(double)scale;
    } else {
      part_height = (2*pzoom)/scale;
      part_width = (2*pzoom*aspect)/scale;
      std::cout << part_width << " " << part_height << std::endl;
    }
    // Calculate part parameters
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    int win_width = viewport[2];
    int win_height = viewport[3];
    int image_width = scale * win_width;
    int image_height = scale * win_height;

    // Allocate memory for the the frame buffer and output buffer
    GLubyte *buffer;              // The GL Frame Buffer
    unsigned char *ibuffer;       // The PNG Output Buffer
    buffer = new GLubyte[win_width * win_height * RGBA_];
    ibuffer = new unsigned char[image_width * image_height * RGB_];

    smallfont = (scale==1);
    double height;
    if(showViewMode != 1) {
      height = bottom;
    } else {
      height = -pzoom;
    }
    for(int i = 0; i < scale; i++) {
      double width;
      if(showViewMode != 1) {
        width = left;
      } else {
        width = -pzoom*aspect;
      }
      for(int j = 0; j < scale; j++) {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        label = false;
        if(showViewMode != 1) {
          glFrustum(neardistance*width, neardistance*(width + part_width),
                    neardistance*(height),
                    neardistance*(height + part_height),
                    neardistance, fardistance);
          glMatrixMode(GL_MODELVIEW);
          if(i==0 && j==0) {
            label = !hide_label;
          }
          DisplayItFunc(mode);
        } else {
          glOrtho( width, width + part_width,
                   height, height + part_height,
                   1.0, 32000.0 );
          glMatrixMode(GL_MODELVIEW);
          if(i==0 && j==0) {
            label = !hide_label;
          }
          DisplayItFunc(mode);
        }

        // Read window contents from GL frame buffer with glReadPixels
        glFinish();
        glReadBuffer(buffermode);
        glReadPixels(0, 0,
                     win_width, win_height,
                     GL_RGBA, GL_UNSIGNED_BYTE,
                     buffer);

        // Loop through the frame buffer data, writing to the PNG file.
        // Be careful
        // to account for the frame buffer having 4 bytes per pixel while the
        // output file has 3 bytes per pixel
                                          // end row
        for (m = 0; m < win_height; m++) {     // For each row
          for (o = 0; o < win_width; o++) {    // For each column
            for (k = 0; k < RGB_; k++) {        // For each RGB component
              int l = (k + RGB_*(image_width*((scale - 1 - i)*win_height + m)
                                + j*win_width + o));
              ibuffer[l]   = (unsigned char)*(buffer +
                                              (RGBA_ * ((win_height-1-m)
                                                       * win_width+o)+k));
            }                                  // end RGB
          }                                    // end column
        }
        width += part_width;
      }
      height += part_height;
    }

    // show the starting scene
    // Restore the original projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(savedMatrix);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(savedModel);
    // show the rednered scene
    label = !hide_label;
    smallfont = true;
    haveToUpdate=2;
    DisplayItFunc(mode);

    delete [] buffer;

	{
		std::unique_lock<std::mutex> lock(png_workers_mutex);
		png_workers_cv.wait(lock, []{return png_workers <= png_workers_max;});
		png_workers += 1;
	}

    // make a copy of filename or otherwise the memory behind it might be deleted by the parent
    std::thread {writePngInBackground, strdup(filename), ibuffer, image_width, image_height}.detach();
}

/**
 *  Prints out which points were clicked on
 */
void ProcessHitsFunc(GLint hits, GLuint buffer[])
{
  // cout << "SIZE " << selected_points[0].size() << endl;
  // cout << "processing " << endl;
  std::set<int> names;
  std::set<sfloat *> unsel_points;

  GLuint *ptr, nr_names;

  ptr = (GLuint *)buffer;

  for(int i = 0 ; i < hits ; i++) {
    nr_names = *ptr;
    ptr+=3; // skip 2 z values
    for(unsigned int j = 0;j < nr_names ; j++){ // iterate over all names
      names.insert(*ptr);

      ptr++;
    }
  }
  // cout << "number of names " << names.size() << endl;
  if (names.empty()) return;

  int index = 0;
  std::set<int>::iterator nit = names.begin();
  // find the respective name

  for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
    // iterate over the selected points as in DrawPoints
    for ( std::set<sfloat*>::iterator it = selected_points[iterator].begin();
        it != selected_points[iterator].end(); it++) {
      if (index == *nit) { // if the current index is the current name
        unsel_points.insert(*it);
        nit++;
      }
      if (nit == names.end()) goto Done; // break out of the loop
      index++;
    }
  }

  Done:

  std::cout << "Erasing " << std::endl;
  for (std::set<sfloat*>::iterator it = unsel_points.begin();
      it != unsel_points.end(); it++) {
    // iterate to the index as indicated by the name *ptr
    for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
      // erase for all scans
      selected_points[iterator].erase(*it);
    }
  }

  std::cout << "processing done" << std::endl;
}


//------------------------------------------------------------------
/**
 * This function deals with all our keyboard activities
 */

void InterfaceFunc(unsigned char key)
{
  strncpy(path_file_name, path_filename_edit->get_text(), 1024);
  strncpy(pose_file_name, pose_filename_edit->get_text(), 1024);
  return;
}


/**
 * Function drawRobotPath
 * \brief This functions draws the path where the
 * robot has travelled along while taking the scans
 */
void drawRobotPath(int dummy)
{
  // clear the camera list as we are going to add the cameras
  // in the path where the robot travelled.

  // lets loop through the entire frame files to extract the
  // total number of places where the robot has taken the scans from
  for(unsigned int i = 0; i < MetaMatrix.size(); i++){
    double *temp;
    // Now, lets go to the last of each frame file to
    // extract the transformation matrix obtained
    // after scan matching has been done.
    glMultMatrixd(MetaMatrix[i].back());

    // temp is final transformation matrix
    temp = MetaMatrix[i].back();

    Point campos(temp[12], temp[13] + 100, temp[14]);

    // calculate lookat point
    Point lookat(0, 0, 50);
    Point up(0, 50, 0);
    double tmat[16];
    for (int i =0;i<16;i++) tmat[i] = temp[i];
    lookat.transform(tmat);
    lookat.x = lookat.x ;
    lookat.y = lookat.y + 100;
    lookat.z = lookat.z  ;

    up.transform(tmat);
    up.x = up.x ;
    up.y = up.y + 100;
    up.z = up.z  ;

    cams.push_back(campos);
    lookats.push_back(lookat);
    ups.push_back(up);
  }
  updateCamera();

  // signal for the update of scene
  haveToUpdate = 1;
}

/**
  * Calculates the positions of the interpolated camera path positions on the
  * Nurbs path. There will be an equal number of intermediate positions between
  * neighboring cameras.
  */
void calcInterpolatedCameras(std::vector<PointXY> vec1, std::vector<PointXY> vec2) {
// TODO incorporate the scaling factor!
  NurbsPath::camRatio.clear();
  double distance = 0.0;
  double dx, dy, dz;
  for(unsigned int i=0;i<vec1.size()-1;i++){
    dx = vec1.at(i+1).x - vec1.at(i).x;
    dy = vec1.at(i+1).y - vec1.at(i).y;
    dz = vec2.at(i+1).y - vec2.at(i).y;

    distance += sqrt(dx*dx + dy*dy + dz*dz );
  }
  double distance2 = 0.0;
  int im_per_cam = (distance/2.0)/(vec1.size() - 1);
  int count = 0;
  for(unsigned int i = 0; i < vec1.size()-1; i++) {
    dx = vec1.at(i+1).x - vec1.at(i).x;
    dy = vec1.at(i+1).y - vec1.at(i).y;
    dz = vec2.at(i+1).y - vec2.at(i).y;
    double curr_dist = sqrt(dx*dx + dy*dy + dz*dz);
    for(int j = 0; j < im_per_cam; j++) {
      count++;
      NurbsPath::camRatio.push_back((distance2
                                     + ((double)j
                                        * (curr_dist/(double)im_per_cam)))
                                    /(distance + 0.1));
    }

    distance2 += sqrt(dx*dx + dy*dy + dz*dz);
  }
}

/**
  * Calculates the number of interpolation points for the camera path based on
  * the length of the path
  */
int calcNoOfPoints(std::vector<PointXY> vec1, std::vector<PointXY> vec2)
{
  double distance = 0.0;
  double dx, dy, dz;
  for(unsigned int i=0;i<vec1.size()-1;i++){
    dx = vec1.at(i+1).x - vec1.at(i).x;
    dy = vec1.at(i+1).y - vec1.at(i).y;
    dz = vec2.at(i+1).y - vec2.at(i).y;

    distance += sqrt( dx*dx + dy*dy + dz*dz );
  }

  return (distance*scale) / path_interp_factor;  // change this to get faster animations
}

void mapColorToValue(int dummy)
{
  switch (listboxColorVal) {
    case 0:
      cm->setCurrentType(PointType::USE_HEIGHT);
      break;
    case 1:
      cm->setCurrentType(PointType::USE_REFLECTANCE);
      break;
    case 2:
      cm->setCurrentType(PointType::USE_TEMPERATURE);
      break;
    case 3:
      cm->setCurrentType(PointType::USE_AMPLITUDE);
      break;
    case 4:
      cm->setCurrentType(PointType::USE_DEVIATION);
      break;
    case 5:
      cm->setCurrentType(PointType::USE_TYPE);
      break;
    case 6:
      cm->setCurrentType(PointType::USE_COLOR);
      break;
    default:
      break;
  };
  resetMinMax(0);
}

void changeColorMap(int dummy)
{
  ColorMap c;
  GreyMap gm;
  HSVMap hsv;
  SHSVMap shsv;
  JetMap jm;
  HotMap hot;
  DiffMap diff;
  TempMap temp;

  switch (listboxColorMapVal) {
    case 0:
      // TODO implement no color map
      cm->setColorMap(c);
      break;
    case 1:
      cm->setColorMap(gm);
      break;
    case 2:
      cm->setColorMap(hsv);
      break;
    case 3:
      cm->setColorMap(jm);
      break;
    case 4:
      cm->setColorMap(hot);
      break;
    case 5:
      cm->setColorMap(diff);
      break;
    case 6:
      cm->setColorMap(shsv);
      break;
    case 7:
      cm->setColorMap(temp);
      break;
    default:
      break;
  }
}

void minmaxChanged(int dummy) {
  cm->setMinMax(mincolor_value, maxcolor_value);
}

void resetMinMax(int dummy) {
  mincolor_value = cm->getMin();
  maxcolor_value = cm->getMax();
  minmaxChanged(0);
}

void setScansColored(int dummy) {
  switch(colorScanVal) {
    case 0:
      cm->setMode(ScanColorManager::MODE_STATIC);
      break;
    case 1:
      cm->setMode(ScanColorManager::MODE_COLOR_SCAN);
      break;
    case 2:
      cm->setMode(ScanColorManager::MODE_POINT_COLOR);
      break;
    default:
      break;
  }
}


void changePointMode(int dummy) {
  if (dummy == 0) {           // always display
    if (pointmode != 1) {  // standard mode
      pointmode = 1;
      // never_box->set_int_val(0);
    } else {
      pointmode = 0;
    }
  } else if (dummy == 1) {    // never display
    if (pointmode != -1) {  // standard mode
      pointmode = -1;
      //always_box->set_int_val(0);
    } else {
      pointmode = 0;
    }
  }
    updatePointModeControls();
}


void callCameraUpdate(int dummy)
{
  updateCamera();
}

void calcPointSequence(std::vector<int> &sequence, int frameNr)
{
  sequence.clear();
  std::vector<std::pair<double, int> > dists;
  double x,y,z;

  for (unsigned int i = 0; i < octpts.size(); i++) {
    // stop at scans that don't have any frames associated with them
    if(i >= MetaMatrix.size()) break;
    // set usable frame
    double* frame;
    if((unsigned int)frameNr >= MetaMatrix[i].size()) {
      // use last possible frame
      frame = MetaMatrix[i].back();
    } else {
      frame = MetaMatrix[i][frameNr];
    }
    x = frame[12];
    y = frame[13];
    z = frame[14];
    dists.push_back( std::pair<double, int>(sqr(X + x) + sqr(Y + y) + sqr(Z + z),
                                       i) );
  }

  sort( dists.begin(), dists.end());

  for (unsigned int i = 0; i < dists.size(); i++) {
    sequence.push_back( dists[i].second);
  }
}
