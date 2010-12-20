#include <string.h>

#include "show/viewcull.h"
#include "show/scancolormanager.h"

bool fullydisplayed = true;   // true if all points have been drawn to the screen
bool showall = false;         // true iff next DrawPoints should redraw scene with all points
bool mousemoving = false;     // true iff a mouse button has been pressed inside a window, but hs not been released
bool delayeddisplay = false;  // true iff mouse button callbacks should redraw the scene after button release
double ptstodisplay = 100000;  
double lastfps = idealfps;    // last frame rate    
int pointmode = -1;


/**
 * Displays all data (i.e., points) that are to be displayed
 * @param mode spezification for drawing to screen or in selection mode
 */
void DrawPoints(GLenum mode)
{
  long time = GetCurrentTimeInMilliSec();
  double  max = 10000000000;
  double min = 10000;
  ptstodisplay *= 1.0 + (lastfps - idealfps)/idealfps;
  if (ptstodisplay < min) ptstodisplay = min;
  else if (ptstodisplay > max) ptstodisplay = max;

  // In case of animation
  if(scanNr != -1) {
    cm->setMode(ScanColorManager<double>::MODE_ANIMATION);

#ifdef USE_GL_POINTS
    for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
#else
    for(int iterator = (int)Scan::allScans.size()-1; iterator >= 0; iterator--) {
#endif

      if (MetaAlgoType[iterator][frameNr] == Scan::INVALID) continue;
      cm->selectColors(MetaAlgoType[iterator][frameNr]);	 
      glPushMatrix();
      glMultMatrixd(MetaMatrix[iterator][frameNr]);

      glPointSize(pointsize);
#ifdef USE_GL_POINTS
        ExtractFrustum(pointsize);
        cm->selectColors(MetaAlgoType[iterator][frameNr]);
        if (pointmode == 1 || (showall && pointmode == 0) ) {
          octpts[iterator]->displayOctTreeAllCulled();
        } else {
          octpts[iterator]->displayOctTreeCulled(ptstodisplay/(int)octpts.size());
        }
#else
      for (unsigned int jterator = 0; jterator < vvertexArrayList[iterator].size(); jterator++) {

        if ((jterator == 0) && vvertexArrayList[iterator][jterator]->numPointsToRender > 0) {
          cm->selectColors(MetaAlgoType[iterator][frameNr]);
        }

        if (vvertexArrayList[iterator][jterator]->numPointsToRender > 0) {
          glCallList(vvertexArrayList[iterator][jterator]->name);
        }
      }
#endif
      glPopMatrix();
    }

    setScansColored(0);
  } else {

    if(mode == GL_SELECT){
      // select points mode
      // ------------------
      for(unsigned int iterator = 0; iterator < Scan::allScans.size(); iterator++) {
        glPushMatrix();
        glMultMatrixd(MetaMatrix[iterator].back());
        for (unsigned int jterator = 0;
            jterator < Scan::allScans[iterator]->get_points()->size();
            jterator++) {
          GLuint name = iterator * 1000000 + jterator  ;
          glLoadName(name);
          glBegin(GL_POINTS);
          setGLPoint(Scan::allScans[iterator]->get_points()->at(jterator).x,
              Scan::allScans[iterator]->get_points()->at(jterator).y,
              Scan::allScans[iterator]->get_points()->at(jterator).z);
          glEnd();
        }
        glFlush();
        glPopMatrix();
      }
    } else {

      // draw point is normal mode
      // -------------------------

      glPointSize(pointsize);

#ifdef USE_GL_POINTS
      for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
#else
      for(int iterator = (int)Scan::allScans.size()-1; iterator >= 0; iterator--) {
#endif
        glPushMatrix();
        if (invert)                               // default: white points on black background
          glColor4d(1.0, 1.0, 1.0, 1.0);
        	   //if (iterator == 0) glColor4d(139.0/255, 69.0/255, 19.0/255, 1.0);
        	   //if (iterator == 0) glColor4d(0.5, 1.0, 0.5, 1.0);
        else                                      // black points on white background
          glColor4d(0.0, 0.0, 0.0, 1.0);

        	   //if (iterator == 0) glColor4d(0.5, 1.0, 0.5, 1.0);
        	   if (iterator == 0) glColor4d(139.0/255, 69.0/255, 19.0/255, 1.0);

        glMultMatrixd(MetaMatrix[iterator].back());

#ifdef USE_GL_POINTS
       //  cout << endl << endl;  calcRay(0,0, 1.0, 40000.0);
        
        ExtractFrustum(pointsize);
        if (pointmode == 1 || (showall && pointmode == 0) ) {
          octpts[iterator]->displayOctTreeAllCulled();
        } else {
          octpts[iterator]->displayOctTreeCulled(ptstodisplay/(int)octpts.size());
        }
        glColor3f(1.0, 0.0, 0.0);
        glPointSize(pointsize + 10.0);
          glBegin(GL_POINTS);
          for (unsigned int i = 0; i < selected_points[iterator].size(); i++) {
            glVertex3d(selected_points[iterator][i][0], selected_points[iterator][i][1], selected_points[iterator][i][2]);
          }
          glEnd();
        glPointSize(pointsize);
        
#else
        for (unsigned int jterator = 0; jterator < vvertexArrayList[iterator].size(); jterator++) {
          if (vvertexArrayList[iterator][jterator]->numPointsToRender > 0) {
            glCallList(vvertexArrayList[iterator][jterator]->name);
          }
        }
#endif
        glPopMatrix();
      }
    }
  }

  if (pointmode == 1 || (showall && pointmode == 0) ) {
    fullydisplayed = true;
  } else {
    lastfps =  1000.0/(GetCurrentTimeInMilliSec() - time);
    fullydisplayed = false;
  }
  showall = false;         
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
    glColor3f(0.0, 1.0, 0.0);
    // set the points
    glVertex3f(path_vectorX.at(j).x,path_vectorX.at(j).y,path_vectorZ.at(j).y);
  }
  glEnd();
  
  // draw lookat path
  glBegin(GL_LINE_STRIP);
  for(unsigned int j = 0; j < lookat_vectorX.size(); j++){
    //set the color 
    glColor3d(1.0, 1.0, 0.0);
    //set the points
    glVertex3f(lookat_vectorX.at(j).x,lookat_vectorX.at(j).y,lookat_vectorZ.at(j).y);
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
      glColor3f(1, 0, 1);
    } else {
      glColor3f(0, 1, 0);
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
    
    glColor3f(1, 1, 0);
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


   /*
    if (i+1 == cam_choice) {
      glColor3f(1, 1, 0);
      glPointSize(20);
    } else {
      glColor3f(0, 0, 1);
      glPointSize(10);
    }
    glBegin(GL_POINTS);
    glVertex3d(p.x, p.y, p.z);
    glEnd();

    if (i+1 == cam_choice) {
      glColor3f(0, 1, 1);
      glPointSize(20);
    } else {
      glColor3f(1, 0, 0);
      glPointSize(10);
    }
    glBegin(GL_POINTS);
    glVertex3d( l.x,  l.y,  l.z);
    glEnd();

    if (i+1 == cam_choice) {
      glColor3f(1, 0, 1);
      glPointSize(20);
    } else {
      glColor3f(0, 1, 0);
      glPointSize(10);
    }
    glBegin(GL_POINTS);
    glVertex3d( u.x,  u.y,  u.z);
    glEnd();
*/ 

    glPopMatrix();
  }
}

//-----------------------------------------------------------------------------------

  
/**
 * Dispaly function
 */
void DisplayItFunc(GLenum mode)
{
  /**
   * Color of the fog 
   */
  GLfloat fogColor[4];
  
  // set the clear color buffer in case of
  // both invert and non invert mode
  if (invert)
    glClearColor(0.0, 0.0, 0.0, 0.0);
  else
    glClearColor(1.0, 1.0, 1.0, 1.0);

  // clear the color and depth buffer bit
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();

  // set the matrix mode
  glMatrixMode(GL_MODELVIEW);

  // init modelview matrix
  glLoadIdentity();

  // set the polygon mode
  glPolygonMode(GL_FRONT/*_AND_BACK*/, GL_LINE); 

  // variable to store the axis values
  double axis[3];

  // convert from quaternion to axis angle
  QuaternionToAxisAngle(quat, axis, angle);

  // do the model-transformation
  if (haveToUpdate == 6 && path_iterator < path_vectorX.size() ) {
    gluLookAt(path_vectorX.at(path_iterator).x, path_vectorX.at(path_iterator).y, path_vectorZ.at(path_iterator).y,
              lookat_vectorX.at(path_iterator).x, lookat_vectorX.at(path_iterator).y, lookat_vectorZ.at(path_iterator).y,
              ups_vectorX.at(path_iterator).x - path_vectorX.at(path_iterator).x,
              ups_vectorX.at(path_iterator).y - path_vectorX.at(path_iterator).y, 
              ups_vectorZ.at(path_iterator).y - path_vectorZ.at(path_iterator).y);
  }else {
    if (cameraNavMouseMode == 1) {
      glRotated( mouseRotX, 1, 0, 0);
      glRotated( mouseRotY, 0, 1, 0);
    } else glRotated(angle, axis[0], axis[1], axis[2]);    // rotate the camera
    glGetFloatv(GL_MODELVIEW_MATRIX, view_rotate_button);
    
    glui2->sync_live();
    glui2->show();

    glTranslated(X, Y, Z);       // move camera	
  }


//   cout << "Position  :" << X << " " << Y << " " << Z << endl;
//   cout << "Quaternion:" << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << endl;
//   cout << "Axis/Angle:" << axis[0] << " " << axis[1] << " " << axis[2] << " " << angle << endl;
//   cout << "Apex angle:" << cangle << endl;
//   cout << endl;
  
  // process fog
  if (show_fog > 0) {
    if (show_fog > 3) 
      fogColor[0] = fogColor[1] = fogColor[2] = fogColor[3] = 1.0;
    else 
      fogColor[0] = fogColor[1] = fogColor[2] = fogColor[3] = 0.0;
    glEnable(GL_FOG);
    {
	 if (show_fog==1) fogMode = GL_EXP;
	 else if (show_fog==2) fogMode = GL_EXP2;
	 else if (show_fog==3) fogMode = GL_LINEAR;
	 else if (show_fog==4) fogMode = GL_EXP;
	 else if (show_fog==5) fogMode = GL_EXP2;
	 else if (show_fog==6) fogMode = GL_LINEAR;
      glFogi(GL_FOG_MODE, fogMode);
      glFogfv(GL_FOG_COLOR, fogColor);
      glFogf(GL_FOG_DENSITY, fogDensity);
      glHint(GL_FOG_HINT, GL_NICEST);
      glFogf(GL_FOG_START, 100.0);
      glFogf(GL_FOG_END, 32000.0);
    }
  } else {
    glDisable(GL_FOG);
  }
 
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // 
  // show the objects __after__ the model-transformation 
  // for all status variables we show the appropiated thing
  // using the drawing functions
  //
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // if show points is true the draw points
  if (show_points == 1) DrawPoints(mode);

  if(show_path == 1) {
    double *pose;
    glColor4d(1.0, 0.0, 0.0, 1.0);
    glLineWidth(5);
    glBegin(GL_LINE_STRIP);
    for(unsigned int i = 0; i < MetaMatrix.size(); i++){
      if(frameNr > -1 && frameNr < (int)MetaMatrix[i].size()) {
	      if(MetaAlgoType[i][frameNr] == Scan::INVALID) {
          continue;
        }
        pose = MetaMatrix[i][frameNr];
      } else {
        pose = MetaMatrix[i].back();
      }
      if(showTopView) {
        glVertex3f(pose[12], 2000, pose[14]);
      } else {
        glVertex3f(pose[12], pose[13], pose[14]);
      }
    }
    glEnd();
  }

  // if show camera is true then draw cameras.
  if (show_cameras == 1) {
    DrawCameras();
  }
  
  // if show path is true the draw path.
  if(show_path == 1) {
    DrawPath();
  }
  
  glPopMatrix();
  // force draw the scene
  glFlush();
  glFinish();
}

/**
 * Function topview. Set the screen for top view.
 */
void topView()
{
  static GLdouble save_qx, save_qy, save_qz, save_qangle, save_X, save_Y, save_Z;
  static GLdouble saveMouseRotX, saveMouseRotY;
  
  if (!showTopView) // set to top view
  {
	  showTopView = true;
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
	 
	  Y = Y - 350.0;
	  Z = Z + 500.0;
	  quat[3] = quat[0] = sqrt(0.5);
	  quat[1] = quat[2] = 0.0;
	  mouseRotX = 90;
	  mouseRotY = 0;
	 
	  haveToUpdate = 2;
  } else {
	  showTopView = false;

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
	 
	  haveToUpdate = 2;	 
  }
}

//---------------------------------------------------------------------------
/**
 * This function is called when the user wants to
 * delete a camera.
 */

void callDeleteCamera(int dummy){
 
  //iterator for the position of camera
  //in the camera list
  vector<Point>::iterator position;
  vector<Point>::iterator positionL;
  vector<Point>::iterator positionU;

  //calculate the position of the camera. we are referring
  //to the selected camera
  position = cams.begin()+ (cam_choice-1);
  positionL = lookats.begin()+ (cam_choice-1);
  positionU = ups.begin()+ (cam_choice-1);

  //if no camera present then return
  if(cam_choice == 0)
    return;

  //if the list is not empty then
  if(!cams.empty()){
    //delete the camera from the position
    cams.erase(position);
    lookats.erase(positionL);
    ups.erase(positionU);
    //reset the cam_choice spinner values
  }
  
  updateCamera();
}


//---------------------------------------------------------------------------
/**
 * Function to reset the viewer window.
 */

void resetView(int dummy)
{
  Y = START_Y;
  quat[3] = 1.0;
  cangle = 60.0;
  cangle_spinner->set_float_val(cangle);  
  pzoom = 2000.0;
  pzoom_spinner->set_float_val(pzoom);  
  quat[0] = quat[1] = quat[2] = X = Z = 0.0;
  rotButton->reset();
  haveToUpdate = 2;
  mouseRotX = 0;
  mouseRotY = 0;
}

/**
 * Function to set the viewer window back to a previously saved state.
 */

void setView(double pos[3], double new_quat[4], 
             double newMouseRotX, double newMouseRotY, double newCangle,
             bool sTV, bool cNMM, double pzoom_new, 
             bool s_points, bool s_path, bool s_cameras, double ps, int
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
  showTopView = sTV,
  cameraNavMouseMode = cNMM;
  pzoom = pzoom_new;
  pzoom_spinner->set_float_val(pzoom);  
  if(showTopView) {
    pzoom_spinner->enable();
    cangle_spinner->disable();
  } else {
    pzoom_spinner->disable();
    cangle_spinner->enable();
  }
  show_points = s_points;
  show_path = s_path;
  show_cameras = s_cameras;
  pointsize = ps;
  show_fog = sf;
  fogDensity = fD;
  invert = inv;
  
  haveToUpdate = 2;
}
  

//---------------------------------------------------------------------------
/**
 * This function is called when the viewer is created. This
 * acts as the display function.
 */

void CallBackDisplayFunc()
{
  if ((cangle_spinner != 0 && (fabs(cangle_spinner->get_float_val() - cangle) > 0.5)) || 
	 (pzoom_spinner != 0 && (fabs(pzoom_spinner->get_float_val() - pzoom) > 0.5))) {

    int viewport[4];
    cangle = cangle_spinner->get_float_val();
    pzoom  = pzoom_spinner->get_float_val();
    glGetIntegerv(GL_VIEWPORT, viewport);
    CallBackReshapeFunc(viewport[2],viewport[3]);
#ifdef _MSC_VER
	Sleep(25);
#else
    usleep(250000);
#endif
  }

  glDrawBuffer(buffermode);
  // delete framebuffer and z-buffer

  //Call the display function
  DisplayItFunc(GL_RENDER);
  
  // show the rednered scene
  glutSwapBuffers(); 

  // live sync the glui variables with the controls
  glui1->sync_live();
  glui2->sync_live();

}
//--------------------------------------------------------------------------------


/**
 * This function is called when there is nothing to be done
 * in the screen.
 */
void CallBackIdleFunc(void)
{

#ifdef _MSC_VER
  Sleep(1);
#else
  usleep(1000);
#endif

  if(glutGetWindow() != window_id)
    glutSetWindow(window_id);
	 
  // return is nothing has to be updated
  if (haveToUpdate == 0) return;

  // case: display is invalid - update it
  if (haveToUpdate == 1) {
    glutPostRedisplay();
    haveToUpdate = 0;
    return;
  }
  // case: display is invalid - update it with all points
  if (haveToUpdate == 7) {
    showall = true;
    glutPostRedisplay();
    haveToUpdate = 0;
    return;
  }

  // case: camera angle is changed - instead of repeating code call Reshape,
  // since these OpenGL commands are the same
  if (haveToUpdate == 2) {
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    CallBackReshapeFunc(viewport[2],viewport[3]);
    glutPostRedisplay();
    haveToUpdate = 0;
    return;
  }
  
  // case: animation
  static int anim_iterator = 0; // # scan
  static int anim_jterator = 0; // # frame in this scan
  /* 
   */
  if(haveToUpdate == 3 ){
    anim_jterator += 1;
    if(!(anim_jterator < (int) MetaMatrix[1].size())){
      anim_iterator = 0;
      haveToUpdate = 4;
      return;
    }
    scanNr = 1;
    frameNr =  calcFrameNo() + anim_jterator;
    glutPostRedisplay();

    if(save_animation){
	 string filename = scandir + "animframe" + to_string(frameNr,4) + ".ppm";
	 cout << filename << endl;
	 glDumpWindowPPM(filename.c_str(),0);
	
    }
    
  }
#ifdef _MSC_VER
  Sleep(300);
  Sleep(anim_delay);
#else
  usleep(anim_delay * 10000);
#endif

  if (haveToUpdate == 4) { // stop animation
    anim_iterator = 0;  // so that the next click on the button restarts the animation.
    anim_jterator = 0;  // delete these lines if you want a 'continue' functionality.
    
    scanNr = frameNr = -1;
    haveToUpdate = 1;
  }

  // case: path animation
  if(haveToUpdate == 6){

    if (path_iterator == 0) {
      oldcamNavMode = cameraNavMouseMode;  // remember state of old mousenav
      cameraNavMouseMode = 0;
    }



    //check if the user wants to animate both
    //scan matching and the path at the same
    //time

    //cout << "path_iterator: " << path_iterator << endl;
    if(path_iterator < path_vectorX.size()){   // standard animation case

      //call the path animation function
      //hide both the cameras and the path
      show_cameras = 0;
      show_path = 0;
      //increase the iteration count

      path_iterator += 1;
      //cout << "i am here" << endl;
      //repaint the screen
      glutPostRedisplay();

      //save the animation
      if(save_animation){
        string filename = scandir + "animframe" + to_string(path_iterator,4) + ".ppm";
        cout << "written " << filename << " of " << path_vectorX.size() << " files" << endl;
        glWriteImagePPM(filename.c_str(), factor, 0);
        haveToUpdate = 6;

      }
    }else{                             // animation has just ended
      cameraNavMouseMode = oldcamNavMode;
      show_cameras = 1;
      show_path = 1;
      //cout << "i am here instead" << endl;
      haveToUpdate = 0;
    }
  }
  
}



//--------------------------------------------------------------------------------------------
/**
 * This function handles the rotation of the view
 */

void update_view_rotate(int t)
{
  double view_rotate_button_quat[4];

  // convert the rotate button matrix to quaternion 
  Matrix4ToQuaternion(view_rotate_button, view_rotate_button_quat);

  // normalize the quartenion
  QuatNormalize(view_rotate_button_quat);
    
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

  X = X + obj_pos_button1[0] * view_rotate_button[0] + obj_pos_button1[1] * view_rotate_button[1] + obj_pos_button1[2] * view_rotate_button[2]; 
  Y = Y + obj_pos_button1[0] * view_rotate_button[4] + obj_pos_button1[1] * view_rotate_button[5] + obj_pos_button1[2] * view_rotate_button[6]; 
  Z = Z + obj_pos_button1[0] * view_rotate_button[8] + obj_pos_button1[1] * view_rotate_button[9] + obj_pos_button1[2] * view_rotate_button[10]; 

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
    }
    else // stop animation
      haveToUpdate = 4;
  }
}

/**
 * calls the resetView function 
 * @param dummy not needed necessary for glui 
 */
void callResetView(int dummy)
{
  if (showTopView) callTopView(dummy);
  resetView(0);
}

/**
 * calls the resetView function 
 * @param dummy not needed necessary for glui 
 */
void invertView(int dummy)
{
  invert = !invert;
  cm->setInvert(!invert);
}

/**
 * calls the topView function 
 * @param dummy not needed necessary for glui 
 */
void callTopView(int dummy)
{ 
  topView();
  if (showTopView) {
    rotButton->disable();
    cangle_spinner->disable();
    pzoom_spinner->enable();
  } else {
    rotButton->enable();
    cangle_spinner->enable();
    pzoom_spinner->disable();
  }
}

//---------------------------------------------------------------------------------

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
  for (int i =0;i<16;i++) tmat[i] = view_rotate_button[i];
  
  lookat.x = -50*tmat[2] -X;
  lookat.y = -50*tmat[6] -Y;
  lookat.z = -50*tmat[10] -Z;

  up.x = 50*tmat[1] -X; 
  up.y = 50*tmat[5] -Y; 
  up.z = 50*tmat[9] -Z; 

  cams.push_back(campos);
  lookats.push_back(lookat);
  ups.push_back(up);
  
  updateCamera();
  
  //signal to repaint screen
  haveToUpdate  = 1;
}

//--------------------------------------------------------------------------------------
void CallBackMouseFuncMoving(int button, int state, int x, int y)
{

  if( state == GLUT_DOWN) {
    mousemoving = true;
  } else {
    mousemoving = false;
    if (delayeddisplay) {
      delayeddisplay = false;
      if (fullydisplayed) return;
      if (haveToUpdate == 6 || haveToUpdate == 3 || haveToUpdate == 4) return;
      haveToUpdate = 7;
    }
  }
}


void CallBackEntryFunc(int state) {
  if (state == GLUT_LEFT) {
      if (mousemoving) {  // mouse button was pressed, delay redisplay
        delayeddisplay = true;
      } 
      else {
        if (fullydisplayed ) return;
        if (haveToUpdate == 6 || haveToUpdate == 3 || haveToUpdate == 4) return;
        haveToUpdate = 7;
      }
  } else if (state == GLUT_ENTERED) {
      showall = false;
  }
  
}

/**
 * This function is called after a mousebutton has been pressed.
 */
void CallBackMouseFunc(int button, int state, int x, int y)
{

  //GLuint selectBuf[BUFSIZE];
  //GLint hits;
  //GLint viewport[4];

  if(cameraNavMouseMode != 1) {
    if (state == GLUT_DOWN && (button == GLUT_LEFT_BUTTON || button == GLUT_RIGHT_BUTTON)) {
      ///////////////////////////////////////

#ifdef USE_GL_POINTS
      // set the matrix mode
      glMatrixMode(GL_MODELVIEW);
      // init modelview matrix
      glLoadIdentity();
      // variable to store the axis values
      double axis[3];
      // convert from quaternion to axis angle
      QuaternionToAxisAngle(quat, axis, angle);

      // do the model-transformation
      if (cameraNavMouseMode == 1) {
        glRotated( mouseRotX, 1, 0, 0);
        glRotated( mouseRotY, 0, 1, 0);
      } else glRotated(angle, axis[0], axis[1], axis[2]);    // rotate the camera
      
      glGetFloatv(GL_MODELVIEW_MATRIX, view_rotate_button);
      glui2->sync_live();
      glui2->show();
      glTranslated(X, Y, Z);       // move camera	

      for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
        selected_points[iterator].clear();
      }
      sfloat *sp = 0;
      for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) {
        glPushMatrix();
        glMultMatrixd(MetaMatrix[iterator].back());
        calcRay(x, y, 1.0, 40000.0);
//        octpts[iterator]->selectRay(selected_points);
        octpts[iterator]->selectRay(sp);
        if (sp != 0) {
          selected_points[iterator].push_back(sp);
        }
        glPopMatrix();
      }
      glPopMatrix();
      glutPostRedisplay();
      /////////////////////////////////////
#else
      if (!showTopView) {

        // set up a special picking matrix, drwa to an virt. screen
        glGetIntegerv(GL_VIEWPORT, viewport);

        glSelectBuffer(BUFSIZE, selectBuf);
        (void) glRenderMode(GL_SELECT);

        glInitNames();
        glPushName(0);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();

        gluPickMatrix((GLdouble)x, (GLdouble)(viewport[3]-y), 2.0, 2.0, viewport);
        gluPerspective(cangle, aspect, 1.0, 40000.0);
        glMatrixMode(GL_MODELVIEW);
        showall = true;
        DisplayItFunc(GL_SELECT);

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

        hits = glRenderMode(GL_RENDER);                       // get hits
        ProcessHitsFunc(hits, selectBuf,button);
        //	 glutPostRedisplay();

      } else {

        glMatrixMode ( GL_PROJECTION );
        glPushMatrix();
        glLoadIdentity ();
        gluPickMatrix((GLdouble)x, (GLdouble)(viewport[3]-y), 3.0, 3.0, viewport);
        glOrtho ( -aspect * pzoom, aspect * pzoom,
            -1 * pzoom, pzoom,
            1.0, 32000.0 );
        glMatrixMode ( GL_MODELVIEW );
        DisplayItFunc(GL_SELECT);
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);

        hits = glRenderMode(GL_RENDER);                       // get hits
        ProcessHitsFunc(hits, selectBuf,button);

        //	 glutPostRedisplay(); 

      }
#endif
    }
  } else {
   
    if( state == GLUT_DOWN) {

      mouseNavX = x;
      mouseNavY = y;
      mouseNavButton = button;
      
      mousemoving = true;
    } else {
      mousemoving = false;
      if (delayeddisplay) {
        delayeddisplay = false;
        if (fullydisplayed) return;
        if (haveToUpdate == 6 || haveToUpdate == 3 || haveToUpdate == 4) return;
        haveToUpdate = 7;
      }
    }
  }
}

void CallBackMouseMotionFunc(int x, int y) {
  if(cameraNavMouseMode == 1) {
    double mat[9];
    double mouseRotXRand = M_PI * mouseRotX / 180; 
    double mouseRotYRand = M_PI * (360-mouseRotY) / 180;
    mat[0] = cos(mouseRotYRand);
    mat[1] = 0;
    mat[2] = sin(mouseRotYRand);
    mat[3] = sin(mouseRotXRand) * sin(mouseRotYRand);
    mat[4] = cos(mouseRotXRand);
    mat[5] = -cos(mouseRotYRand) * sin(mouseRotXRand);
    mat[6] = -cos(mouseRotXRand) * sin(mouseRotYRand);
    mat[7] = sin(mouseRotXRand);
    mat[8] = cos(mouseRotXRand) * cos(mouseRotYRand);

    int deltaMouseX = x - mouseNavX;
    int deltaMouseY = mouseNavY - y;
    int deltaMouseZ = y - mouseNavY;
    mouseNavX = x;
    mouseNavY = y;
      
    double transX, transY, transZ;
    transX = transY = transZ = 0.0;
    if( mouseNavButton == GLUT_RIGHT_BUTTON){
      if ( showTopView ) {
        deltaMouseX *= 5;
        deltaMouseY *= 5;
      }
      transX = deltaMouseX * mat[0] + -deltaMouseY * mat[3];
      transY = deltaMouseX * mat[1] + deltaMouseY * mat[4];
      transZ = -deltaMouseX * mat[2] + deltaMouseY * mat[5]; 
    } else if( mouseNavButton == GLUT_MIDDLE_BUTTON ){
      if ( !showTopView ) {
        deltaMouseY *= -5;
        deltaMouseZ *= -5;
      }
      transX = deltaMouseX * mat[0] + deltaMouseZ * mat[6];
      transY = deltaMouseX * mat[1] + deltaMouseY * mat[7];      
      transZ = -deltaMouseX * mat[2] + -deltaMouseZ * mat[8];
    } else if ( mouseNavButton == GLUT_LEFT_BUTTON ){
      if ( !showTopView ){
        mouseRotX += deltaMouseY;
        mouseRotY -= deltaMouseX;
        if (mouseRotX < -90) mouseRotX=90;
        else if (mouseRotX > 90) mouseRotX=90;
        if (mouseRotY > 360) mouseRotY=0;
        else if (mouseRotY < 0) mouseRotY=360;
      }
    }
    X += transX;
    Y += transY;
    Z += transZ;
    haveToUpdate = 1;
  }
}



//--------------------------------------------------------------------------------

void initScreenWindow()
{
  // init display
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);

  // define the window position and size
  glutInitWindowPosition(START_X, START_Y);
  glutInitWindowSize( START_WIDTH, START_HEIGHT );

  // create window and append callback functions
  window_id = glutCreateWindow("3D_Viewer");

  glutDisplayFunc( CallBackDisplayFunc );
  glutReshapeFunc( CallBackReshapeFunc );
  
  glutMouseFunc  ( CallBackMouseFunc );
  glutKeyboardFunc ( CallBackKeyboardFunc);
  glutMotionFunc ( CallBackMouseMotionFunc); 
  glutSpecialFunc ( CallBackSpecialFunc);
  glutEntryFunc ( CallBackEntryFunc);
  GLUI_Master.set_glutReshapeFunc( CallBackReshapeFunc );
  GLUI_Master.set_glutIdleFunc( CallBackIdleFunc );

  update_view_rotate(0);

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
  glReadBuffer(buffermode);
  glReadPixels(0, 0, win_width, win_height,
                        GL_RGBA, GL_UNSIGNED_BYTE, buffer);

  // Open the output file
  fp.open(filename, ios::out);

  // Write a proper P6 PPM header
  fp << "P6" << endl << "# CREATOR: 3D_Viewer by Andreas Nuechter, University of Osnabrueck"
	<< endl << win_width  << " " << win_height << " " << UCHAR_MAX << endl;

  // Loop through the frame buffer data, writing to the PPM file.  Be careful
  //   to account for the frame buffer having 4 bytes per pixel while the
  //   output file has 3 bytes per pixel
  l = 0;
  for (i = 0; i < win_height; i++) {     // For each row
    for (j = 0; j < win_width; j++) {    // For each column
      for (k = 0; k < RGB; k++) {        // For each RGB component
        //cout << (RGBA*((win_height-1-i)*win_width+j)+k) << endl;
        ibuffer[l++] = (unsigned char)
          *(buffer + (RGBA*((win_height-1-i)*win_width+j)+k));
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

/* +++++++++-------------++++++++++++
 * NAME
 *   glDumpWindowPPM
 * DESCRIPTION
 *   writes an ppm file of the window
 *   content
 *   size is scale times the window size
 * PARAMETERS
 *   filename
 * RESULT
 *  writes the framebuffer content
 *  to a ppm file
+++++++++-------------++++++++++++ */
void glWriteImagePPM(const char *filename, int scale, GLenum mode)
{
  //if(!showTopView) {
    int m,o,k;                  // Counter variables
    // Get viewport parameters
    double left, right, top, bottom;
    double tmp = 1.0/tan(rad(cangle)/2.0);
   
    // Save camera parameters
    GLdouble savedMatrix[16];
    glGetDoublev(GL_PROJECTION_MATRIX,savedMatrix);
     
    top = 1.0/tmp;
    bottom = -top;
    right = (aspect)/tmp;
    left = -right;

    double part_height, part_width;
    if(!showTopView) {
      part_width = (right - left)/(double)scale;
      part_height = (top - bottom)/(double)scale;
    } else {
      part_height = (2*pzoom)/scale;
      part_width = (2*pzoom*aspect)/scale;
      cout << part_width << " " << part_height << endl;
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
    unsigned char *ibuffer;       // The PPM Output Buffer
    buffer = new GLubyte[win_width * win_height * RGBA];
    ibuffer = new unsigned char[image_width * image_height * RGB];

    double height;
    if(!showTopView) {
      height = bottom;
    } else {
      height = -pzoom;
    }
    for(int i = 0; i < scale; i++) {
      double width;
      if(!showTopView) {
        width = left;
      } else {
        width = -pzoom*aspect;
      }
      for(int j = 0; j < scale; j++) {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        if(!showTopView) {
          glFrustum(width, width + part_width, height, height + part_height, 1.0, 40000.0);
          showall = true;
          DisplayItFunc(mode); 
        } else {
          cout << width << " " << width + part_width << " " << height << " "
          << height + part_height << endl;
          glOrtho( width, width + part_width, 
                   height, height + part_height, 
                   1.0, 32000.0 );
          glMatrixMode(GL_MODELVIEW);
          DisplayItFunc(mode);
        }
    
        // Read window contents from GL frame buffer with glReadPixels
        glFinish();
        glReadBuffer(buffermode);
        glReadPixels(0, 0, win_width, win_height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
       
        // Loop through the frame buffer data, writing to the PPM file.  Be careful
        //   to account for the frame buffer having 4 bytes per pixel while the
        //   output file has 3 bytes per pixel
                                          // end row
        for (m = 0; m < win_height; m++) {     // For each row
          for (o = 0; o < win_width; o++) {    // For each column
            for (k = 0; k < RGB; k++) {        // For each RGB component
              int l = (k+RGB*(image_width*((scale - 1 - i)*win_height + m) + j*win_width + o));
              ibuffer[l]   = (unsigned char) *(buffer + (RGBA*((win_height-1-m)*win_width+o)+k));
            }                                  // end RGB
          }                                    // end column
        }
        width += part_width;
      }
      height += part_height;
    }
   
    // show the starting scene 
    glLoadMatrixd(savedMatrix);
    // show the rednered scene
    haveToUpdate=2;

    ofstream fp;                  // The PPM File

    // Open the output file
    fp.open(filename, ios::out);

    // Write a proper P6 PPM header
    fp << "P6" << endl << "# CREATOR: 3D_Viewer by Dorit Borrmann, Jacobs University Bremen gGmbH"
    << endl << image_width  << " " << image_height << " " << UCHAR_MAX << endl;

    // Write output buffer to the file 
    fp.write((const char*)ibuffer, sizeof(unsigned char) * (RGB * image_width * image_height));
    fp.close();
    fp.clear();
    delete [] buffer;
    delete [] ibuffer;
}

/** Reshape Function
 * TODO: have to describe it.
 *
 */
void CallBackReshapeFunc(int width, int height)
{
    aspect = (double)width/(double)height;
  if (!showTopView) {
    // usage of the vsize of a structiewport
    glViewport(0, 0, (GLint)width, (GLint)height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // angle, aspect, near clip, far clip
    // get matrix
    gluPerspective(cangle, aspect, 1.0, 40000.0);

    // now use modelview-matrix as current matrix
    glMatrixMode(GL_MODELVIEW);

    haveToUpdate = 1;
    
  } else {

    // usage of the viewport
    glViewport ( 0, 0, width, height);
    
    glMatrixMode ( GL_PROJECTION );
    glLoadIdentity ();

    // get matrix
	 glOrtho ( -aspect * pzoom, aspect * pzoom,
			 -1 * pzoom, pzoom,
			 1.0, 32000.0 );

    // now use modelview-matrix as current matrix
    glMatrixMode(GL_MODELVIEW);

    haveToUpdate = 1;
 
  }
  glEnable(GL_BLEND);
//  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);  
  glEnable (GL_POINT_SMOOTH);
}

/**
 *  Prints out which points were clicked on
 */
void ProcessHitsFunc(GLint hits, GLuint buffer[],int button)
{
  GLuint *ptr, names;
  GLuint scannr, pointnr;

  unsigned int j;
  int i;
  ptr = (GLuint *)buffer;

  for(i = 0 ; i < hits ; i++) {
    names = *ptr;
    ptr+=3;
    for(j = 0;j < names ; j++){
                  
      scannr = (*ptr)/1000000;
      pointnr = *ptr - (scannr * 1000000);
      cout << (*ptr) << " Scan: " << scannr << " Point: " << pointnr;
      cout << " x: " << Scan::allScans[scannr]->get_points()->at(pointnr).x
           << " y: " << Scan::allScans[scannr]->get_points()->at(pointnr).y
           << " z: " << Scan::allScans[scannr]->get_points()->at(pointnr).z;
      ptr++;
    }
    cout << endl;
  }

  cout << endl;
}


//------------------------------------------------------------------
/**
 * This function deals with all our keyboard activities
 */

void InterfaceFunc(unsigned char key){
 
  strncpy(path_file_name, path_filename_edit->get_text(), sizeof(GLUI_String));  
  strncpy(pose_file_name, pose_filename_edit->get_text(), sizeof(GLUI_String));  
  return;
}

//-----------------------------------------------------------------

void CallBackSpecialFunc(int key , int x, int y) {
  cout << "Called: CallBackSpecialFunc() ... " << endl;
  // return;
}



//---------------------------------------------------------------------------------------

/**
 * Function drawRobotPath
 * \brief This functions draws the path where the
 * robot has travelled along while taking the scans
 */

void drawRobotPath(int dummy){
  // TODO FIXME change to work with new camera path
  
  //clear the camera list as we are going to add the cameras
  //in the path where the robot travelled.

  //lets loop through the entire frame files to extract the
  //total number of places where the robot has taken the scans from
  for(unsigned int i = 0; i < MetaMatrix.size(); i++){

    //temp variable
    double *temp;
   
    
    //Now, lets go to the last of each frame file to
    //extract the transformation matrix obtained
    //after scan matching has been done.
    glMultMatrixd(MetaMatrix[i].back());

    //temp is final transformation matrix
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
   
  //signal for the update of scene
  haveToUpdate = 1;
}
//----------------------------------------------------------------------------------------------

int calcFrameNo(){
  //calculate the frame number
  //this ensures that we avoid the invalid frames

  int counter = 0;             //to store the frame no

  //check until a valid frame is found
  //any frame with colour code starting with
  // -1 is invalid
  
  while(MetaAlgoType[1][counter] == Scan::INVALID){   
    counter++;                 //increment the frame counter
   //  cout << "val is: " << MetaAlgoType[iterator][frameNr] << endl;
//     cout << "counter is: " << counter << endl;
  }

  return counter;
}

//----------------------------------------------------------------------------------------------

int calcNoOfPoints(vector<PointXY> vec1, vector<PointXY> vec2)
{
  double distance = 0.0;
  double dx, dy, dz;
  for(unsigned int i=0;i<vec1.size()-1;i++){
    dx = vec1.at(i+1).x - vec1.at(i).x;
    dy = vec1.at(i+1).y - vec1.at(i).y;
    dz = vec2.at(i+1).y - vec2.at(i).y;

    distance += sqrt(dx*dx + dy*dy + dz*dz );
  }

  return distance/10;
}

//-----------------------------------------------------------------

/**
 * This function handles the the keyboard input
 */

void CallBackKeyboardFunc(unsigned char key, int x, int y) {
  //call the interfacefunc. it deals with all our
  //keyboard activities
  InterfaceFunc(key);
}

void mapColorToValue(int dummy) {
  switch (listboxColorVal) {
    case 0:
      cm->setCurrentType(PointType<sfloat>::USE_HEIGHT);
      break;
    case 1:
      cm->setCurrentType(PointType<sfloat>::USE_REFLECTANCE);
      break;
    case 2:
      cm->setCurrentType(PointType<sfloat>::USE_AMPLITUDE);
      break;
    case 3:
      cm->setCurrentType(PointType<sfloat>::USE_DEVIATION);
      break;
    case 4:
      cm->setCurrentType(PointType<sfloat>::USE_TYPE);
      break;
    case 5:
      cm->setCurrentType(PointType<sfloat>::USE_COLOR);
      break;
    default:
      break;
  };
  resetMinMax(0);
}

void changeColorMap(int dummy) {
  ColorMap c;
  GreyMap gm;
  HSVMap hsv;
  SHSVMap shsv;
  JetMap jm;
  HotMap hot;
  DiffMap diff;

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
      cm->setMode(ScanColorManager<double>::MODE_STATIC);
      break;
    case 1: 
      cm->setMode(ScanColorManager<double>::MODE_COLOR_SCAN);
      break;
    case 2:
      cm->setMode(ScanColorManager<double>::MODE_POINT_COLOR);
      break;
    default:
      break;
  }
}


void changePointMode(int dummy) {
  if (dummy == 0) {           // always display
    if (pointmode != 1) {  // standard mode
      pointmode = 1;
      never_box->set_int_val(0);
    } else {
      pointmode = 0;
    }
  } else if (dummy == 1) {    // never display
    if (pointmode != -1) {  // standard mode
      pointmode = -1;
      always_box->set_int_val(0);
    } else {
      pointmode = 0;
    }
  }
}
