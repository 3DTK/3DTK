#include "viewcull.h"
#include <string.h>
#include "scancolormanager.h"

bool fullydisplayed = true;   // true if all points have been drawn to the screen
bool showall = false;         // true iff next DrawPoints should redraw scene with all points
bool mousemoving = false;     // true iff a mouse button has been pressed inside a window, but hs not been released
bool delayeddisplay = false;  // true iff mouse button callbacks should redraw the scene after button release
long ptstodisplay = 100000;  
double lastfps = idealfps;    // last frame rate    
int pointmode = -1;


/**
 * Displays all data (i.e., points) that are to be displayed
 * @param mode spezification for drawing to screen or in selection mode
 */
void DrawPoints(GLenum mode)
{
  long time = GetCurrentTimeInMilliSec();
  long max = 10000000000;
  long min = 10000;
  ptstodisplay *= 1.0 + (lastfps - idealfps)/idealfps;
  if (ptstodisplay < min) ptstodisplay = min;
  else if (ptstodisplay > max) ptstodisplay = max;


  // In case of animation
  if(scanNr != -1) {
    cm->setMode(ScanColorManager::MODE_ANIMATION);

    for(int iterator = (int)Scan::allScans.size()-1; iterator >= 0; iterator--) {

      if (MetaAlgoType[iterator][frameNr] == Scan::INVALID) continue;
      cm->selectColors(MetaAlgoType[iterator][frameNr]);	 
      glPushMatrix();
      glMultMatrixd(MetaMatrix[iterator][frameNr]);

      glPointSize(pointsize);
#ifdef USE_GL_POINTS
        ExtractFrustum();
          
        cm->selectColors(MetaAlgoType[iterator][frameNr]);
        if (pointmode == 1 || (showall && pointmode == 0) ) {
          octpts[iterator]->displayOctTreeAllCulled();
        } else {
          octpts[iterator]->displayOctTreeCulled(ptstodisplay/(int)Scan::allScans.size());
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

      for(int iterator = (int)Scan::allScans.size()-1; iterator >= 0; iterator--) {
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
        ExtractFrustum();
        if (pointmode == 1 || (showall && pointmode == 0) ) {
          octpts[iterator]->displayOctTreeAllCulled();
        } else {
          octpts[iterator]->displayOctTreeCulled(ptstodisplay/(int)Scan::allScans.size());
        }
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

  if (!showall) {
    lastfps =  1000.0/(GetCurrentTimeInMilliSec() - time);
    fullydisplayed = false;
  } else {
    fullydisplayed = true;
  }
  showall = false;         
}

/**
 * Draw a smooth path passing from all the camera points.
 *
 */
void DrawPath()
{
  
  // draw path
  glBegin(GL_LINE_STRIP);
  for(unsigned int j = 0; j < path_vectorX.size(); j++){
    // set the color 
    glColor4d(1.0, 1.0, 0.0, 1.0);
    // set the points
    glVertex3f(path_vectorX.at(j).x,path_vectorX.at(j).y,path_vectorZ.at(j).y);
  }
  glEnd();
  
  // draw lookat path
  glBegin(GL_LINE_STRIP);
  for(unsigned int j = 0; j < lookat_vectorX.size(); j++){
    //set the color 
    glColor4d(1.0, 0.0, 0.0, 1.0);
    //set the points
    glVertex3f(lookat_vectorX.at(j).x,lookat_vectorX.at(j).y,lookat_vectorZ.at(j).y);
  }
  glEnd();
}

/**
 * Draw the camera boxes in the viewer
 *
 */
void DrawCameras(void)
{
  for (int i = 0; i < cams.size(); i++) {
    glPushMatrix();
    
 // TODO improve upon this primitive camera   
    Point p = cams[i];
    Point l = lookats[i];

    
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
              0, 1, 0  );
    /*
    gluLookAt(camposition.x, camposition.y, camposition.z,
              lookatposition.x, lookatposition.y, lookatposition.z,
              0, 1, 0  );*/
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
    double *test;
    glColor4d(1.0, 0.0, 0.0, 1.0);
    glLineWidth(5);
    glBegin(GL_LINE_STRIP);
    for(unsigned int i = 0; i<Scan::allScans.size(); i++){
      if(frameNr > -1 && frameNr < (int)MetaMatrix[1].size()) {
	      if (MetaAlgoType[i][frameNr] == Scan::INVALID) continue;
        test = MetaMatrix[i][frameNr];
      } else {
        test = MetaMatrix[i].back();
      }
      if(showTopView) {
        glVertex3f(test[12], 2000, test[14]);
      } else {
        glVertex3f(test[12], test[13], test[14]);
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

  //calculate the position of the camera. we are referring
  //to the selected camera
  position = cams.begin()+ (cam_choice-1);
  positionL = lookats.begin()+ (cam_choice-1);

  //if no camera present then return
  if(cam_choice == 0)
    return;

  //if the list is not empty then
  if(!cams.empty()){
    //delete the camera from the position
    cams.erase(position);
    lookats.erase(positionL);
    //reset the cam_choice spinner values
    cam_spinner->set_int_limits( 1, cams.size());
    cam_spinner->set_int_val(cam_choice);
  }
  calcPath();
  calcLookAtPath();

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

void setView(double pos[3], double new_quat[4], 
             double newMouseRotX, double newMouseRotY, double newCangle)
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
  //pzoom = pzoom_new;
  //pzoom_spinner->set_float_val(pzoom);  
  //rotButton->reset();
  
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

  glDrawBuffer(GL_BACK);
  //glDrawBuffer(GL_FRONT);
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
      //pathAnimate1(path_iterator);
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
        cout << filename << endl;
        glDumpWindowPPM(filename.c_str(),0);

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
  double mat[16], obj_pos_button_new[3], obj_pos_button1[3];

  for (int i = 0; i < 3; i++) {
    if (fabs(obj_pos_button_old[i] - obj_pos_button[i]) > COMPARE_EPSILON) {
      obj_pos_button1[i] = obj_pos_button[i] - obj_pos_button_old[i];
      obj_pos_button_old[i] = obj_pos_button[i];
    } else obj_pos_button1[i] = 0.0;
  }

  QuaternionToMatrix4(quat, mat);
  obj_pos_button_new[0] =   obj_pos_button1[0] * mat[0]
    + obj_pos_button1[1] * mat[4]
    + obj_pos_button1[2] * mat[8];
  obj_pos_button_new[1] =   obj_pos_button1[0] * mat[1]
    + obj_pos_button1[1] * mat[5]
    + obj_pos_button1[2] * mat[9];
  obj_pos_button_new[2] =   obj_pos_button1[0] * mat[2]
    + obj_pos_button1[1] * mat[6]
    + obj_pos_button1[2] * mat[10];
    
  X = X + obj_pos_button_new[0];
  Y = Y + obj_pos_button_new[1];
  Z = Z + obj_pos_button_new[2];

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
void callCameraView(int dummy)
{
  Point campos(-X, -Y, -Z);

  // calculate lookat point
  Point lookat(0, 0, -200);
  double tmat[16];
  for (int i =0;i<16;i++) tmat[i] = view_rotate_button[i];
  lookat.transform(tmat);
  lookat.x = -lookat.x -X; 
  lookat.y = -lookat.y -Y; 
  lookat.z = lookat.z  -Z; 

  cams.push_back(campos);
  lookats.push_back(lookat);
  
  calcPath();
  calcLookAtPath();

  
  // now reset the value of the cam_choice spinner
  cam_spinner->set_int_limits( 1, cams.size() );
  cam_spinner->set_int_val(cams.size());
  
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
      if (haveToUpdate == 6) return;
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
        if (haveToUpdate == 6) return;
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

  GLuint selectBuf[BUFSIZE];
  GLint hits;
  GLint viewport[4];

  if(cameraNavMouseMode != 1) {
    if (state == GLUT_DOWN && (button == GLUT_LEFT_BUTTON || button == GLUT_RIGHT_BUTTON)) {

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
        if (haveToUpdate == 6) return;
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
  glReadBuffer(mode);
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
  int m,o,k,l;                  // Counter variables
  // Get viewport parameters
  double left, right, top, bottom;
  double tmp = 1.0/tan(rad(cangle)/2.0);
 
  // Get camera parameters
  GLdouble savedMatrix[16];
  glGetDoublev(GL_PROJECTION_MATRIX,savedMatrix);
   
  top = 1.0/tmp;
  bottom = -top;
  right = 1.25/tmp;
  left = -right;

  double part_height, part_width;
  part_width = (right - left)/(double)scale;
  part_height = (top - bottom)/(double)scale;
  
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

  double height = bottom;
  for(int i = 0; i < scale; i++) {
    double width = left;
    for(int j = 0; j < scale; j++) {
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      string imageFileName;
      glFrustum(width, width + part_width, height, height + part_height, 1.0, 40000.0);
      showall = true;
      DisplayItFunc(mode); 
      //imageFileName = "image" + to_string(i,3) + "_" + to_string(j,3) + ".ppm";
      //glDumpWindowPPM(imageFileName.c_str(),mode);
  
      // Read window contents from GL frame buffer with glReadPixels
      glFinish();
      glReadBuffer(mode);
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
  for(unsigned int i = 0; i<Scan::allScans.size(); i++) {

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
    Point lookat(0, 0, 200);
    double tmat[16];
    for (int i =0;i<16;i++) tmat[i] = temp[i]; 
    lookat.transform(tmat);
    lookat.x = lookat.x ; 
    lookat.y = lookat.y + 100; 
    lookat.z = lookat.z  ; 

    cams.push_back(campos);
    lookats.push_back(lookat);
  }
  calcPath();
  calcLookAtPath();

   
  //reset the cam_choice spinner
  cam_spinner->set_int_limits( 1, cams.size() ); 
  
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
      cm->setCurrentType(ScanColorManager::USE_HEIGHT);
      break;
    case 1:
      cm->setCurrentType(ScanColorManager::USE_REFLECTANCE);
      break;
    case 2:
      cm->setCurrentType(ScanColorManager::USE_AMPLITUDE);
      break;
    case 3:
      cm->setCurrentType(ScanColorManager::USE_DEVIATION);
      break;
    case 4:
      cm->setCurrentType(ScanColorManager::USE_TYPE);
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
  if (scans_colored) {
    cm->setMode(ScanColorManager::MODE_COLOR_SCAN);
  } else {
    cm->setMode(ScanColorManager::MODE_STATIC);
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
