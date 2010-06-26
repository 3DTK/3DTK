/**
 * @file
 * @brief Functions for the menu panels of the viewer software
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */
#include "colormanager.h"

// GUI variables

GLUI *glui1,  ///< pointer to the glui window(s)
     *glui2; ///< pointer to the glui window(s)

/** GLUI spinner for the fog */
GLUI_Spinner    *fog_spinner;

/** GLUI spinner for the point size */
GLUI_Spinner    *ps_spinner;

/** GLUI spinner for the current angle */
GLUI_Spinner    *cangle_spinner;
/** GLUI spinner for the current angle */
GLUI_Spinner    *pzoom_spinner;

int window_id_menu1, ///< menue window ids
    window_id_menu2; ///< menue window ids

/** User IDs for callbacks */
#define BOX_ID               201
/** User IDs for callbacks */
#define ENABLE_ID            300
/** User IDs for callbacks */
#define DISABLE_ID           301
/** User IDs for callbacks */
#define SHOW_ID              302
/** User IDs for callbacks */
#define HIDE_ID              303

/** Pointer to the panels */
GLUI_Panel      *ground_panel;
/** Pointer to the panels */
GLUI_Panel      *points_panel;
/** Pointer to the panels */
GLUI_Panel      *wireframe_panel;
/** Pointer to the panels */
GLUI_Panel      *path_panel;
/** Pointer to the panels */
GLUI_Panel      *camera_panel;
/** Pointer to the panels */
GLUI_Panel      *nav_panel;
/** Pointer to the panels */
GLUI_Panel      *mode_panel;
/** Pointer to the panels */
GLUI_Panel      *settings_panel;
/** Pointer to the button */
GLUI_Button     *button1;
/** Pointer to the edit text box*/
GLUI_EditText *path_filename_edit;
/** Pointer to the edit text box*/
GLUI_EditText *flength_edit;

/** Pointer to the rotation button */
GLUI_Rotation  *rotButton;

/** used for GLUI menue */
float obj_pos_button[3];
/** used for GLUI menue */
GLfloat view_rotate_button[16];
/** used for GLUI menue */
GLfloat obj_pos_button_old[3]; 

/** used for GLUI menue */
GLfloat X_button;
/** used for GLUI menue */
GLfloat Y_button;
/** used for GLUI menue */
GLfloat Z_button;
/** GLUI spinner for choosing the camera */
GLUI_Spinner    *cam_spinner;
/** GLUI spinner for choosing the animation delay */
GLUI_Spinner    *anim_spinner;
/** Panel for the camera controls **/
GLUI_Panel *cam_panel;
/** ListBox for choosing which value to map to a color  */
GLUI_Listbox    *value_listbox;
/** ListBox for choosing which color map to use  */
GLUI_Listbox    *colormap_listbox;
GLUI_Spinner    *mincol_spinner;
GLUI_Spinner    *maxcol_spinner;

/**
 * Generate the menu for the application.
 * It consists of control and selection menus.
 */
void newMenu()
{
  /*** Create the bottom subwindow ***/
  glui2 = GLUI_Master.create_glui("3D_Viewer - Controls");
  window_id_menu2 = glui2->get_glut_window_id();
  glutSetWindow(window_id_menu2);

  glutPositionWindow(START_X, START_Y + START_HEIGHT + 65);
  glutSetWindow(window_id);
  glui2->set_main_gfx_window( window_id );

  settings_panel = glui2->add_panel("Settings: ");
  
  static float dummy1;
  cangle_spinner = glui2->add_spinner_to_panel(settings_panel, "Apex Angle :   ", GLUI_SPINNER_FLOAT, &dummy1);
  cangle_spinner->set_float_limits( 1.0, 180.0 );
  cangle_spinner->set_speed( 20.0 );
  cangle_spinner->set_float_val(60.0);
  cangle_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  static float dummy2;
  pzoom_spinner = glui2->add_spinner_to_panel(settings_panel, "Parallel Zoom :", GLUI_SPINNER_FLOAT, &dummy2);
  pzoom_spinner->set_float_limits( 10.0, 50000.0 );
  pzoom_spinner->set_speed( 50.0 );
  pzoom_spinner->set_float_val(2000.0);
  pzoom_spinner->set_alignment( GLUI_ALIGN_RIGHT );
  pzoom_spinner->disable();

  flength_edit = glui2->add_edittext_to_panel(settings_panel,"  Focal Length :",GLUI_EDITTEXT_FLOAT,&flength);
  flength_edit->set_alignment( GLUI_ALIGN_LEFT );
 
  glui2->add_column( true );

  mode_panel = glui2->add_panel("Mode: ");
  
  /****** Top view *****/
  glui2->add_button_to_panel(mode_panel, "Top view", 0, callTopView )->set_alignment( GLUI_ALIGN_CENTER );

  
  /****** Reset button *****/
  glui2->add_button_to_panel(mode_panel, "Reset position", 0, resetView )->set_alignment( GLUI_ALIGN_CENTER );
  glui2->add_button_to_panel(mode_panel, "Camera Mode", 0, callCameraMode )->set_alignment( GLUI_ALIGN_CENTER );
  glui2->add_column( true );

  /****** Add Camera View *****/

 
  camera_panel = glui2->add_panel("Camera: ");
  cam_spinner = glui2->add_spinner_to_panel(camera_panel, "Choose Camera", GLUI_SPINNER_INT, &cam_choice);
  cam_spinner->set_int_limits( 0, 0 );
  cam_spinner->set_speed( 1 );
  cam_spinner->set_alignment( GLUI_ALIGN_LEFT );
  
  glui2->add_button_to_panel(camera_panel, "Add Camera", 0, callCameraView )->set_alignment( GLUI_ALIGN_CENTER );
  glui2->add_button_to_panel(camera_panel, "Delete Camera", 0, callDeleteCamera )->set_alignment( GLUI_ALIGN_CENTER );
  
  /******* Other navigation controls**********/
  glui2->add_column(true);

  nav_panel = glui2->add_panel("Navigation: ");
  rotButton = glui2->add_rotation_to_panel(nav_panel, "Rotation", view_rotate_button, -1, update_view_rotate);

  glui2->add_column_to_panel(nav_panel, true );
  glui2->add_translation_to_panel(nav_panel, "Move XY", GLUI_TRANSLATION_XY,
						    obj_pos_button, -1, update_view_translation);

  glui2->add_column_to_panel(nav_panel, false );
  glui2->add_translation_to_panel(nav_panel, "Move X", GLUI_TRANSLATION_X,
						    &obj_pos_button[0], -1, update_view_translation);

  glui2->add_column_to_panel(nav_panel, false );
  glui2->add_translation_to_panel(nav_panel, "Move Y", GLUI_TRANSLATION_Y,
						    &obj_pos_button[1], -1, update_view_translation);

  glui2->add_column_to_panel(nav_panel, false );
  glui2->add_translation_to_panel(nav_panel, "Move Z", GLUI_TRANSLATION_Z,
						    &obj_pos_button[2], -1, update_view_translation);

  glui2->add_column_to_panel(nav_panel, false);
  glui2->add_checkbox_to_panel(nav_panel, "MouseNav", &cameraNavMouseMode );
  glui2->set_glutMouseFunc(CallBackMouseFuncMoving);
  
  /*** Create the right subwindow ***/
  glui1 = GLUI_Master.create_glui("3D_Viewer - Selection");
  window_id_menu1 = glui1->get_glut_window_id();
  glutSetWindow(window_id_menu1);
  glutPositionWindow(START_X + START_WIDTH + 50, START_Y + 30);
  glutSetWindow(window_id);
  glui1->set_main_gfx_window( window_id );

  glui1->add_checkbox( "Draw Points", &show_points );
  glui1->add_checkbox( "Draw Camera", &show_cameras);
  glui1->add_checkbox( "Draw Path", &show_path);
  ps_spinner = glui1->add_spinner( "Point Size:", GLUI_SPINNER_FLOAT, &pointsize);
  ps_spinner->set_float_limits( 1.0, 5.0 );
  ps_spinner->set_speed( 25.0 );
  ps_spinner->set_float_val(pointsize);
  ps_spinner->set_alignment( GLUI_ALIGN_LEFT );  

  glui1->add_separator();

  /**** Fog Panel *****/
  
  GLUI_Panel *fogt_panel = glui1->add_rollout("Fog :", false );
  fogt_panel ->set_alignment( GLUI_ALIGN_LEFT );
  GLUI_RadioGroup *fogt = glui1-> add_radiogroup_to_panel( fogt_panel, &show_fog );
  glui1->add_radiobutton_to_group( fogt, "No Fog" );
  glui1->add_radiobutton_to_group( fogt, "Fog Black Exp" );
  glui1->add_radiobutton_to_group( fogt, "Fog Black Exp2" );
  glui1->add_radiobutton_to_group( fogt, "Fog Black Linear" );
  glui1->add_radiobutton_to_group( fogt, "Fog White Exp" );
  glui1->add_radiobutton_to_group( fogt, "Fog White Exp2" );
  glui1->add_radiobutton_to_group( fogt, "Fog White Linear" );

  
  
  
  fog_spinner = glui1->add_spinner( "Fog Density:", GLUI_SPINNER_FLOAT, &fogDensity);
  fog_spinner->set_float_limits( 0.0, 1.0 );
  fog_spinner->set_speed( 0.5 );
  fog_spinner->set_float_val(0.001);
  fog_spinner->set_alignment( GLUI_ALIGN_LEFT );  


  glui1->add_separator();

  /**** Path panel *******/

  path_panel = glui1->add_panel("Path:");
  path_filename_edit = glui1->add_edittext_to_panel(path_panel,"File: ",GLUI_EDITTEXT_TEXT, path_file_name);
  path_filename_edit->set_alignment( GLUI_ALIGN_LEFT );
  glui1->add_button_to_panel(path_panel, "Save Path   ", 0, savePath)->set_alignment( GLUI_ALIGN_CENTER);
  glui1->add_button_to_panel(path_panel, "Load Path   ", 0, loadPath)->set_alignment( GLUI_ALIGN_CENTER);
  glui1->add_button_to_panel(path_panel, "Load Robot P.", 0, drawRobotPath )->set_alignment( GLUI_ALIGN_CENTER );
  glui1->add_separator_to_panel(path_panel);
  glui1->add_checkbox_to_panel(path_panel, "Save Animation", &save_animation);
  glui1->add_button_to_panel(path_panel, "Animate Path", 0, pathAnimate)->set_alignment( GLUI_ALIGN_CENTER);
 
  
  glui1->add_separator();

  /****** Invert button *****/
  glui1->add_button( "Invert", 0, invertView )->set_alignment( GLUI_ALIGN_CENTER );
  /****** Animate button *****/
  anim_spinner = glui1->add_spinner( "Anim delay:", GLUI_SPINNER_INT, &anim_delay);
  anim_spinner->set_int_limits( 0, 100 );
  anim_spinner->set_speed( 1 );
  glui1->add_button( "Animate", 0, startAnimation )->set_alignment( GLUI_ALIGN_CENTER );
  
 
  glui1->add_separator();
  
  /****** Color Controls *****/
  if (types) {
    value_listbox = glui1->add_listbox( "Color values:", &listboxColorVal, 0, &mapColorToValue);
    value_listbox->set_alignment(GLUI_ALIGN_RIGHT);
    value_listbox->add_item(ColorManager::USE_HEIGHT, "height");
    if (types & ColorManager::USE_REFLECTANCE) 
      value_listbox->add_item(ColorManager::USE_REFLECTANCE, "reflectance");
    if (types & ColorManager::USE_AMPLITUDE) 
      value_listbox->add_item(ColorManager::USE_AMPLITUDE, "amplitude");
    if (types & ColorManager::USE_DEVIATION) 
      value_listbox->add_item(ColorManager::USE_DEVIATION, "deviation");

    colormap_listbox = glui1->add_listbox( "Colormap:   ", &listboxColorMapVal, 1, &changeColorMap);
    colormap_listbox->set_alignment(GLUI_ALIGN_RIGHT);
    colormap_listbox->add_item(0, "None");
    colormap_listbox->add_item(1, "Grey");
    colormap_listbox->add_item(2, "HSV");
    colormap_listbox->add_item(3, "Jet");
    colormap_listbox->add_item(4, "Hot");

    glui1->add_button( "Reset Min/Max", 0, &resetMinMax )->set_alignment( GLUI_ALIGN_CENTER );
    mincol_spinner = glui1->add_spinner( "Min Val:", GLUI_SPINNER_FLOAT, &mincolor_value, 0, &minmaxChanged);
    mincol_spinner->set_alignment(GLUI_ALIGN_RIGHT);
    maxcol_spinner = glui1->add_spinner( "Max Val:", GLUI_SPINNER_FLOAT, &maxcolor_value, 0, &minmaxChanged);
    maxcol_spinner->set_alignment(GLUI_ALIGN_RIGHT); 

    glui1->add_separator();
  }

  /****** A 'quit' button *****/
  glui1->add_button( "Quit", 0,(GLUI_Update_CB)exit )->set_alignment( GLUI_ALIGN_CENTER );

  /**** Link windows to GLUI, and register idle callback ******/  
  glutSetWindow(window_id);
  glui1->set_main_gfx_window( window_id );  // right
  glui2->set_main_gfx_window( window_id ); // bottom
  glui1->sync_live();
  glui2->sync_live();

  // cout << "Called : myNewMenu()...."<<endl;
  // cout << "show_points: " << show_points << endl;
  GLUI_Master.set_glutMouseFunc( CallBackMouseFunc );
  GLUI_Master.set_glutKeyboardFunc( CallBackKeyboardFunc );
  GLUI_Master.set_glutIdleFunc( CallBackIdleFunc );
  GLUI_Master.set_glutSpecialFunc( CallBackSpecialFunc );
}


/**
 * This function is called when a user starts to animate the generated path
 */
void pathAnimate(int dummy) {
  
  //signal that the screen needs to be repainted for animation
  haveToUpdate = 6;
  path_iterator = 0;
 
}
