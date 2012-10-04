/*
 * show_menu implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Functions for the menu panels of the viewer software
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */
#include "show/colormanager.h"

// GUI variables

GLUI *glui1,  ///< pointer to the glui window(s)
     *glui2;  ///< pointer to the glui window(s)

/** GLUI spinner for the fog */
GLUI_Spinner    *fog_spinner;

/** GLUI spinner for the point size */
GLUI_Spinner    *ps_spinner;

/** GLUI spinner for the current angle */
GLUI_Spinner    *cangle_spinner;
/** GLUI spinner for the current angle */
GLUI_Spinner    *pzoom_spinner;
/** GLUI spinner for the factor for the image size */
GLUI_Spinner    *image_spinner;
/** GLUI_Spinner for the depth to select groups of points */
GLUI_Spinner    *depth_spinner;
GLUI_Spinner    *brushsize_spinner;
GLUI_Spinner    *frame_spinner;
GLUI_Spinner    *fps_spinner;
GLUI_Spinner    *farplane_spinner;
GLUI_Spinner    *nearplane_spinner;
GLUI_Spinner    *lod_spinner;

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
GLUI_Panel      *pose_panel;
/** Pointer to the panels */
GLUI_Panel      *selection_panel;
/** Pointer to the panels */
GLUI_Panel      *color_panel;
/** Pointer to the panels */
GLUI_Panel      *camera_panel;
/** Pointer to the panels */
GLUI_Panel      *nav_panel;
/** Pointer to the panels */
GLUI_Panel      *mode_panel;
/** Pointer to the panels */
GLUI_Panel      *settings_panel;
/** Pointer to the panels */
GLUI_Panel      *advanced_panel;
/** Pointer to the button */
GLUI_Button     *button1;
/** Pointer to the edit text box*/
GLUI_EditText *path_filename_edit;
/** Pointer to the edit text box*/
GLUI_EditText *pose_filename_edit;
/** Pointer to the edit text box*/
GLUI_EditText *selection_filename_edit;

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

/** Checkboxes for changing point display mode **/
GLUI_Checkbox *always_box;
GLUI_Checkbox *never_box;
/** Checkbox for changing interpolation mode **/
GLUI_Checkbox *interpol_box;

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
  
  cangle_spinner = glui2->add_spinner_to_panel(settings_panel, "Field of View :   ", GLUI_SPINNER_FLOAT, &cangle);
  cangle_spinner->set_float_limits( 1.0, 180.0 );
  cangle_spinner->set_speed( 20.0 );
  cangle_spinner->set_float_val(60.0);
  cangle_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  pzoom_spinner = glui2->add_spinner_to_panel(settings_panel, "Parallel Zoom :", GLUI_SPINNER_FLOAT, &pzoom);
  pzoom_spinner->set_float_limits( 10.0, 50000.0 );
  pzoom_spinner->set_speed( 5.0 );
  pzoom_spinner->set_float_val(2000.0);
  pzoom_spinner->set_alignment( GLUI_ALIGN_RIGHT );
  pzoom_spinner->disable();

  glui2->add_column( true );

  mode_panel = glui2->add_panel("Mode: ");
  
  /****** Top view *****/
  glui2->add_button_to_panel(mode_panel, "Top view", 0, callTopView )->set_alignment( GLUI_ALIGN_CENTER );

  
  /****** Reset button *****/
  glui2->add_button_to_panel(mode_panel, "Reset position", 0, resetView )->set_alignment( GLUI_ALIGN_CENTER );
  glui2->add_column( true );

  /****** Add Camera View *****/

 
  camera_panel = glui2->add_panel("Camera: ");
  cam_spinner = glui2->add_spinner_to_panel(camera_panel, "Choose Camera", GLUI_SPINNER_INT, &cam_choice);
  cam_spinner->set_int_limits( 0, 0 );
  cam_spinner->set_speed( 1 );
  cam_spinner->set_alignment( GLUI_ALIGN_LEFT );
  
  glui2->add_button_to_panel(camera_panel, "Add Camera", 1, callAddCamera )->set_alignment( GLUI_ALIGN_CENTER );
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
  
  static int dummy4;
  always_box = glui2->add_checkbox_to_panel(nav_panel, "Always all Points", &dummy4, 0, &changePointMode);
  glui2->set_glutMouseFunc(CallBackMouseFuncMoving);
  static int dummy5 = 1;
  never_box =  glui2->add_checkbox_to_panel(nav_panel, "Always reduce Points", &dummy5, 1, &changePointMode );
  
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
  ps_spinner->set_float_limits( 0.0000001, 10.0 );
  ps_spinner->set_speed( 25.0 );
  ps_spinner->set_float_val(pointsize);
  ps_spinner->set_alignment( GLUI_ALIGN_LEFT );  


  /**** Fog Panel *****/
  
  GLUI_Panel *fogt_panel = glui1->add_rollout("Fog :", false );
  fogt_panel ->set_alignment( GLUI_ALIGN_LEFT );
  GLUI_RadioGroup *fogt = glui1-> add_radiogroup_to_panel( fogt_panel, &show_fog );
  glui1->add_radiobutton_to_group( fogt, "No Fog" );
  glui1->add_radiobutton_to_group( fogt, "Fog Exp" );
  glui1->add_radiobutton_to_group( fogt, "Fog Exp2" );
  glui1->add_radiobutton_to_group( fogt, "Fog Linear" );
  glui1->add_radiobutton_to_group( fogt, "inverted Fog Exp" );
  glui1->add_radiobutton_to_group( fogt, "inverted Fog Exp2" );
  glui1->add_radiobutton_to_group( fogt, "inverted Fog Linear" );

  
  
  
  fog_spinner = glui1->add_spinner( "Fog Density:", GLUI_SPINNER_FLOAT, &fogDensity);
  fog_spinner->set_float_limits( 0.0, 1.0 );
  fog_spinner->set_speed( 0.5 );
  fog_spinner->set_float_val(0.001);
  fog_spinner->set_alignment( GLUI_ALIGN_LEFT );  


  /****** Color Controls *****/
  color_panel = glui1->add_rollout("Color :", false );
  color_panel ->set_alignment( GLUI_ALIGN_LEFT );

  GLUI_Panel *color_ro = glui1->add_rollout_to_panel(color_panel, "Color values:", false);
  color_ro->set_alignment(GLUI_ALIGN_LEFT);

  GLUI_RadioGroup *color_rog = glui1->add_radiogroup_to_panel( color_ro, &listboxColorVal, 0, &mapColorToValue );
  glui1->add_radiobutton_to_group( color_rog, "height");
  GLUI_RadioButton *rbrefl =  glui1->add_radiobutton_to_group( color_rog, "reflectance");
  GLUI_RadioButton *rbtemp =  glui1->add_radiobutton_to_group( color_rog, "temperature");
  GLUI_RadioButton *rbampl = glui1->add_radiobutton_to_group( color_rog, "amplitude");
  GLUI_RadioButton *rbdevi = glui1->add_radiobutton_to_group( color_rog, "deviation");
  GLUI_RadioButton *rbtype = glui1->add_radiobutton_to_group( color_rog, "type");
  //if (!(types & PointType::USE_REFLECTANCE)) rbrefl->disable(); 
  //if (!(types & PointType::USE_AMPLITUDE)) rbampl->disable();
  //if (!(types & PointType::USE_DEVIATION)) rbdevi->disable(); 
  //if (!(types & PointType::USE_TYPE)) rbtype->disable(); 
  if (!(pointtype.hasReflectance())) rbrefl->disable(); 
  if (!(pointtype.hasTemperature())) rbtemp->disable(); 
  if (!(pointtype.hasAmplitude())) rbampl->disable();
  if (!(pointtype.hasDeviation())) rbdevi->disable(); 
  if (!(pointtype.hasType())) rbtype->disable(); 

  GLUI_Panel *colorm_ro = glui1->add_rollout_to_panel(color_panel, "Colormap:", false);
  colorm_ro->set_alignment(GLUI_ALIGN_LEFT);

  GLUI_RadioGroup *colorm_rog = glui1->add_radiogroup_to_panel(colorm_ro, &listboxColorMapVal, 0, &changeColorMap);
  glui1->add_radiobutton_to_group(colorm_rog, "Solid");
  glui1->add_radiobutton_to_group(colorm_rog, "Grey");
  glui1->add_radiobutton_to_group(colorm_rog, "HSV");
  glui1->add_radiobutton_to_group(colorm_rog, "Jet");
  glui1->add_radiobutton_to_group(colorm_rog, "Hot");
  glui1->add_radiobutton_to_group(colorm_rog, "Rand");
  glui1->add_radiobutton_to_group(colorm_rog, "SHSV");
  glui1->add_radiobutton_to_group(colorm_rog, "TEMP");

  GLUI_Panel *scans_color = glui1->add_rollout_to_panel(color_panel, "Color type:", false);
  scans_color->set_alignment(GLUI_ALIGN_LEFT);
  GLUI_RadioGroup *scans_colored = glui1->add_radiogroup_to_panel(scans_color, &colorScanVal, 0, &setScansColored);
  glui1->add_radiobutton_to_group(scans_colored, "None");
  glui1->add_radiobutton_to_group(scans_colored, "Id Scans by Color");
  GLUI_RadioButton *colorb = glui1->add_radiobutton_to_group( scans_colored, "Color by Points");
  if (!(pointtype.hasColor())) colorb->disable(); 

  mincol_spinner = glui1->add_spinner_to_panel(color_panel, "Min Val:", GLUI_SPINNER_FLOAT, &mincolor_value, 0, &minmaxChanged);
  mincol_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  maxcol_spinner = glui1->add_spinner_to_panel(color_panel, "Max Val:", GLUI_SPINNER_FLOAT, &maxcolor_value, 0, &minmaxChanged);
  maxcol_spinner->set_alignment(GLUI_ALIGN_RIGHT); 
  glui1->add_button_to_panel(color_panel, "Reset Min/Max", 0, &resetMinMax )->set_alignment( GLUI_ALIGN_CENTER );

  glui1->add_separator();
 
  /****** Invert button *****/
  glui1->add_button( "Invert", 0, invertView )->set_alignment( GLUI_ALIGN_CENTER );
  /****** Animate button *****/
  anim_spinner = glui1->add_spinner( "Anim delay:", GLUI_SPINNER_INT, &anim_delay);
  anim_spinner->set_int_limits( 0, 100 );
  anim_spinner->set_speed( 1 );
  glui1->add_button( "Animate", 0, startAnimation )->set_alignment( GLUI_ALIGN_CENTER );
  
  glui1->add_separator();

  
/**** Path panel *******/

  path_panel = glui1->add_rollout("Camera Path :", false );
  path_panel ->set_alignment( GLUI_ALIGN_LEFT );
  path_filename_edit = glui1->add_edittext_to_panel(path_panel,"File: ",GLUI_EDITTEXT_TEXT, path_file_name);
  path_filename_edit->set_alignment( GLUI_ALIGN_LEFT );
  glui1->add_button_to_panel(path_panel, "Save Path   ", 0, savePath)->set_alignment( GLUI_ALIGN_CENTER);
  glui1->add_button_to_panel(path_panel, "Load Path   ", 0, loadPath)->set_alignment( GLUI_ALIGN_CENTER);
  glui1->add_button_to_panel(path_panel, "Load Robot P.", 0, drawRobotPath )->set_alignment( GLUI_ALIGN_CENTER );
  glui1->add_separator_to_panel(path_panel);
  glui1->add_checkbox_to_panel(path_panel, "Save Animation", &save_animation);
  interpol_box = glui1->add_checkbox_to_panel(path_panel, "Interpolate by Distance", &inter_by_dist, -1, &callCameraUpdate);
  //always_box = glui2->add_checkbox_to_panel(nav_panel, "Always all Points", &dummy4, 0, &changePointMode);
  //glui1->add_checkbox_to_panel(path_panel, "Interpolate by Distance", &inter_by_dist);
  glui1->add_button_to_panel(path_panel, "Animate Path", 0, pathAnimate)->set_alignment( GLUI_ALIGN_CENTER);

  /**** Position panel *******/
  
  pose_panel = glui1->add_rollout("Position :", false );
  pose_panel ->set_alignment( GLUI_ALIGN_LEFT );
  pose_filename_edit = glui1->add_edittext_to_panel(pose_panel,"File: ",GLUI_EDITTEXT_TEXT, pose_file_name);
  pose_filename_edit->set_alignment( GLUI_ALIGN_LEFT );
  glui1->add_button_to_panel(pose_panel, "Save Pose   ", 0, savePose)->set_alignment( GLUI_ALIGN_CENTER);
  glui1->add_button_to_panel(pose_panel, "Load Pose   ", 0, loadPose)->set_alignment( GLUI_ALIGN_CENTER);
  image_spinner = glui1->add_spinner_to_panel(pose_panel, "Factor :   ",
GLUI_SPINNER_INT, &factor);
  image_spinner->set_int_limits( 1, 10 );
  image_spinner->set_speed( 1 );
  image_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  glui1->add_button_to_panel(pose_panel, "Save Image   ", 0, saveImage)->set_alignment( GLUI_ALIGN_CENTER);
  
 
  glui1->add_separator();
  
  /**** Selection panel ******/
  selection_panel = glui1->add_rollout("Selection :", false );
  selection_panel ->set_alignment( GLUI_ALIGN_LEFT );
  
  selection_filename_edit = glui1->add_edittext_to_panel(selection_panel,"File: ",GLUI_EDITTEXT_TEXT, selection_file_name);
  selection_filename_edit->set_alignment( GLUI_ALIGN_LEFT );
  glui1->add_button_to_panel(selection_panel, "Save selected points   ", 0, saveSelection)->set_alignment( GLUI_ALIGN_CENTER);

  glui1->add_button_to_panel(selection_panel, "Clear selected points   ", 0, clearSelection)->set_alignment( GLUI_ALIGN_CENTER);
  glui1->add_checkbox_to_panel(selection_panel, "Select/Unselect", &selectOrunselect);
  glui1->add_checkbox_to_panel(selection_panel, "Select Voxels", &select_voxels);
  depth_spinner = glui1->add_spinner_to_panel(selection_panel, "Depth :   ",
      GLUI_SPINNER_INT, &selection_depth);
  depth_spinner->set_int_limits( 1, 100 );
  depth_spinner->set_speed( 1 );
  depth_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  brushsize_spinner = glui1->add_spinner_to_panel(selection_panel, "Brushsize :   ",
      GLUI_SPINNER_INT, &brush_size);
  brushsize_spinner->set_int_limits( 0, 100 );
  brushsize_spinner->set_speed( 1 );
  brushsize_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  
 
  glui1->add_separator();
  /**** Advanced panel ******/
  if (advanced_controls) {
    advanced_panel = glui1->add_rollout("Advanced :", false );
    advanced_panel->set_alignment( GLUI_ALIGN_LEFT );

    //  glui1->add_edittext_to_panel(advanced_panel,"Frame #: ",GLUI_EDITTEXT_TEXT, current_frame)->set_alignment( GLUI_ALIGN_LEFT );
    frame_spinner = glui1->add_spinner_to_panel(advanced_panel, "Frame #:   ",
        GLUI_SPINNER_INT, &current_frame);
    frame_spinner->set_int_limits( 0, MetaMatrix[0].size()-1 );
    frame_spinner->set_speed( 10 );
    frame_spinner->set_alignment(GLUI_ALIGN_RIGHT);

    fps_spinner = glui1->add_spinner_to_panel(advanced_panel, "FPS :   ",
        GLUI_SPINNER_FLOAT, &idealfps);
    fps_spinner->set_int_limits( 0, 100 );
    fps_spinner->set_speed( 1 );
    fps_spinner->set_alignment(GLUI_ALIGN_RIGHT);

    farplane_spinner = glui1->add_spinner_to_panel(advanced_panel, "farplane :   ",
        GLUI_SPINNER_FLOAT, &maxfardistance);
    farplane_spinner->set_float_limits( 1, 100000 );
    farplane_spinner->set_speed( 1 );
    farplane_spinner->set_alignment(GLUI_ALIGN_RIGHT);
    
    nearplane_spinner = glui1->add_spinner_to_panel(advanced_panel, "nearplane :   ",
        GLUI_SPINNER_FLOAT, &neardistance);
    nearplane_spinner->set_int_limits( 1, 100000 );
    nearplane_spinner->set_speed( 1 );
    nearplane_spinner->set_alignment(GLUI_ALIGN_RIGHT);

    glui1->add_button_to_panel(advanced_panel, "Cycle LOD", 0,(GLUI_Update_CB)cycleLOD )->set_alignment( GLUI_ALIGN_CENTER );

    lod_spinner = glui1->add_spinner_to_panel(advanced_panel, "lod speed :   ",
        GLUI_SPINNER_FLOAT, &adaption_rate);
    lod_spinner->set_float_limits( 0, 3.0 );
    lod_spinner->set_speed( 0.1 );
    lod_spinner->set_alignment(GLUI_ALIGN_RIGHT);
    
    glui1->add_separator();
  }

  /****** A 'quit' button *****/
  glui1->add_button( "Quit", 0,(GLUI_Update_CB)exit )->set_alignment( GLUI_ALIGN_CENTER );

  glui1->set_glutMouseFunc(CallBackMouseFuncMoving);
  /**** Link windows to GLUI, and register idle callback ******/  
  glutSetWindow(window_id);
  glui1->set_main_gfx_window( window_id );  // right
  glui2->set_main_gfx_window( window_id ); // bottom
  glui1->sync_live();
  glui2->sync_live();

  // cout << "Called : myNewMenu()...."<<endl;
  // cout << "show_points: " << show_points << endl;
  GLUI_Master.set_glutMouseFunc( CallBackMouseFunc );
  GLUI_Master.set_glutKeyboardFunc( CallBackInterfaceFunc );
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


/**
 * This function clears the selected points  
 */
void clearSelection(int dummy) {
  for(int iterator = (int)octpts.size()-1; iterator >= 0; iterator--) 
    selected_points[iterator].clear();
}

