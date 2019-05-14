/*
 * show_menu implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __SHOW_MENU_H__
#define __SHOW_MENU_H__

#include "show/show_common.h"

#ifdef WITH_OPENGL
#include <GL/glui.h>
#else
#include "show/dummygl.h"
#endif

/**
 * @file
 * @brief Functions for the menu panels of the viewer software
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */
#include "show/colormanager.h"

// GUI variables

extern GLUI *glui1,  ///< pointer to the glui window(s)
            *glui2;  ///< pointer to the glui window(s)

/** GLUI spinner for the fog */
extern GLUI_Spinner    *fog_spinner;

/** GLUI spinner for the point size */
extern GLUI_Spinner    *ps_spinner;

/** GLUI spinner for the current angle */
extern GLUI_Spinner    *cangle_spinner;
/** GLUI spinner for the current angle */
extern GLUI_Spinner    *pzoom_spinner;
/** GLUI spinner for the current angle */
extern GLUI_Spinner    *rzoom_spinner;
/** GLUI spinner for the factor for the image size */
extern GLUI_Spinner    *image_spinner;
/** GLUI_Spinner for the depth to select groups of points */
extern GLUI_Spinner    *depth_spinner;
extern GLUI_Spinner    *brushsize_spinner;
extern GLUI_Spinner    *frame_spinner;
extern GLUI_Spinner    *fps_spinner;
extern GLUI_Spinner    *farplane_spinner;
extern GLUI_Spinner    *nearplane_spinner;
extern GLUI_Spinner    *lod_spinner;
extern GLUI_Spinner    *threed_spinner;

/** GLUI_Spinner for changing the scans displayed */
extern GLUI_Spinner    *startScanIdx_spinner;
extern GLUI_Spinner    *endScanIdx_spinner;



extern int window_id_menu1, ///< menue window ids
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
extern GLUI_Panel      *ground_panel;
/** Pointer to the panels */
extern GLUI_Panel      *points_panel;
/** Pointer to the panels */
extern GLUI_Panel      *wireframe_panel;
/** Pointer to the panels */
extern GLUI_Panel      *path_panel;
/** Pointer to the panels */
extern GLUI_Panel      *pose_panel;
/** Pointer to the panels */
extern GLUI_Panel      *selection_panel;
/** Pointer to the panels */
extern GLUI_Panel      *color_panel;
/** Pointer to the panels */
extern GLUI_Panel      *camera_panel;
/** Pointer to the panels */
extern GLUI_Panel      *nav_panel;
/** Pointer to the panels */
extern GLUI_Panel      *mode_panel;
/** Pointer to the panels */
extern GLUI_Panel      *settings_panel;
/** Pointer to the panels */
extern GLUI_Panel      *advanced_panel;
/** Pointer to the button */
extern GLUI_Button     *button1;
/** Pointer to the edit text box*/
extern GLUI_EditText *path_filename_edit;
/** Pointer to the edit text box*/
extern GLUI_EditText *pose_filename_edit;
/** Pointer to the edit text box*/
extern GLUI_EditText *selection_filename_edit;

/** Pointer to the rotation button */
extern GLUI_Rotation  *rotButton;

/** used for GLUI menue */
extern float obj_pos_button[3];
/** used for GLUI menue */
extern GLfloat view_rotate_button[16];
/** used for GLUI menue */
extern GLfloat obj_pos_button_old[3];

/** used for GLUI menue */
extern GLfloat X_button;
/** used for GLUI menue */
extern GLfloat Y_button;
/** used for GLUI menue */
extern GLfloat Z_button;
/** GLUI spinner for choosing the camera */
extern GLUI_Spinner    *cam_spinner;
/** GLUI spinner for choosing the animation factor */
extern GLUI_Spinner    *anim_spinner;
/** GLUI spinner for choosing the interpolation factor */
extern GLUI_Spinner    *interp_spinner;
/** Panel for the camera controls **/
extern GLUI_Panel *cam_panel;
/** ListBox for choosing which value to map to a color  */
extern GLUI_Listbox    *value_listbox;
/** ListBox for choosing which color map to use  */
extern GLUI_Listbox    *colormap_listbox;
extern GLUI_Spinner    *mincol_spinner;
extern GLUI_Spinner    *maxcol_spinner;

/** Checkboxes for changing point display mode **/
extern GLUI_Checkbox *always_box;
extern GLUI_Checkbox *never_box;
/** Checkbox for changing interpolation mode **/
extern GLUI_Checkbox *interpol_box;

/**
 * Generate the menu for the application.
 * It consists of control and selection menus.
 */
void newMenu();

/**
 * This function is called when the scan range is supposed to be increased by
 * one.
 */
void stepScansUp(int dummy);


void stepScansDown(int dummy);


/**
 * This function is called when a user starts to animate the generated path
 */
void pathAnimate(int dummy);

void pathMatchingAnimate(int dummy);

	/**
 * This function clears the selected points
 */
void clearSelection(int dummy);


/**
 * Loads selected points from a file.
 *
 * Dependent on output of saveSelection function - if changes are made to the output format of saved points they need to be reflected here!!
 */
void loadSelection(int dummy);

#endif
