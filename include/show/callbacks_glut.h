#ifndef _CALLBACKS_GLUT_H_
#define _CALLBACKS_GLUT_H_

namespace callbacks {
namespace glut{

/**
 * This function is called when the viewer is created. This
 * acts as the display function.
 */
void CallBackDisplayFunc();

/**
 * This function is called when there is nothing to be done
 * in the screen.
 */
void CallBackIdleFunc(void);

void CallBackMouseFuncMoving(int button, int state, int x, int y);

/**
 * This function is called after a mousebutton has been pressed.
 */
void CallBackMouseFunc(int button, int state, int x, int y);

void CallBackCloseFunc();

void CallBackMouseMotionFunc(int x, int y);

/** Reshape Function
 * TODO: have to describe it.
 *
 */
void CallBackReshapeFunc(int width, int height);

void CallBackSpecialFunc(int key, int x, int y);

/**
 * This function handles the the keyboard input
 */
void CallBackInterfaceFunc(unsigned char key, int x, int y);

void CallBackKeyboardUpFunc(unsigned char key, int x, int y);

void CallBackKeyboardFunc(unsigned char key, int x, int y);

}
}

#endif
