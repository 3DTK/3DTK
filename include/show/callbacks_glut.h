#ifndef _CALLBACKS_GLUT_H_
#define _CALLBACKS_GLUT_H_

namespace callbacks {
namespace glut{

/**
 * This function is called when the viewer is created. This
 * acts as the display function.
 */
void display();

/**
 * This function is called when there is nothing to be done
 * in the screen.
 */
void idle();

/**
 * This function is called after a mousebutton has been pressed.
 */
void mouseButton(int button, int state, int x, int y);

void quit();

void mouseMove(int x, int y);

/** Reshape Function
 * TODO: have to describe it.
 *
 */
void reshape(int width, int height);

/**
 * This function handles the the keyboard input
 */
void keyReleased(unsigned char key, int x, int y);

void keyPressed(unsigned char key, int x, int y);

}
}

#endif
