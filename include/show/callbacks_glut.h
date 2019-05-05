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

void mouseMoveDelta(double deltaMouseX, double deltaMouseY);

/** Reshape Function
 * TODO: have to describe it.
 *
 */
void reshape(int width, int height);

/** Callback for GLUT. Records the key release in ::keymap.
 */
void keyReleased(unsigned char key, int x, int y);

/** Callback for GLUT. Records the key press in ::keymap, then delegates to keyHandler().
 */
void keyPressed(unsigned char key, int x, int y);

/** Does the actual work on keypress. This function is used both from show and wxShow, which is why `key` is sized as `int`.
 *
 * Mainly moves the camera, with different speeds according to modifier keys. Also toggles fullscreen.
 *
 * @param key Key ID, which is an ASCII char for GLUT and some int for WX
 * @param control whether Ctrl is pressed
 * @param alt whether Alt is pressed
 * @param shift whether Shift is pressed
 */
void keyHandler(int key, bool control, bool alt, bool shift);

}
}

#endif
