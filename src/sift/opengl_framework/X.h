/*
 * X.h
 *
 *  Created on: Feb 21, 2010
 *      Author: darko
 */

#ifndef X_H_
#define X_H_

#include <string>
#include <X11/Xlib.h>

class X {

private:
	X();
	virtual ~X();
	static Window* GET_WND_REC(Display *dsp, Window *wnd, pid_t pid, std::string window_title);

public:

	static void fixMouse(Display *dsp, Window *w, int x, int y);
	static Window* getWindow(Display *dsp, pid_t pid, std::string window_title);

};

#endif /* X_H_ */
