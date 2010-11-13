/*
 * X.cpp
 *
 *  Created on: Feb 21, 2010
 *      Author: darko
 */


#include "X.h"
#include <string>
#include <iostream>
#include <string.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>


using namespace std;

X::X() {
	// TODO Auto-generated constructor stub

}

X::~X() {
	// TODO Auto-generated destructor stub
}

Window* X::GET_WND_REC(Display *dsp, Window *wnd, pid_t pid, std::string window_title)
{


//
//    Atom _atomPID = XInternAtom(dsp, "_NET_WM_PID", True);
//    if(_atomPID == None)
//    {
//		cout << "No such atom" << endl;
//		return NULL;
//    }
//
//    Atom           type;
//    int            format;
//    unsigned long  nItems;
//    unsigned long  bytesAfter;
//    unsigned char *propPID = 0;
//    if(Success == XGetWindowProperty(dsp, *wnd, _atomPID, 0, 1, False, XA_CARDINAL,
//                                     &type, &format, &nItems, &bytesAfter, &propPID))
//    {
//
//
//			if(propPID != 0)
//            {
//
////            // If the PID matches, add this window to the result set.
////                    if(_pid == *((unsigned long *)propPID))
////                            _result.push_back(w);
////
//					cout << *propPID << endl;
//                    XFree(propPID);
//            }
//    }

	//TODO check also pid

	XTextProperty prop;
	int status = XGetWMName(dsp, *wnd, &prop);
	if (status != 0) {
		if (!strcmp((const char*) prop.value, window_title.c_str())) {
			cout << "window found" << endl;
//			cout << wnd << endl;
			return wnd;
		}
	}

	Window    wRoot;
	Window    wParent;
	Window   *wChild;
	unsigned  nChildren;
	if(0 != XQueryTree(dsp, *wnd, &wRoot, &wParent, &wChild, &nChildren)) {
		for(unsigned i = 0; i < nChildren; i++) {
			Window *res;
			res = X::GET_WND_REC(dsp, (Window*) &wChild[i], pid, window_title);
			if (res != NULL) {
				return res;
			}
		}
	}
	return NULL;
}



void X::fixMouse(Display *dsp, Window *wnd, int x, int y)
{
	Display *ds;
	ds = XOpenDisplay(0);
	XWarpPointer(ds, *wnd, *wnd, 0, 0, 0, 0, x, y);
	XCloseDisplay(ds);

}


Window* X::getWindow(Display *dsp, pid_t pid, std::string window_title)
{

	Window root;
	root = XDefaultRootWindow(dsp);
	Window *wnd = GET_WND_REC(dsp, &root, pid, window_title);
	return wnd;

}




