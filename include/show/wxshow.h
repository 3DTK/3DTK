#ifndef _glpane_
#define _glpane_

#include "GL/glut.h"
#include "wx/wx.h"
#include "wx/glcanvas.h"
 
class BasicGLPane : public wxGLCanvas
{
    wxGLContext*  m_context;
 
public:
  BasicGLPane(wxFrame* parent, int* args);
  virtual ~BasicGLPane();
    
  void resized(wxSizeEvent& evt);
    
  int getWidth();
  int getHeight();
    
  void render(wxPaintEvent& evt);
  void paint(); 
  void prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
    
  // events
  void idle();
  void idle(wxIdleEvent& event);
  void mouseMoved(wxMouseEvent& event);
  void mouseEvent(wxMouseEvent& event);
  void mouseDown(wxMouseEvent& event);
  void mouseWheelMoved(wxMouseEvent& event);
  void mouseReleased(wxMouseEvent& event);
  void rightClick(wxMouseEvent& event);
  void mouseLeftWindow(wxMouseEvent& event);
  void keyPressed(wxKeyEvent& event);
  void keyReleased(wxKeyEvent& event);
    
  DECLARE_EVENT_TABLE()
};
#endif 
