#include <cmath>

#include <GL/glu.h>

#include "qtshow/GLWidget.h"

void CallBackReshapeFunc(int w, int h);
void DisplayItFunc(GLenum mode, bool interruptable = false);
extern GLfloat cangle, cangle_old, pzoom, pzoom_old;
extern GLenum buffermode;

GLWidget::GLWidget(QWidget *parent)
  : QOpenGLWidget(parent)
{}

void GLWidget::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void GLWidget::resizeGL(int w, int h) {
  CallBackReshapeFunc(w, h);
}

void GLWidget::paintGL() {
  if (fabs(cangle_old - cangle) > 0.5 ||
      fabs(pzoom_old - pzoom) > 0.5) {

    cangle_old = cangle;
    pzoom_old = pzoom;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    CallBackReshapeFunc(viewport[2],viewport[3]);
  }

  glDrawBuffer(buffermode);
  // delete framebuffer and z-buffer

  // Call the display function
  DisplayItFunc(GL_RENDER );

}
