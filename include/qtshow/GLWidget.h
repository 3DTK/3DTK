#ifndef __GLWIDGET_H__
#define __GLWIDGET_H__

#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QWidget>

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
};

#endif
