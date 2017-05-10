#ifndef __GLWIDGET_H__
#define __GLWIDGET_H__

#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QWidget>

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
public:
  GLWidget(QWidget *parent = 0);

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);

protected:
  QPoint lastMousePos;
};

#endif
