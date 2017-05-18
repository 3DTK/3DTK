#ifndef __GLWIDGET_H__
#define __GLWIDGET_H__

#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QWidget>

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  GLWidget(QWidget *parent = 0);

public slots:
  void setDrawPoints(bool drawPoints);
  void setDrawCameras(bool drawCameras);
  void setDrawPath(bool drawPath);
  void setPointSize(int pointSize);

  void setFogType(int fogType);
  void setFogInverted(bool fogInverted);
  void setFogDensity(double fogDensity);

  void setColorProperty(int colorProperty);
  void setColorMinimumValue(double colorMinimum);
  void setColorMaximumValue(double colorMaximum);
  void setAutoRecalculateColor(bool autoRecalculate);
  void recalculateColor();
  void setColorType(int colorType);
  void setColorMap(int colorMap);
  void setColorInverted(bool colorInverted);

signals:
  void colorMinimumValueChanged(double);
  void colorMaximumValueChanged(double);

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void updateFogType();


protected:
  QPoint lastMousePos;

  int fogType;
  bool fogInverted;

  bool autoRecalculateColor;

};

#endif
