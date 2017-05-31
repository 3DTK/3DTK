#include <cmath>
#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include <GL/glu.h>
#endif

#include "show/show_common.h"
#include "show/show_gl.h"
#include "qtshow/GLWidget.h"

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
  DisplayItFunc(GL_RENDER, false);
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
  lastMousePos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  float dx = event->x() - lastMousePos.x();
  float dy = event->y() - lastMousePos.y();

  if (event->buttons() & Qt::LeftButton) {
    moveCamera(0,0,0,dy,dx,0);
    update();
  }

  lastMousePos = event->pos();
}

void GLWidget::setDrawPoints(bool drawPoints) {
  show_points = drawPoints;
  update();
}

void GLWidget::setDrawCameras(bool drawCameras) {
  show_cameras = drawCameras;
  update();
}

void GLWidget::setDrawPath(bool drawPath) {
  show_path = drawPath;
  update();
}

void GLWidget::setPointSize(int pointSize) {
  pointsize = pointSize;
  update();
}

void GLWidget::setFogType(int fogType) {
  this->fogType = fogType;
  updateFogType();
  update();
}

void GLWidget::setFogInverted(bool fogInverted) {
  this->fogInverted = fogInverted;
  updateFogType();
  update();
}

void GLWidget::updateFogType() {
  show_fog = fogType;
  if (fogInverted && fogType != 0) {
    show_fog += 3;
  }
}

void GLWidget::setFogDensity(double _fogDensity) {
  fogDensity = _fogDensity;
  update();
}

void GLWidget::setColorProperty(int colorProperty) {
  listboxColorVal = colorProperty;
  mapColorToValue(42);
  if (autoRecalculateColor) {
    recalculateColor();
  }
  update();
}

void GLWidget::setColorMinimumValue(double colorMinimum) {
  if (mincolor_value != colorMinimum) {
    mincolor_value = colorMinimum;
    minmaxChanged(0);
    emit colorMinimumValueChanged(colorMinimum);
    update();
  }
}

void GLWidget::setColorMaximumValue(double colorMaximum) {
  if (maxcolor_value != colorMaximum) {
    maxcolor_value = colorMaximum;
    minmaxChanged(0);
    emit colorMaximumValueChanged(colorMaximum);
    update();
  }
}

void GLWidget::setAutoRecalculateColor(bool autoRecalculate) {
  autoRecalculateColor = autoRecalculate;
  if (autoRecalculateColor) {
    recalculateColor();
  }
}

void GLWidget::recalculateColor() {
  double temp_mincolor_value = mincolor_value;
  double temp_maxcolor_value = maxcolor_value;

  resetMinMax(0);

  if (temp_mincolor_value != mincolor_value) {
    emit colorMinimumValueChanged(mincolor_value);
    update();
  }

  if (temp_maxcolor_value != maxcolor_value) {
    emit colorMaximumValueChanged(maxcolor_value);
    update();
  }
}

void GLWidget::setColorType(int colorType) {
  colorScanVal = colorType;
  setScansColored(0);
  update();
}

void GLWidget::setColorMap(int colorMap) {
  listboxColorMapVal = colorMap;
  changeColorMap(0);
  update();
}

void GLWidget::setColorInverted(bool colorInverted) {
  invert = !colorInverted;
  update();
}

void GLWidget::setAnimatePath(bool animatePath) {
  if (animatePath != this->animatePath) {
    this->animatePath = animatePath;
    emit animationPossible(animateMatching || animatePath);
  }
}

void GLWidget::setPathFileName(QString pathFileName) {
  this->pathFileName = pathFileName;
}

void GLWidget::setInterpolationFactor(double interpolationFactor) {
  path_interp_factor = interpolationFactor;
}

void GLWidget::setInterpolateByDistance(bool interpolateByDistance) {
  inter_by_dist = interpolateByDistance;
  updateCamera();
}

void GLWidget::setAnimateMatching(bool animateMatching) {
  if (animateMatching != this->animateMatching) {
    this->animateMatching = animateMatching;
    emit animationPossible(animateMatching || animatePath);
  }
}

void GLWidget::setAnimationSpeed(double animationSpeed) {
  anim_delay = (int)(1000.0 / animationSpeed);
}

void GLWidget::animate() {
  startAnimation(0);
  // FIXME need to update() with every frame
}

void GLWidget::setSnapshotScale(int snapshotScale) {
  factor = snapshotScale;
}

void GLWidget::saveSnapshot() {
  saveImage(0);
}
