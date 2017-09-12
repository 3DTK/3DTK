#include <cmath>

#ifdef _MSC_VER
#include "windows.h"
#endif

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include <GL/glu.h>
#endif
#include <Qt>
#include <QFileDialog>

#include "show/show_common.h"
#include "show/show_gl.h"
#include "qtshow/GLWidget.h"

GLWidget::GLWidget(QWidget *parent)
  : QOpenGLWidget(parent)
{
  // a timer with the default timeout of 0 runs whenever its thread's event loop is idle, which is pretty much when we want to call CallBackIdleFunc
  idleTimer = new QTimer(this);
  connect(idleTimer, &QTimer::timeout, this, &GLWidget::idle);
  idleTimer->start();
}

void GLWidget::initializeGL() {
  initializeOpenGLFunctions();
  load_url_texture();
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
  initialMousePos = mapToGlobal(event->pos());
  QPoint center(width() / 2, height() / 2);
  QCursor::setPos(mapToGlobal(center));
  setCursor(Qt::BlankCursor);
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->buttons() == 0) {
    QCursor::setPos(initialMousePos);
    unsetCursor();
  }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  float dx = event->x() - width()/2;
  float dy = event->y() - height()/2;

  // TODO different speeds for topView
  if (event->buttons() & Qt::LeftButton) {
    moveCamera(0, 0, 0, dy, -dx, 0);
  } else if (event->buttons() & Qt::MidButton) {
    dx *= movementSpeed/10.0;
    dy *= movementSpeed/10.0;
    moveCamera(dx, 0, dy, 0, 0, 0);
  } else if (event->buttons() & Qt::RightButton) {
    dx *= movementSpeed/10.0;
    dy *= movementSpeed/10.0;
    moveCamera(dx, -dy, 0, 0, 0, 0);
  }

  update();

  QPoint center(width() / 2, height() / 2);
  QCursor::setPos(mapToGlobal(center));
}

void GLWidget::keyPressEvent(QKeyEvent *event) {
  // TODO include the time delta
  double stepsize = movementSpeed;

  /* FIXME holding a modifier somehow hangs up the application
  if (event->modifiers() & Qt::ControlModifier) stepsize *= 0.1;
  if (event->modifiers() & Qt::ShiftModifier) stepsize *= 10;
  if (event->modifiers() & Qt::AltModifier) stepsize *= 100;
  */

  double rotsize = 0.2 * stepsize;

  switch (event->key()) {
  case Qt::Key_W:
   moveCamera(0,0,stepsize,0,0,0);
   break;
  case Qt::Key_A:
    moveCamera(stepsize,0,0,0,0,0);
    break;
  case Qt::Key_S:
    moveCamera(0,0,-stepsize,0,0,0);
    break;
  case Qt::Key_D:
    moveCamera(-stepsize,0,0,0,0,0);
    break;
  case Qt::Key_C:
    moveCamera(0,stepsize,0,0,0,0);
    break;
  case Qt::Key_Space:
    moveCamera(0,-stepsize,0,0,0,0);
    break;
  case Qt::Key_Left:
    moveCamera(0,0,0,0,rotsize,0);
    break;
  case Qt::Key_Up:
    moveCamera(0,0,0,-rotsize,0,0);
    break;
  case Qt::Key_Right:
    moveCamera(0,0,0,0,-rotsize,0);
    break;
  case Qt::Key_Down:
    moveCamera(0,0,0,rotsize,0,0);
    break;
  case Qt::Key_Q:
  case Qt::Key_PageUp:
    moveCamera(0,0,0,0,0,rotsize);
    break;
  case Qt::Key_E:
  case Qt::Key_PageDown:
    moveCamera(0,0,0,0,0,-rotsize);
    break;
  case Qt::Key_F:
    // TODO does not work
    setWindowState(windowState() ^ Qt::WindowFullScreen);
    break;
  default:
    // need to pass the event to parent class if we don't handle it
    // according to Qt docs
    GLWidget::keyPressEvent(event);
    break;
  }

  update();
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

void GLWidget::saveCameraPath() {
  QWidget *mainWindow = parentWidget()->parentWidget();
  QString pathFileName = QFileDialog::getSaveFileName(
    mainWindow, "Save Camera Path", path_file_name,
    "3DTK Path Files (*.path *.dat);;All Files (*)");
  strncpy(path_file_name, pathFileName.toStdString().c_str(), 1024);
  savePath(0);
}

void GLWidget::loadCameraPath() {
  QWidget *mainWindow = parentWidget()->parentWidget();
  QString pathFileName = QFileDialog::getOpenFileName(
    mainWindow, "Load Camera Path", path_file_name,
    "3DTK Path Files (*.path *.dat);;All Files (*)");
  strncpy(path_file_name, pathFileName.toStdString().c_str(), 1024);
  loadPath(0);
}

void GLWidget::drawPath() {
  drawRobotPath(0);
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
  anim_delay = (int)(5.0 / animationSpeed);
}

void GLWidget::animate() {
  if (animateMatching && animatePath) {
    pathMatchingAnimate(0);
  } else if (animateMatching) {
    startAnimation(0);
  } else if (animatePath) {
    pathAnimate(0);
  }
}

void GLWidget::setSnapshotScale(int snapshotScale) {
  factor = snapshotScale;
}

void GLWidget::saveSnapshot() {
  QWidget *mainWindow = parentWidget()->parentWidget();
  QString imageFileName = QFileDialog::getSaveFileName(
    mainWindow, "Save Snapshot", suggestImageFileName().c_str(),
    "PPM Files (*.ppm);;All Files (*)");

  if (!imageFileName.isEmpty()) {
    saveImageAt(imageFileName.toStdString());
    emit status("Snapshot saved as \"" + imageFileName + "\"", 5000);
  }
}

void GLWidget::idle() {
  CallBackIdleFunc();
}
