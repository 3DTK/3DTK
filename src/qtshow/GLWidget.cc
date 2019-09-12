#include <cmath>

#ifdef _MSC_VER
#include "windows.h"
#endif

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include <GL/glu.h>
#endif

#include <QApplication>
#include <Qt>
#include <QFileDialog>

#include "show/callbacks_glut.h"
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
  cameraChanged();

  emit zoomValueChanged(cangle);
}

void GLWidget::resizeGL(int w, int h) {
  callbacks::glut::reshape(w, h);
}

void GLWidget::paintGL() {
  // Copy-paste from callbacks::glut::display() TODO unify this
  if (fabs(cangle_old - cangle) > 0.5 ||
      fabs(pzoom_old - pzoom) > 0.5) {

    cangle_old = cangle;
    pzoom_old = pzoom;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    callbacks::glut::reshape(viewport[2],viewport[3]);
  }

  glDrawBuffer(buffermode);

  // Call the display function
  DisplayItFunc(GL_RENDER, false);
}

/** Processes a mouse event for the legacy parts of show. */
void passMouseEvent(QMouseEvent *event, bool press) {
  int button = -1;
  selectOrunselect = true;
  // let's hope that GLUT never changes its button numbers
  if (event->button() == Qt::LeftButton) {
    button = 0; // GLUT_LEFT_BUTTON
  } else if (event->button() == Qt::MiddleButton) {
    button = 1; // GLUT_MIDDLE_BUTTON
  } else if (event->button() == Qt::RightButton) {
    if (!cameraNavMouseMode)
      selectOrunselect = false;
    button = 2; // GLUT_RIGHT_BUTTON
  }

  if (button != -1) {
    callbacks::glut::mouseButton(button, press ? 0 : 1, event->x(), event->y());
  }
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
  if (captureMouseCursor) {
    // Remember the cursor position
    initialMousePos = mapToGlobal(event->pos());

    // Reset cursor to center
    QPoint center(width() / 2, height() / 2);
    QCursor::setPos(mapToGlobal(center));

    // Hide the cursor
    setCursor(Qt::BlankCursor);

    // Trick the legacy code
    mousePresX = width() / 2;
    mousePresY = height() / 2;
  }

  passMouseEvent(event, true);
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->buttons() == Qt::NoButton && captureMouseCursor) {
    // Restore cursor position
    QCursor::setPos(initialMousePos);

    // Show the cursor
    unsetCursor();
  }

  passMouseEvent(event, false);
}

/** Notify interested widgets of the camera position. */
void GLWidget::cameraChanged() {
  emit positionChanged(QString::asprintf("%.5g,%.5g,%.5g", X, Y, Z));
  emit quaternionChanged(QString::asprintf("%.3g,%.3g,%.3g,%.3g", quat[0], quat[1], quat[2], quat[3]));
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  if (captureMouseCursor) {
    int dx =  width()/2 - event->x(),
        dy = height()/2 - event->y();

    mouseNavX += dx;
    mouseNavY += dy;

    callbacks::glut::mouseMoveDelta(dx, dy);

    QPoint center(width() / 2, height() / 2);
    QCursor::setPos(mapToGlobal(center));
  } else {
    callbacks::glut::mouseMove(event->x(), event->y());
  }

  update();
  cameraChanged();
}

void GLWidget::keyPressEvent(QKeyEvent *event) {
  bool ctrl  = (event->modifiers() & Qt::ControlModifier) != 0,
       alt   = (event->modifiers() & Qt::AltModifier)     != 0,
       shift = (event->modifiers() & Qt::ShiftModifier)   != 0;

  cameraNavMouseMode = !shift;

  // Translate the Qt keycodes inside event->key() to the keycodes
  // used by show with this map. It accepts a combination of ASCII
  // and wxWidgets keycodes (c.f. callbacks::glut::keyHandler()).
  std::map<int, int> qt_to_show_keycodes {
    {Qt::Key_W,        'w'},
    {Qt::Key_A,        'a'},
    {Qt::Key_S,        's'},
    {Qt::Key_D,        'd'},
    {Qt::Key_C,        'c'},
    {Qt::Key_Space,    ' '},
    {Qt::Key_Left,     314},
    {Qt::Key_Up,       315},
    {Qt::Key_Right,    316},
    {Qt::Key_Down,     317},
    {Qt::Key_Q,        'q'},
    {Qt::Key_PageUp,   'q'},
    {Qt::Key_E,        'e'},
    {Qt::Key_PageDown, 'e'}
  };

  auto kc_it = qt_to_show_keycodes.find(event->key());
  if (event->key() == Qt::Key_F) {
    // We do not want fullscreen handled by GLUT (wouldn't work anyway)
    setFullscreen(!fullscreen);
  } else if (kc_it != qt_to_show_keycodes.end()) {
    // Map has key: pass it to legacy key handler
    callbacks::glut::keyHandler(kc_it->second, ctrl, alt, shift);
  } else {
    // Map lacks key: Need to pass the event to parent class if we
    // don't handle it (according to Qt docs).
    QOpenGLWidget::keyPressEvent(event);
  }

  update();
  cameraChanged();
}

void GLWidget::keyReleaseEvent(QKeyEvent *event) {
  if(event->isAutoRepeat()) return;

  cameraNavMouseMode = (event->modifiers() & Qt::ShiftModifier) == 0;
}

void GLWidget::setFullscreen(bool fullscreenWanted) {
  // TODO make only this widget fullscreen
  QWidget *grandparent = parentWidget()->parentWidget();
  if (fullscreenWanted) {
    grandparent->showFullScreen();
    emit dockWidgetsHidden();
  } else {
    grandparent->showNormal();
    emit dockWidgetsShown();
  }
  fullscreen = grandparent->isFullScreen();
}

void GLWidget::setDrawPoints(bool drawPoints) {
  show_points = drawPoints;
  update();
}

void GLWidget::setDrawAllPoints(int allPoints) {
  switch(allPoints) {
    case 0:
      pointmode = -1;
      break;
    case 1:
      pointmode =  0;
      break;
    case 2:
      pointmode =  1;
      break;
  }
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

void GLWidget::setDrawPoses(bool drawPoses) {
  show_poses = drawPoses;
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

void GLWidget::addCamera() {
  callAddCamera(1);
  cam_choice = cams.size();
  camsListAdded(cam_choice);
  update();
}

void GLWidget::deleteCamera() {
  callDeleteCamera(0);
  camsListDeleted(cam_choice - 1);
  update();
}

void GLWidget::changeCamCombobox(int index) {
  cam_choice = index + 1;
  update();
}

void GLWidget::setSnapshotScale(int snapshotScale) {
  factor = snapshotScale;
}

void GLWidget::setViewMode(int mode) {
  if(mode == 0) {
    if(showViewMode == 1) topView();
    else if(showViewMode == 2) rotateView();
    emit zoomValueChanged(cangle);
  }
  else if(mode == 1) {
    if(showViewMode == 2) rotateView();
    topView();
    emit zoomValueChanged(pzoom);
  }
  else {
    if(showViewMode == 1) topView();
    rotateView();
    emit zoomValueChanged(rzoom);
  }
}

void GLWidget::setZoom(double zoom) {
  if(showViewMode == 1) {
    pzoom = zoom;
  }
  else if(showViewMode == 2) {
    rzoom = zoom;
  }
  else {
    cangle = zoom;
  }
  update();
}

void GLWidget::resetPosition() {
  makeCurrent();
  resetView(0);
}

void GLWidget::saveSnapshot() {
  QWidget *mainWindow = parentWidget()->parentWidget();
  QString imageFileName = QFileDialog::getSaveFileName(
    mainWindow, "Save Snapshot", suggestImageFileName().c_str(),
    "PPM Files (*.ppm);;All Files (*)");

  if (!imageFileName.isEmpty()) {
    makeCurrent();
    saveImageAt(imageFileName.toStdString());
    emit status("Snapshot saved as \"" + imageFileName + "\"", 5000);
  }
}

void GLWidget::resetSelection() {
  clearSelection(0);
  update();
}

void GLWidget::setSelectVoxels() {
  select_voxels = !select_voxels;
}

void GLWidget::idle() {
  makeCurrent();
  callbacks::glut::idle();
}
