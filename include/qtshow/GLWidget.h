#ifndef __GLWIDGET_H__
#define __GLWIDGET_H__

#include <QMouseEvent>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QTimer>
#include <QWidget>

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  GLWidget(QWidget *parent = 0);

public slots:
  void setFullscreen(bool fullscreenWanted);
  void setDrawPoints(bool drawPoints);
  void setDrawCameras(bool drawCameras);
  void setDrawPath(bool drawPath);
  void setDrawPoses(bool drawPoses);
  void setPointSize(int pointSize);
  void setDrawAllPoints(int allPoints);

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

  void setAnimatePath(bool animatePath);
  void saveCameraPath();
  void loadCameraPath();
  void drawPath();
  void setInterpolationFactor(double interpolationFactor);
  void setInterpolateByDistance(bool interpolateByDistance);
  void setAnimateMatching(bool animateMatching);
  void setAnimationSpeed(double animationSpeed);
  void animate();

  void addCamera();
  void deleteCamera();
  void changeCamCombobox(int index);

  void setViewMode(int mode);
  void setZoom(double zoom);
  void resetPosition();

  void setSnapshotScale(int snapshotScale);
  void saveSnapshot();

  void resetSelection();
  void setSelectVoxels();

  void idle();

  void cameraChanged();

signals:
  void positionChanged(QString);
  void quaternionChanged(QString);

  void colorMinimumValueChanged(double);
  void colorMaximumValueChanged(double);

  void animationPossible(bool);

  void status(QString, int);

  void camsListAdded(int index);
  void camsListDeleted(int index);

  void zoomValueChanged(double);

  void dockWidgetsHidden();
  void dockWidgetsShown();

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

  void mousePressEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void keyPressEvent(QKeyEvent *event);
  void keyReleaseEvent(QKeyEvent *event);

  void updateFogType();

protected:
  QPoint initialMousePos;

  bool fullscreen = false;

  int fogType;
  bool fogInverted = false;

  bool autoRecalculateColor = false;

  bool animatePath = true;
  bool animateMatching = false;

  QTimer *idleTimer;
};

#endif
