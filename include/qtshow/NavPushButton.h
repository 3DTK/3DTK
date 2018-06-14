#ifndef __NAVPUSHBUTTON_H__
#define __NAVPUSHBUTTON_H__

#include <QPushButton>

class NavPushButton : public QPushButton {
  Q_OBJECT

public:
  NavPushButton(QWidget *parent = 0);

  void setType(int t);

protected:
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);

private:
  QPoint oldPos;
  Qt::CursorShape cursor;
  int type;
};

#endif
