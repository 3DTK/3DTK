#include "qtshow/NavPushButton.h"

#include <QApplication>
#include <QMouseEvent>
#include <QDebug>

#include "show/show_common.h"

NavPushButton::NavPushButton(QWidget *parent)
  : QPushButton(parent)
{
}

void NavPushButton::setType(int t) {
  type = t;
  switch(type) {
    case 0: // Move X
      cursor = Qt::SizeHorCursor;
      break;
    case 1: // Move Y
      cursor = Qt::SizeVerCursor;
      break;
    case 2: // Move Z
      cursor = Qt::SizeVerCursor;
      break;
  }
}

void NavPushButton::mousePressEvent(QMouseEvent *e) {
  QPushButton::mousePressEvent(e);
  if(e->button() == Qt::LeftButton) {
    QApplication::setOverrideCursor(cursor);
    oldPos = e->pos();
  }
}

void NavPushButton::mouseMoveEvent(QMouseEvent *e) {
  switch(type) {
    case 0: // Move X
      X += (e->pos() - oldPos).x();
      break;
    case 1: // Move Y
      Y -= (e->pos() - oldPos).y();
      break;
    case 2: // Move Z
      Z -= (e->pos() - oldPos).y();
      break;
  }
  oldPos = e->pos();
  parentWidget()->parentWidget()->parentWidget()->update();
}

void NavPushButton::mouseReleaseEvent(QMouseEvent *e) {
  QPushButton::mouseReleaseEvent(e);
  QApplication::restoreOverrideCursor();
}
