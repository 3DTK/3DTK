#include <QFileDialog>

#include "qtshow/MainWindow.h"
#include "qtshow/ScanPicker.h"

MainWindow::MainWindow(const window_settings& ws, QWidget *parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags)
{
  if (!ws.nogui) {
    setupUi(this);
  } else {
    glWidget = new GLWidget(this);
    glWidget->setFocusPolicy(Qt::ClickFocus);
    setCentralWidget(glWidget);
    resize(ws.dimensions.w, ws.dimensions.h);
  }
}

void MainWindow::openScanDirectory() {
  ScanPicker sp(".", this);
  connect(&sp, &ScanPicker::scanPicked, this, &MainWindow::scanPicked);
  sp.exec();
}

void MainWindow::scanPicked(QString dir, QString format, int start, int end, double scale) {
  emit scanDirectoryOpened(dir, format, start, end, scale);
}
