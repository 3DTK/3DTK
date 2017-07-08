#include <QFileDialog>

#include "qtshow/MainWindow.h"
#include "qtshow/ScanPicker.h"

MainWindow::MainWindow(const dataset_settings& ds, const window_settings& ws, QWidget *parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags)
  , ds(ds)
{
  if (!ws.nogui) {
    setupUi(this);
  } else {
    glWidget = new GLWidget(this);
    glWidget->setFocusPolicy(Qt::ClickFocus);
    setCentralWidget(glWidget);
    resize(ws.dimensions.w, ws.dimensions.h);
  }

  progressbar = new QProgressBar(statusbar);
  statusbar->addPermanentWidget(progressbar);
}

void MainWindow::openScanDirectory() {
  ScanPicker sp(ds, this);
  if (sp.exec())
    emit scanDirectoryOpened(ds);
}
