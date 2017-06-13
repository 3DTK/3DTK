#include "qtshow/MainWindow.h"
#include "show/show_common.h"

MainWindow::MainWindow(const window_settings& ws, QWidget *parent)
  : QMainWindow(parent)
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

void MainWindow::pickScanDirectory() {
  
}
