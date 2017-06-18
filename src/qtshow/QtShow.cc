#include "qtshow/QtShow.h"
#include "show/show_common.h"
#include "show/program_options.h"

QtShow::QtShow(int &argc, char **argv)
  : QApplication(argc, argv)
{
  glutInit(&argc, argv);

  try {
    parse_args(argc, argv, ds, ws);
  } catch (std::exception e) {
    std::cerr << "Error while parsing settings: " << e.what() << endl;
    exit(1);
  }

  initShow(ds, ws);

  mainWindow = new MainWindow(ws);
  mainWindow->show();

  connect(mainWindow, &MainWindow::scanDirectoryChanged, this, &QtShow::loadDifferentScans);
}

void QtShow::loadDifferentScans(QString dir) {
  // TODO turn this into proper context handling logic for show
  Scan::closeDirectory();
  for (colordisplay* tree : octpts) {
    delete tree;
  }
  octpts.clear();
  ds.input_directory = dir.toStdString();
  ds.scale = 1.0;
  initShow(ds, ws);
}
