#include "qtshow/QtShow.h"
#include "show/show_common.h"
#include "show/program_options.h"
#include "slam6d/io_types.h"

QtShow::QtShow(int &argc, char **argv)
  : QApplication(argc, argv)
{
  glutInit(&argc, argv);

  bool has_initial_directory;
  try {
    parse_args(argc, argv, ds, ws, &has_initial_directory);
  } catch (std::exception e) {
    std::cerr << "Error while parsing settings: " << e.what() << endl;
    exit(1);
  }

  if (has_initial_directory) {
    initShow(ds, ws);
  }

  mainWindow = new MainWindow(ds, ws);
  mainWindow->show();

  connect(mainWindow, &MainWindow::scanDirectoryOpened, this, &QtShow::loadDifferentScan);
}

void QtShow::loadDifferentScan(dataset_settings new_ds) {
  ds = new_ds;

  // TODO turn this into proper context handling logic for show
  // dirty hacks
  Scan::closeDirectory();
  displays.clear();
  for (colordisplay* tree : octpts) {
    delete tree;
  }
  octpts.clear();
  MetaMatrix.clear();
  MetaAlgoType.clear();
  trajectory.clear();

  // actual switching
  initShow(ds, ws);
}
