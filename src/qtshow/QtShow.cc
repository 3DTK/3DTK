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

  mainWindow = new MainWindow(ws);
  mainWindow->show();

  connect(mainWindow, &MainWindow::scanDirectoryOpened, this, &QtShow::loadDifferentScan);
}

void QtShow::loadDifferentScan(QString dir, QString format, int start, int end, double scale) {
  // TODO turn this into proper context handling logic for show
  // dirty hacks
  Scan::closeDirectory();
  for (colordisplay* tree : octpts) {
    delete tree;
  }
  octpts.clear();

  // actual switching
  ds.input_directory = dir.toStdString();
  // TODO gracefully alert the user if the chosen format does not exist
  ds.format = formatname_to_io_type(format.toStdString().c_str());
  ds.scan_numbers.min = start;
  ds.scan_numbers.max = end;
  ds.scale = scale;
  initShow(ds, ws);
}
