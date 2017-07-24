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
  } catch (std::exception& e) {
    std::cerr << "Error while parsing settings: " << e.what() << endl;
    exit(1);
  }

  mainWindow = new MainWindow(ds, ws);

  if (has_initial_directory) {
    initShow(ds, ws);
    mainWindow->addRecentDirectory();
  } else {
    // doing DrawUrl without initShow hangs the program for ~10s
    label = false;
  }

  update_callback = [&]() {
    mainWindow->glWidget->update();
  };

  loading_status = [&](const std::string& message) {
    mainWindow->statusbar->showMessage(QString::fromStdString(message));
  };

  loading_progress = [&](int current, int min, int max) {
    mainWindow->progressbar->setMinimum(min);
    mainWindow->progressbar->setMaximum(max);
    mainWindow->progressbar->setValue(current);
  };

  connect(mainWindow, &MainWindow::scanDirectoryOpened, this, &QtShow::loadDifferentScan);

  mainWindow->show();
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
  cams.clear();
  path_vectorX.clear();
  path_vectorZ.clear();
  lookats.clear();
  lookat_vectorX.clear();
  lookat_vectorZ.clear();
  ups.clear();
  ups_vectorX.clear();
  ups_vectorZ.clear();

  ds.distance_filter.min = 0;
  ds.distance_filter.max = -1;
  ds.octree_reduction_voxel = 0;
  ds.octree_reduction_randomized_bucket = 1;
  ds.skip_files = 1;

  // actual switching, includes callbacks to the progress bar
  initShow(ds, ws);
}
