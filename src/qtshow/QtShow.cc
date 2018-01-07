#include "qtshow/QtShow.h"
#include "show/show_common.h"
#include "show/program_options.h"
#include "slam6d/io_types.h"

QtShow::QtShow(int &argc, char **argv)
  : QApplication(argc, argv)
{
  // Qt wants to automatically set the locale for us (c.f. Qt bugs #15247 #10994).
  // This is a problem for sscanf for comma-separated lists of numbers when e.g. LC_NUMERIC=de_DE
  // We reset this behaviour to POSIX before we execute any of our own code:
  setlocale(LC_NUMERIC, "POSIX");

  glutInit(&argc, argv);

  bool has_initial_directory;
  try {
    parse_args(argc, argv, ds, ws, &has_initial_directory);
    // FIXME in advanced mode, only scan 1 is drawn
    ws.advanced_controls = false;
  } catch (std::exception& e) {
    std::cerr << "Error while parsing settings: " << e.what() << std::endl;
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
    if (!nogui) {
      mainWindow->statusbar->showMessage(QString::fromStdString(message));
    }
  };

  loading_progress = [&](int current, int min, int max) {
    QProgressBar *pb = mainWindow->progressbar;
    if (!nogui) {
      if (min <= max) {
        pb->show();
        pb->setMinimum(min);
        pb->setMaximum(max);
        pb->setValue(current);
      } else { // max > min is code for "done"
        pb->hide();
      }
    }
  };

  connect(mainWindow, &MainWindow::scanDirectoryOpened, this, &QtShow::loadDifferentScan);

  mainWindow->show();

  // After 1 second (and next GLWidget::update()) we can have the watermark back on
  QTimer::singleShot(1000, []() {
    label = true;
  });
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
  mainWindow->glWidget->update();
}
