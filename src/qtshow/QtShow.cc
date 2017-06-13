#include "qtshow/QtShow.h"
#include "show/show_common.h"
#include "show/program_options.h"

QtShow::QtShow(int &argc, char **argv)
  : QApplication(argc, argv)
{
  glutInit(&argc, argv);

  dataset_settings ds;
  window_settings ws;

  try {
    parse_args(argc, argv, ds, ws);
  } catch (std::exception e) {
    std::cerr << "Error while parsing settings: " << e.what() << endl;
    exit(1);
  }

  initShow(ds, ws);
  mainWindow.show();
}
