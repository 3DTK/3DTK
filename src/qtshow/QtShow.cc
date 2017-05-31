#include "qtshow/QtShow.h"
#include "show/show_common.h"

QtShow::QtShow(int &argc, char **argv)
  : QApplication(argc, argv)
{
  initShow(argc, argv);
  mainWindow.show();
}
