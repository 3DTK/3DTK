#include <algorithm>
#include <fstream>

#include <QFileDialog>

#include "qtshow/MainWindow.h"
#include "qtshow/ScanPicker.h"

MainWindow::MainWindow(const dataset_settings& ds, const window_settings& ws, QWidget *parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags)
  , ds(ds)
{
  if (!ws.nogui) {
    setupUi(this);

    progressbar = new QProgressBar(statusbar);
    progressbar->hide();
    statusbar->addPermanentWidget(progressbar);

    updateRecentDirectoriesMenu(loadRecentDirectories());

    connect(this, &MainWindow::scanDirectoryOpened, this, &MainWindow::addRecentDirectory);
  } else {
    glWidget = new GLWidget(this);
    glWidget->setFocusPolicy(Qt::ClickFocus);
    setCentralWidget(glWidget);
    resize(ws.dimensions.w, ws.dimensions.h);
  }
}

void MainWindow::updateRecentDirectoriesMenu(std::vector<std::string> directories) {
  menuOpenRecent->clear();
  for (auto directory : loadRecentDirectories()) {
    QAction *action = menuOpenRecent->addAction(
      QString::fromStdString(directory), this,
      SLOT(openRecentDirectory()));
      // TODO uncomment this when Andreas upgrades to Qt 5.6
      // &MainWindow::openRecentDirectory);
    action->setData(QString::fromStdString(directory));
  }
  menuOpenRecent->addSeparator();
  menuOpenRecent->addAction(actionClearRecent);
}

void MainWindow::openScanDirectory() {
  ScanPicker sp(ds, this);
  if (sp.exec())
    emit scanDirectoryOpened(ds);
}

void MainWindow::openRecentDirectory() {
  QAction *action = qobject_cast<QAction *>(sender());
  if (action) {
    ds.input_directory = action->data().toString().toStdString();
    emit scanDirectoryOpened(ds);
  }
}

std::string recent_directory_path() {
  std::string config_home;
#ifndef _MSC_VER
  // in $XDG_CONFIG_HOME/3dtk/qtshow-recent.txt
  char *home_c = getenv("HOME");
  char *config_home_c = getenv("XDG_CONFIG_HOME");
  if (config_home_c && *config_home_c != '\0') {
    config_home = config_home_c;
  } else {
    config_home = std::string(home_c) + "/.config";
  }
#else
  // in %APPDATA%/3dtk/qtshow-recent.txt
  config_home = std::string(getenv("APPDATA"));
#endif

  return (config_home + "/3dtk/qtshow-recent.txt").c_str();
}

void MainWindow::addRecentDirectory() {
  std::vector<std::string> directories = loadRecentDirectories();

  auto duplicate = std::find(directories.begin(), directories.end(), ds.input_directory);
  if (duplicate != directories.end()) {
    directories.erase(duplicate);
  }
  directories.insert(directories.begin(), ds.input_directory);

  std::ofstream directories_file(recent_directory_path());

  // Only remember 10 most recent directories
  for (auto directoryIt = directories.begin();
       directoryIt != directories.end()
         && directoryIt != directories.begin() + 10;
       directoryIt++) {
    directories_file << *directoryIt << std::endl;
  }

  updateRecentDirectoriesMenu(directories);
}

std::vector<std::string> MainWindow::loadRecentDirectories() {
  std::vector<std::string> result;

  // Read recent directories file
  std::ifstream directories_file(recent_directory_path());

  std::string line;
  while (std::getline(directories_file, line)) {
    if (!line.empty()) {
      result.push_back(line);
    }
  }

  directories_file.close();

  return result;
}

void MainWindow::clearRecentDirectories() {
  std::ofstream directories_file(recent_directory_path());
  directories_file.close();
  updateRecentDirectoriesMenu({});
}
