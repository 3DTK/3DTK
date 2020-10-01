#include "qtshow/QtShow.h"
#include "show/show_common.h"
#include "show/program_options.h"
#include "slam6d/io_types.h"

#if defined(SPACEMOUSE) && defined(__APPLE__)
static MainWindow *mainWindow;
static uint16_t clientID;
static bool fixRotation = false;
static bool fixTranslation = false;
static float translationMultiplier = 0.1;
static float rotationMultiplier = 0.01;
#endif

QtShow::QtShow(int &argc, char **argv)
  : QApplication(argc, argv)
{
  // Qt wants to automatically set the locale for us (c.f. Qt bugs #15247 #10994).
  // This is a problem for scanf for comma-separated lists of numbers when e.g. LC_NUMERIC=de_DE
  // We reset this behaviour to POSIX before we execute any of our own code:
  setlocale(LC_NUMERIC, "POSIX");

  glutInit(&argc, argv);

  bool has_initial_directory;
  try {
    parse_args(argc, argv, dss, ws, ds, &has_initial_directory);
  } catch (std::exception& e) {
    std::cerr << "Error while parsing settings: " << e.what() << std::endl;
    exit(1);
  }
#ifdef SPACEMOUSE
  mainWindow = new MainWindow(dss, ws, ds, &translationMultiplier, &rotationMultiplier);
#else
    mainWindow = new MainWindow(dss, ws, ds);
#endif
  mainWindow->show();
  if (has_initial_directory) {
    initShow(dss, ws, ds);
    mainWindow->addRecentDirectory();
  } else {
    // doing DrawUrl without initShow hangs the program for ~10s
    label = false;
  }

  // Set spin box values for advanced mode
  mainWindow->spinBoxFrame->setValue(current_frame);
  mainWindow->spinBoxFrame->setMaximum(current_frame);
  mainWindow->doubleSpinBoxFps->setValue(idealfps);
  mainWindow->doubleSpinBoxFarPlane->setValue(fardistance);
  mainWindow->doubleSpinBoxNearPlane->setValue(neardistance);
  mainWindow->doubleSpinBoxLodSpeed->setValue(adaption_rate);
  mainWindow->doubleSpinBox3dShift->setValue(shifted);
  mainWindow->spinBoxStartIndex->setValue(startScanIdx);
  mainWindow->spinBoxStartIndex->setMinimum(startScanIdx);
  mainWindow->spinBoxStartIndex->setMaximum(endScanIdx);
  mainWindow->spinBoxEndIndex->setValue(endScanIdx);
  mainWindow->spinBoxEndIndex->setMinimum(startScanIdx);
  mainWindow->spinBoxEndIndex->setMaximum(endScanIdx);

  if(advanced_controls)
    mainWindow->dockAdvanced->setFloating(true);
  else
    mainWindow->dockAdvanced->hide();

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

  //mainWindow->show();

  // After 1 second (and next GLWidget::update()) we can have the watermark back on
  QTimer::singleShot(1000, []() {
    label = true;
  });
#ifdef SPACEMOUSE
    #ifndef __APPLE__
    spnav_controller = new SpaceNavController(mainWindow->glWidget, &translationMultiplier, &rotationMultiplier);
    #else
    int16_t result = SetConnexionHandlers(&Connexion3DMouseMessageHandler, nullptr, nullptr, true);
    clientID = RegisterConnexionClient(kConnexionClientWildcard, (uint8_t*)"qtshow",kConnexionClientModeTakeOver, kConnexionMaskAll);
    SetConnexionClientButtonMask(clientID, kConnexionMaskAllButtons);
    if(result == 0){
        std::cout << "found and activate 3DConnexion mouse" << std::endl;
    }
    #endif
#endif
}

QtShow::~QtShow()
{
#if defined(SPACEMOUSE) && defined(__APPLE__)
    UnregisterConnexionClient(clientID);
    CleanupConnexionHandlers();
#endif
  delete mainWindow;
}

#if defined(SPACEMOUSE) && defined(__APPLE__)
void QtShow::Connexion3DMouseMessageHandler(unsigned int connection, unsigned int messageType, void *messageArgument)
{
    //std::cout << "messageHandler" << std::endl;
    ConnexionDeviceState *state;
    switch (messageType)
    {
        case kConnexionMsgDeviceState:
            state = (ConnexionDeviceState*)messageArgument;
            //std::cout << "sender client id: "<< state->client << std::endl;
            if (state->client == clientID)
            {
                // decipher what command/event is being reported by the driver
                switch (state->command)
                {
                    case kConnexionCmdHandleAxis:
                        //std::cout << "move axis" << std::endl;
                        // state->axis will contain values for the 6 axis
                        if(!fixRotation && !fixTranslation) {
                            mainWindow->glWidget->spaceNavEvent(-state->axis[0] * translationMultiplier,
                                                                state->axis[2] * translationMultiplier,
                                                                -state->axis[1] * translationMultiplier,
                                                                -state->axis[3] * rotationMultiplier,
                                                                -state->axis[5] * rotationMultiplier,
                                                                state->axis[4] * rotationMultiplier);
                        } else if(fixRotation && !fixTranslation){
                            mainWindow->glWidget->spaceNavEvent(-state->axis[0] * translationMultiplier,
                                                                state->axis[2] * translationMultiplier,
                                                                -state->axis[1] * translationMultiplier,
                                                                0,
                                                                0,
                                                                0);
                        } else if(!fixRotation && fixTranslation){
                            mainWindow->glWidget->spaceNavEvent(0,
                                                                0,
                                                                0,
                                                                -state->axis[3] * rotationMultiplier,
                                                                -state->axis[5] * rotationMultiplier,
                                                                state->axis[4] * rotationMultiplier);
                        }
                        break;
                    case kConnexionCmdHandleButtons:
                        // state->buttons reports the buttons that are pressed
                        //std::cout << "pressed buttons" << state->buttons << std::endl;
                        switch (state->buttons) {
                            case 1:
                                fixRotation = !fixRotation;
                                fixTranslation = false;
                                break;
                            case 2:
                                fixTranslation = !fixTranslation;
                                fixRotation = false;
                                break;
                            default:
                                //std::cout << "unsupported button: " << state->buttons << std::endl;
                                break;
                        }
                        break;
                }
            }
            break;
        default:
            // other messageTypes can happen and should be ignored
            break;
    }
}
#endif

void QtShow::loadDifferentScan(dataset_settings new_dss) {
  dss = new_dss;

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

  dss.distance_filter.min = 0;
  dss.distance_filter.max = -1;
  dss.octree_reduction_voxel = 0;
  dss.octree_reduction_randomized_bucket = 1;
  dss.skip_files = 1;

  // actual switching, includes callbacks to the progress bar
  initShow(dss, ws, ds);
  mainWindow->glWidget->update();
}
