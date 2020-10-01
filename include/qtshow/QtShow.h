#ifndef __QTSHOW_H__
#define __QTSHOW_H__

#include <QApplication>

#include "qtshow/MainWindow.h"
#ifdef SPACEMOUSE
    #ifdef __APPLE__
        #include "ConnexionClientAPI.h"
        #include "ConnexionClient.h"
    #else
        #include "qtshow/spnav_controller.h"
    #endif
#endif

class QtShow : public QApplication {
  Q_OBJECT

public:
  QtShow(int &argc, char **argv);
  ~QtShow() override;
#if defined(SPACEMOUSE) && defined(__APPLE__)
    static void Connexion3DMouseMessageHandler(unsigned int productID, unsigned int messageType, void *messageArgument);
#endif
//    void MyAddedMessageHandler(unsigned int connection);
//    void MyRemovedMessageHandler(unsigned int connection);

public slots:
  void loadDifferentScan(dataset_settings dss);

protected:
#if !(defined(SPACEMOUSE) && defined(__APPLE__))
  MainWindow *mainWindow;
#endif
  dataset_settings dss;
  window_settings ws;
  display_settings ds;
#if defined(SPACEMOUSE) && !defined(__APPLE__)
  SpaceNavController *spnav_controller;
private:
  float translationMultiplier = 0.1;
  float rotationMultiplier = 0.01;
#endif
};

#endif
