#ifndef __SETTINGSDIALOG_H__
#define __SETTINGSDIALOG_H__

#include "ui_SettingsDialog.h"

//#include "show/program_options.h"

class SettingsDialog : public QDialog, public Ui::SettingsDialog {
  Q_OBJECT

public:
  SettingsDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

};

#endif
