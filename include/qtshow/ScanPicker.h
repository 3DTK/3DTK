#ifndef __SCANPICKER_H__
#define __SCANPICKER_H__

#include "ui_ScanPicker.h"

#include "show/program_options.h"

class ScanPicker : public QDialog, private Ui::ScanPicker {
  Q_OBJECT

public:
  ScanPicker(dataset_settings& dss, QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

public slots:
  void accept();
  void chooseScanDirectory();

protected:
  dataset_settings& dss;
};

#endif
