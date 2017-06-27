#ifndef __SCANPICKER_H__
#define __SCANPICKER_H__

#include <QDialog>

#include "ui_ScanPicker.h"

class ScanPicker : public QDialog, private Ui::ScanPicker {
  Q_OBJECT

public:
  ScanPicker(const QString& current_directory, QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());

signals:
  void scanPicked(QString dir, QString format, int start, int end, double scale);

public slots:
  void accept();
  void chooseScanDirectory();
};

#endif
