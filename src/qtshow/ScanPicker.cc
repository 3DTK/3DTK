#include <QFileDialog>

#include "qtshow/ScanPicker.h"


ScanPicker::ScanPicker(dataset_settings& dss, QWidget *parent, Qt::WindowFlags f)
  : QDialog(parent, f)
  , dss(dss)
{
  setupUi(this);
  lineEditDirectory->setText(QString::fromStdString(dss.data_source));
}

void ScanPicker::accept() {
  dss.data_source = lineEditDirectory->text().toStdString();
  // TODO gracefully alert the user if the chosen format does not exist
  dss.format = formatname_to_io_type(comboBoxFormat->currentText().toStdString().c_str());
  dss.scan_numbers.min = spinBoxStart->value();
  dss.scan_numbers.max = spinBoxEnd->value();
  dss.scale = spinBoxScale->value();

  QDialog::accept();
}

void ScanPicker::chooseScanDirectory() {
  QString dir = QFileDialog::getExistingDirectory(this, "Choose Scan Directory…",
    lineEditDirectory->text());
  lineEditDirectory->setText(dir);
}
