#include <QFileDialog>

#include "qtshow/ScanPicker.h"


ScanPicker::ScanPicker(dataset_settings& ds, QWidget *parent, Qt::WindowFlags f)
  : QDialog(parent, f)
  , ds(ds)
{
  setupUi(this);
  lineEditDirectory->setText(QString::fromStdString(ds.input_directory));
}

void ScanPicker::accept() {
  ds.input_directory = lineEditDirectory->text().toStdString();
  // TODO gracefully alert the user if the chosen format does not exist
  ds.format = formatname_to_io_type(comboBoxFormat->currentText().toStdString().c_str());
  ds.scan_numbers.min = spinBoxStart->value();
  ds.scan_numbers.max = spinBoxEnd->value();
  ds.scale = spinBoxScale->value();

  QDialog::accept();
}

void ScanPicker::chooseScanDirectory() {
  QString dir = QFileDialog::getExistingDirectory(this, "Choose Scan Directory…",
    lineEditDirectory->text());
  lineEditDirectory->setText(dir);
}
